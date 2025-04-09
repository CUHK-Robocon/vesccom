#include "vesccom/socketcan.h"

#include <errno.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "bldc/datatypes.h"
#include "boost/crc.hpp"
#include "spdlog/spdlog.h"
#include "vesccom/buf.h"
#include "vesccom/util.h"

namespace vesccom::socketcan {

const uint8_t TO_SLAVE_COMMANDS_PROCESS_PACKET = 0;
const uint8_t MASTER_CONTROLLER_ID = 0;

master::master(const char* device_name) : device_name_(device_name) {
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ == -1)
    throw std::runtime_error("Failed to open SocketCAN socket");

  ifreq ifr;
  std::strcpy(ifr.ifr_name, device_name);
  if (ioctl(socket_, SIOCGIFINDEX, &ifr) == -1) {
    close(socket_);

    throw std::runtime_error("Failed to obtain SocketCAN interface index");
  }

  sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == -1) {
    close(socket_);

    throw std::runtime_error("Failed to bind SocketCAN interface");
  }

  monitor_stop_efd_ = eventfd(0, EFD_NONBLOCK);
  if (monitor_stop_efd_ == -1) {
    close(socket_);

    throw std::runtime_error(
        "Failed to create eventfd for stopping CAN Bus monitoring");
  }
}

master::~master() {
  close(socket_);
  close(monitor_stop_efd_);
}

// Write data to the CAN Bus the socket is binded to.
//
// Length check is NOT performed. Caller is responsible for ensuring `len <= 8`.
void can_write(int socket, uint8_t controller_id, uint8_t packet_type,
               const uint8_t* data, size_t len) {
  can_frame frame;
  frame.can_id = CAN_EFF_FLAG | controller_id | packet_type << 8;
  frame.len = len;
  std::memcpy(frame.data, data, len);

  write(socket, &frame, sizeof(can_frame));
}

void master::write(uint8_t controller_id, const uint8_t* data, size_t len) {
  uint8_t send_buffer[8];

  if (len <= 6) {
    uint32_t ind = 0;

    send_buffer[ind++] = MASTER_CONTROLLER_ID;
    send_buffer[ind++] = TO_SLAVE_COMMANDS_PROCESS_PACKET;

    std::memcpy(send_buffer + ind, data, len);
    ind += len;

    can_write(socket_, controller_id, CAN_PACKET_PROCESS_SHORT_BUFFER,
              send_buffer, ind);

    return;
  }

  size_t i = 0;

  for (; i < len; i += 7) {
    if (i > 255) break;

    size_t send_len = 7;

    send_buffer[0] = i;

    if (i + 7 > len) send_len = len - i;
    std::memcpy(send_buffer + 1, data + i, send_len);

    can_write(socket_, controller_id, CAN_PACKET_FILL_RX_BUFFER, send_buffer,
              send_len + 1);
  }

  for (; i < len; i += 6) {
    size_t send_len = 6;

    send_buffer[0] = i >> 8;
    send_buffer[1] = i & 0xFF;

    if (i + 6 > len) send_len = len - i;
    std::memcpy(send_buffer + 2, data + i, send_len);

    can_write(socket_, controller_id, CAN_PACKET_FILL_RX_BUFFER_LONG,
              send_buffer, send_len + 2);
  }

  uint32_t ind = 0;
  send_buffer[ind++] = MASTER_CONTROLLER_ID;
  send_buffer[ind++] = TO_SLAVE_COMMANDS_PROCESS_PACKET;
  send_buffer[ind++] = len >> 8;
  send_buffer[ind++] = len & 0xFF;

  boost::crc_xmodem_t crc;
  crc.process_bytes(data, len);
  uint16_t checksum = crc.checksum();

  send_buffer[ind++] = checksum >> 8;
  send_buffer[ind++] = checksum & 0xFF;

  can_write(socket_, controller_id, CAN_PACKET_PROCESS_RX_BUFFER, send_buffer,
            ind);
}

void master::register_slave(uint8_t controller_id) {
  slaves_status_[controller_id] = {};
}

slave_status master::get_slave_status(uint8_t controller_id) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    return slaves_status_.at(controller_id);
  } catch (std::out_of_range) {
    throw std::logic_error("Slave is not registered");
  }
}

void master::wait_all_ready() {
  std::unique_lock<std::mutex> lock(mutex_);
  is_all_ready_cv_.wait(lock, [this] { return is_all_ready_unlocked(); });
}

void master::start_monitor_thread() {
  reset_monitor_stop_efd();
  monitor_thread_ = std::thread(&master::monitor_thread_f, this);
}

void master::stop_monitor_thread() {
  uint64_t MONITOR_STOP_EFD_INC_VAL = 1;
  if (::write(monitor_stop_efd_, &MONITOR_STOP_EFD_INC_VAL, sizeof(uint64_t)) ==
      -1) {
    if (errno == EAGAIN) {
      throw std::logic_error(
          "Monitor stop eventfd counter is about to overflow, perhaps the "
          "program is trying to stop the monitor thread repeatedly");
    } else {
      throw std::runtime_error("Failed to write to monitor stop eventfd");
    }
  }
}

void master::join_monitor_thread() { monitor_thread_.join(); }

void master::reset_monitor_stop_efd() {
  uint64_t tmp;
  // We only need to handle non-EAGAIN errors. As we are resetting monitor stop
  // eventfd to zero, it is okay for the counter to be zero when we read it.
  if (read(monitor_stop_efd_, &tmp, sizeof(uint64_t)) == -1 &&
      errno != EAGAIN) {
    throw std::runtime_error("Failed to read monitor stop eventfd");
  }
}

bool master::is_all_ready_unlocked() {
  std::size_t count = slaves_status_.size();
  return status_1_ready_count_ == count && status_4_ready_count_ == count &&
         status_5_ready_count_ == count;
}

void master::log_ready_slaves() {
  std::stringstream ids;

  auto it = slaves_status_.begin();
  int start = it->first;
  int end = it->first;
  ++it;

  for (; it != slaves_status_.end(); ++it) {
    if (it->first == end + 1) {
      end = it->first;
      continue;
    }

    if (start == end) {
      ids << start;
    } else {
      ids << start;
      ids << '-';
      ids << end;
    }
    ids << ", ";

    start = it->first;
    end = it->first;
  }

  if (start == end) {
    ids << start;
  } else {
    ids << start;
    ids << '-';
    ids << end;
  }

  spdlog::info("Master `{}` ready slaves: {}", device_name_, ids.str());
}

void master::process_can_frame(can_frame frame) {
  std::lock_guard<std::mutex> lock(mutex_);

  uint8_t id = frame.can_id & 0xFF;
  if (!slaves_status_.contains(id)) return;

  uint8_t cmd = frame.can_id >> 8;
  if (cmd != CAN_PACKET_STATUS && cmd != CAN_PACKET_STATUS_4 &&
      cmd != CAN_PACKET_STATUS_5) {
    return;
  }

  slave_status& status = slaves_status_[id];
  int ind = 0;

  switch (cmd) {
    case CAN_PACKET_STATUS:
      if (!status.status_1.ready) {
        ++status_1_ready_count_;

        if (is_all_ready_unlocked()) {
          log_ready_slaves();
          is_all_ready_cv_.notify_all();
        }
      }

      status.status_1.ready = true;
      status.status_1.rpm = buf::get_int32_be(frame.data, ind);
      status.status_1.current = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_1.duty = buf::get_int16_be(frame.data, ind) / 1000.0;

      break;

    case CAN_PACKET_STATUS_4:
      if (!status.status_4.ready) {
        ++status_4_ready_count_;

        if (is_all_ready_unlocked()) {
          log_ready_slaves();
          is_all_ready_cv_.notify_all();
        }
      }

      status.status_4.ready = true;
      status.status_4.temp_fet = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_4.temp_motor = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_4.current_in = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_4.pid_pos_now = buf::get_int16_be(frame.data, ind) / 50.0;

      break;

    case CAN_PACKET_STATUS_5:
      if (!status.status_5.ready) {
        ++status_5_ready_count_;

        if (is_all_ready_unlocked()) {
          log_ready_slaves();
          is_all_ready_cv_.notify_all();
        }
      }

      status.status_5.ready = true;
      status.status_5.pid_pos_full_now = buf::get_float32_be(frame.data, ind);
      status.status_5.v_in = buf::get_int16_be(frame.data, ind) / 10.0;

      break;

    default:
      return;
  }
}

void master::monitor_thread_f() {
  pollfd pfds[2];

  pollfd& can_pfd = pfds[0] = {
      .fd = socket_,
      .events = POLLIN,
  };
  pollfd& monitor_stop_pfd = pfds[1] = {
      .fd = monitor_stop_efd_,
      .events = POLLIN,
  };

  while (true) {
    if (poll(pfds, 2, -1) == -1) {
      throw std::runtime_error(
          "Failed to poll SocketCAN and monitor stop eventfd");
    }

    // Stop monitoring immediately when monitor stop eventfd receives an event.
    // `can_frame`s still in the CAN Bus socket are not processed.
    if (monitor_stop_pfd.revents & POLLIN) break;

    // Timeout is set to -1, poll will block infinitely. If monitor stop eventfd
    // is not ready, it must be that CAN Bus socket is ready here. Therefore, we
    // does not need to perform another check.

    can_frame frame;
    while (recv(socket_, &frame, sizeof(can_frame), MSG_DONTWAIT) != -1)
      process_can_frame(frame);

    if (errno != EAGAIN && errno != EWOULDBLOCK)
      throw std::runtime_error("Failed to read from CAN Bus socket");
  }
}

slave::slave(master& can_master, uint8_t controller_id)
    : can_master_(&can_master), controller_id_(controller_id) {
  can_master.register_slave(controller_id);
}

void slave::send_payload_mut(std::vector<uint8_t>& payload) {
  can_master_->write(controller_id_, payload.data(), payload.size());
}

void slave::send_payload(const uint8_t* data, size_t size) {
  can_master_->write(controller_id_, data, size);
}

void slave::set_duty_cycle(double duty_cycle) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_DUTY);

  boost::endian::big_int32_buf_t duty_cycle_buf(duty_cycle * 1e5);
  buf.insert(buf.end(), duty_cycle_buf.data(),
             duty_cycle_buf.data() + sizeof(duty_cycle_buf));

  send_payload_mut(buf);
}

void slave::set_erpm(int erpm) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_RPM);

  boost::endian::big_int32_buf_t erpm_buf(erpm);
  buf.insert(buf.end(), erpm_buf.data(), erpm_buf.data() + sizeof(erpm_buf));

  send_payload_mut(buf);
}

void slave::set_current(double current) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_CURRENT);

  boost::endian::big_int32_buf_t current_buf(current * 1e3);
  buf.insert(buf.end(), current_buf.data(),
             current_buf.data() + sizeof(current_buf));

  send_payload_mut(buf);
}

void slave::set_pos_abs(double pos) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_POS);

  boost::endian::big_int32_buf_t pos_buf(pos * 1e6);
  buf.insert(buf.end(), pos_buf.data(), pos_buf.data() + sizeof(pos_buf));

  send_payload_mut(buf);
}

void slave::set_pos(double pos) {
  // Position is sent to the slave as integer after multiplying a scale
  // constant. Normalize once before sending it to prevent precision loss.
  double pos_full = zero_full + pos;
  double pos_norm = norm_0i_360e(pos_full);
  set_pos_abs(pos_norm);
}

void slave::set_pos_full_abs(float pos) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_POS_FULL);

  boost::endian::big_float32_buf_t pos_buf(pos);
  buf.insert(buf.end(), pos_buf.data(), pos_buf.data() + sizeof(float));

  send_payload_mut(buf);
}

void slave::set_pos_full(double pos) { set_pos_full_abs(zero_full + pos); }

int slave::get_erpm() {
  slave_status slave_status = get_status();
  if (!slave_status.status_1.ready)
    throw std::logic_error("ERPM is not available yet");
  return slave_status.status_1.rpm;
}

float slave::get_current() {
  slave_status slave_status = get_status();
  if (!slave_status.status_1.ready)
    throw std::logic_error("Motor current is not available yet");
  return slave_status.status_1.current;
}

float slave::get_duty() {
  slave_status slave_status = get_status();
  if (!slave_status.status_1.ready)
    throw std::logic_error("Duty cycle is not available yet");
  return slave_status.status_1.duty;
}

float slave::get_temp_fet() {
  slave_status slave_status = get_status();
  if (!slave_status.status_4.ready)
    throw std::logic_error("FET temperature is not available yet");
  return slave_status.status_4.temp_fet;
}

float slave::get_temp_motor() {
  slave_status slave_status = get_status();
  if (!slave_status.status_4.ready)
    throw std::logic_error("Motor temperature is not available yet");
  return slave_status.status_4.temp_motor;
}

float slave::get_current_in() {
  slave_status slave_status = get_status();
  if (!slave_status.status_4.ready)
    throw std::logic_error("Input current is not available yet");
  return slave_status.status_4.current_in;
}

float slave::get_pid_pos_abs() {
  slave_status slave_status = get_status();
  if (!slave_status.status_4.ready)
    throw std::logic_error("PID position is not available yet");
  return slave_status.status_4.pid_pos_now;
}

double slave::get_pid_pos() {
  return norm_0i_360e(get_pid_pos_abs() - zero_full);
}

float slave::get_v_in() {
  slave_status slave_status = get_status();
  if (!slave_status.status_5.ready)
    throw std::logic_error("Input voltage is not available yet");
  return slave_status.status_5.v_in;
}

float slave::get_pid_pos_full_abs() {
  slave_status slave_status = get_status();
  if (!slave_status.status_5.ready)
    throw std::logic_error("Full range PID position is not available yet");
  return slave_status.status_5.pid_pos_full_now;
}

double slave::get_pid_pos_full() { return get_pid_pos_full_abs() - zero_full; }

void slave::set_zero() { zero_full = get_pid_pos_full_abs(); }

void slave::set_zero(double current_angle) {
  zero_full = get_pid_pos_full_abs() - current_angle;
}

void slave::reset_zero() { zero_full = 0; }

slave_status slave::get_status() {
  return can_master_->get_slave_status(controller_id_);
}

}  // namespace vesccom::socketcan

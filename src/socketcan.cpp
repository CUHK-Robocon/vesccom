#include "vesccom/socketcan.h"

#include <errno.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <utility>

#include "bldc/datatypes.h"
#include "boost/crc.hpp"
#include "vesccom/buf.h"

namespace vesccom {

const uint8_t TO_SLAVE_COMMANDS_PROCESS_PACKET = 0;
const uint8_t MASTER_CONTROLLER_ID = 0;

socketcan_master::socketcan_master(const char* device_name) {
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

socketcan_master::~socketcan_master() {
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

void socketcan_master::write(uint8_t controller_id, const uint8_t* data,
                             size_t len) {
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

void socketcan_master::register_slave(uint8_t controller_id) {
  slaves_status_[controller_id] = {};
}

socketcan_status socketcan_master::get_slave_status(uint8_t controller_id) {
  try {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    return slaves_status_.at(controller_id);
  } catch (std::out_of_range) {
    throw std::logic_error("Slave is not registered");
  }
}

void socketcan_master::wait_pid_pos_full_now_all_ready() {
  std::unique_lock<std::mutex> lock(monitor_mutex_);
  pid_pos_full_now_all_ready_cv_.wait(
      lock, [this] { return is_pid_pos_full_now_all_ready_unlocked(); });
}

void socketcan_master::start_monitor_thread() {
  reset_monitor_stop_efd();
  monitor_thread_ = std::thread(&socketcan_master::monitor_thread_f, this);
}

void socketcan_master::stop_monitor_thread() {
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

void socketcan_master::join_monitor_thread() { monitor_thread_.join(); }

void socketcan_master::reset_monitor_stop_efd() {
  uint64_t tmp;
  // We only need to handle non-EAGAIN errors. As we are resetting monitor stop
  // eventfd to zero, it is okay for the counter to be zero when we read it.
  if (read(monitor_stop_efd_, &tmp, sizeof(uint64_t)) == -1 &&
      errno != EAGAIN) {
    throw std::runtime_error("Failed to read monitor stop eventfd");
  }
}

bool socketcan_master::is_pid_pos_full_now_all_ready_unlocked() {
  return pid_pos_full_now_ready_count_ == slaves_status_.size();
}

void socketcan_master::process_can_frame(can_frame frame) {
  std::lock_guard<std::mutex> lock(monitor_mutex_);

  uint8_t id = frame.can_id & 0xFF;
  if (!slaves_status_.contains(id)) return;

  uint8_t cmd = frame.can_id >> 8;
  if (cmd != CAN_PACKET_STATUS && cmd != CAN_PACKET_STATUS_4 &&
      cmd != CAN_PACKET_STATUS_5) {
    return;
  }

  socketcan_status& status = slaves_status_[id];
  int ind = 0;

  switch (cmd) {
    case CAN_PACKET_STATUS:
      status.status_1.ready = true;
      status.status_1.rpm = buf::get_int32_be(frame.data, ind);
      status.status_1.current = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_1.duty = buf::get_int16_be(frame.data, ind) / 1000.0;
      break;

    case CAN_PACKET_STATUS_4:
      status.status_4.ready = true;
      status.status_4.temp_fet = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_4.temp_motor = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_4.current_in = buf::get_int16_be(frame.data, ind) / 10.0;
      status.status_4.pid_pos_now = buf::get_int16_be(frame.data, ind) / 50.0;
      break;

    case CAN_PACKET_STATUS_5:
      if (!status.status_5.ready) ++pid_pos_full_now_ready_count_;

      status.status_5.ready = true;
      status.status_5.pid_pos_full_now = buf::get_float32_be(frame.data, ind);
      status.status_5.v_in = buf::get_int16_be(frame.data, ind) / 10.0;

      if (is_pid_pos_full_now_all_ready_unlocked()) {
        // TODO: It is possible that the waiter will block again for the mutex.
        // The best solution is to notify after the lock guard is dropped. For
        // readibility, it is keep as is for now.
        pid_pos_full_now_all_ready_cv_.notify_all();
      }

      break;

    default:
      return;
  }
}

void socketcan_master::monitor_thread_f() {
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

}  // namespace vesccom

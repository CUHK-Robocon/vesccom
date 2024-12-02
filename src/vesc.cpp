#include "vesccom/vesc.h"

#include <chrono>
#include <stdexcept>

#include "bldc/datatypes.h"
#include "boost/endian.hpp"

namespace vesccom {

vesc::vesc(const char* device_path, int baud_rate)
    : serial_(io_ctx_, device_path) {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

  {
    std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
    keep_alive_instances_.insert(this);
  }
}

vesc::vesc(socketcan_master& can_master, uint8_t controller_id)
    : serial_(io_ctx_),
      can_master_(&can_master),
      controller_id_(controller_id) {
  {
    std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
    keep_alive_instances_.insert(this);
  }

  can_master.register_slave(controller_id);
}

vesc::~vesc() {
  {
    std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
    keep_alive_instances_.erase(this);
  }
}

void vesc::start_keep_alive_thread() {
  if (keep_alive_thread_.joinable()) return;

  keep_alive_thread_ = std::thread(keep_alive_thread_f);
}

void vesc::stop_keep_alive_thread() {
  {
    std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
    keep_alive_thread_should_stop_ = true;
  }
}

void vesc::join_keep_alive_thread() { keep_alive_thread_.join(); }

packet_parse_status vesc::receive_to(std::vector<uint8_t>& payload_out) {
  if (is_slave())
    throw std::logic_error("Packet receiving for slaves is not supported");

  packet_parse_result result;
  std::vector<uint8_t> read_buf;
  packet_parse_state state;

  {
    std::lock_guard<std::mutex> lock(serial_read_mutex_);

    while (true) {
      result =
          packet_parse(read_buf.data(), read_buf.size(), payload_out, state);
      if (result.status != packet_parse_status::INSUFFICIENT_DATA) break;

      read_buf.resize(read_buf.size() + result.bytes_needed);

      boost::asio::read(serial_, boost::asio::buffer(read_buf.end().base() -
                                                         result.bytes_needed,
                                                     result.bytes_needed));
    }
  }

  return result.status;
}

void vesc::send_payload_mut(std::vector<uint8_t>& payload) {
  if (!is_slave()) packet_wrap(payload);
  write(payload.data(), payload.size());
}

void vesc::send_payload(const uint8_t* data, size_t size) {
  std::vector<uint8_t> buf;
  buf.insert(buf.end(), data, data + size);

  send_payload_mut(buf);
}

void vesc::set_duty_cycle(double duty_cycle) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_DUTY);

  boost::endian::big_int32_buf_t duty_cycle_buf(duty_cycle * 1e5);
  buf.insert(buf.end(), duty_cycle_buf.data(),
             duty_cycle_buf.data() + sizeof(duty_cycle_buf));

  send_payload_mut(buf);
}

void vesc::set_erpm(int erpm) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_RPM);

  boost::endian::big_int32_buf_t erpm_buf(erpm);
  buf.insert(buf.end(), erpm_buf.data(), erpm_buf.data() + sizeof(erpm_buf));

  send_payload_mut(buf);
}

void vesc::set_current(double current) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_CURRENT);

  boost::endian::big_int32_buf_t current_buf(current * 1e3);
  buf.insert(buf.end(), current_buf.data(),
             current_buf.data() + sizeof(current_buf));

  send_payload_mut(buf);
}

void vesc::set_pos(double pos) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_POS);

  boost::endian::big_int32_buf_t pos_buf(pos * 1e6);
  buf.insert(buf.end(), pos_buf.data(), pos_buf.data() + sizeof(pos_buf));

  send_payload_mut(buf);
}

float vesc::get_pid_pos_full() {
  if (!is_slave())
    throw std::logic_error("Getter not implemented for serial VESCs yet");

  socketcan_status slave_status = get_status();
  if (!slave_status.status_5.ready) return NAN;
  return slave_status.status_5.pid_pos_full_now;
}

void vesc::keep_alive_thread_f() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);

      if (keep_alive_thread_should_stop_) break;

      for (vesc* instance : keep_alive_instances_) {
        uint8_t keep_alive_payload[] = {COMM_ALIVE};
        instance->send_payload(keep_alive_payload, sizeof(keep_alive_payload));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

std::vector<uint8_t> vesc::read(size_t size) {
  if (is_slave())
    throw std::logic_error(
        "Reading slave packets from master is not supported");

  std::vector<uint8_t> buf;
  buf.resize(size);

  {
    std::lock_guard<std::mutex> lock(serial_read_mutex_);
    boost::asio::read(serial_, boost::asio::buffer(buf.data(), size));
  }

  return buf;
}

void vesc::write(const void* buf, size_t size) {
  if (is_slave()) {
    can_master_->write(controller_id_, static_cast<const uint8_t*>(buf), size);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    boost::asio::write(serial_, boost::asio::buffer(buf, size));
  }
}

socketcan_status vesc::get_status() {
  auto slave_status_opt = can_master_->get_slave_status(controller_id_);
  if (!slave_status_opt.has_value()) {
    throw std::logic_error(
        "Slave status not found, maybe the slave have not been registered");
  }
  return slave_status_opt.value();
}

}  // namespace vesccom

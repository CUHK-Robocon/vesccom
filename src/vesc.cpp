#include "vesccom/vesc.h"

#include <chrono>

#include "bldc/datatypes.h"

namespace vesccom {

vesc::vesc(const char* device, int baud_rate) : serial_(io_ctx_, device) {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

  {
    std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);

    keep_alive_instances_.insert(this);
  }
}

vesc::~vesc() {
  std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);

  keep_alive_instances_.erase(this);
}

void vesc::start_keep_alive_thread() {
  if (keep_alive_thread_.joinable()) return;

  keep_alive_thread_ = std::thread(keep_alive_thread_f);
}

void vesc::stop_keep_alive_thread() {
  std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);

  keep_alive_thread_should_stop_ = true;
}

void vesc::join_keep_alive_thread() { keep_alive_thread_.join(); }

std::vector<uint8_t> vesc::read(size_t size) {
  std::vector<uint8_t> buf;
  buf.resize(size);

  {
    std::lock_guard<std::mutex> lock(serial_read_mutex_);

    boost::asio::read(serial_, boost::asio::buffer(buf.data(), size));
  }

  return buf;
}

void vesc::write(const void* buf, size_t size) {
  std::lock_guard<std::mutex> lock(serial_write_mutex_);

  boost::asio::write(serial_, boost::asio::buffer(buf, size));
}

packet_parse_status vesc::receive_to(std::vector<uint8_t>& payload_out) {
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
  packet_wrap(payload);

  write(payload.data(), payload.size());
}

void vesc::send_payload(const uint8_t* data, size_t size) {
  std::vector<uint8_t> buf;

  buf.insert(buf.end(), data, data + size);

  send_payload_mut(buf);
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

}  // namespace vesccom

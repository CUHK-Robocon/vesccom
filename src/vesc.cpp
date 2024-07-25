#include "vesccom/vesc.h"

namespace vesccom {

vesc::vesc(const char* device, int baud_rate) : serial_(io_ctx_, device) {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

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

}  // namespace vesccom

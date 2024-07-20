#include "vesccom/vesc.h"

#include "vesccom/packet.h"

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

void vesc::send_payload_mut(std::vector<uint8_t>& payload) {
  packet_wrap(payload);

  write(payload.data(), payload.size());
}

}  // namespace vesccom

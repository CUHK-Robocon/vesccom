#ifndef VESCCOM_VESC_H_
#define VESCCOM_VESC_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include "boost/asio.hpp"

namespace vesccom {

class vesc {
 public:
  explicit vesc(const char* device, int baud_rate = 115200);

  // Reads and blocks until `size` bytes have been read.
  std::vector<uint8_t> read(size_t size);
  void write(const void* buf, size_t size);

  // Sends `payload` while mutating the original vector.
  //
  // Note that `payload` will be in-place modified.
  void send_payload_mut(std::vector<uint8_t>& payload);

 private:
  // Must be declared before `serial_` for the correct initialization order.
  boost::asio::io_context io_ctx_;

  // NEVER perform read and write directly on this object, use the `vesc::read`
  // and `vesc::write` method instead.
  boost::asio::serial_port serial_;
  std::mutex serial_read_mutex_;
  std::mutex serial_write_mutex_;
};

}  // namespace vesccom

#endif

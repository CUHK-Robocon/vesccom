#ifndef VESCCOM_VESC_H_
#define VESCCOM_VESC_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "boost/asio.hpp"
#include "vesccom/packet.h"

namespace vesccom {

class vesc {
 public:
  explicit vesc(const char* device, int baud_rate = 115200);
  ~vesc();

  static void start_keep_alive_thread();
  static void stop_keep_alive_thread();
  static void join_keep_alive_thread();

  // Reads and blocks until `size` bytes have been read.
  std::vector<uint8_t> read(size_t size);
  void write(const void* buf, size_t size);

  // Receives a packet. Blocks until a complete packet is received.
  //
  // Payload is appended to `payload_out` iif the returned status is
  // `packet_parse_status::SUCCESS`.
  packet_parse_status receive_to(std::vector<uint8_t>& payload_out);

  // Sends `payload` while mutating the original vector.
  //
  // Note that `payload` will be in-place modified.
  void send_payload_mut(std::vector<uint8_t>& payload);
  void send_payload(const uint8_t* data, size_t size);

 private:
  static void keep_alive_thread_f();

  inline static std::thread keep_alive_thread_;
  inline static bool keep_alive_thread_should_stop_ = false;
  inline static std::unordered_set<vesc*> keep_alive_instances_;
  inline static std::mutex keep_alive_state_mutex_;

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

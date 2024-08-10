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
  // Constructs an instance representing a master VESC.
  explicit vesc(const char* device, int baud_rate = 115200);

  // Constructs an instance representing a slave VESC.
  vesc(vesc& master, uint8_t controller_id);

  ~vesc();

  static void start_keep_alive_thread();
  static void stop_keep_alive_thread();
  static void join_keep_alive_thread();

  // Receives a packet. Blocks until a complete packet is received.
  //
  // Payload is appended to `payload_out` iif the returned status is
  // `packet_parse_status::SUCCESS`.
  //
  // Packet receiving is not supported for slaves. Throws `std::logic_error` if
  // the object represents a slave.
  packet_parse_status receive_to(std::vector<uint8_t>& payload_out);

  // Sends `payload` while mutating the original vector.
  //
  // Note that `payload` will be in-place modified.
  void send_payload_mut(std::vector<uint8_t>& payload);

  void send_payload(const uint8_t* data, size_t size);

 private:
  static void keep_alive_thread_f();

  // Reads and blocks until `size` bytes have been read.
  //
  // Reading slave packets from master is not supported. Throws
  // `std::logic_error` if the object represents a slave.
  std::vector<uint8_t> read(size_t size);

  // Writes `buf` to master.
  //
  // Note that `buf` WILL NOT be wrapped in a `COMM_FORWARD_CAN` packet
  // automatically. Thus, use with caution, make sure packets targetting a slave
  // is not send to the master accidentally.
  void write(const void* buf, size_t size);

  inline static std::thread keep_alive_thread_;
  inline static bool keep_alive_thread_should_stop_ = false;
  inline static std::unordered_set<vesc*> keep_alive_instances_;
  inline static std::mutex keep_alive_state_mutex_;

  vesc* master_ = nullptr;

  uint8_t controller_id_;

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

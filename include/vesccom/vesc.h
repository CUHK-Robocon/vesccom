#ifndef VESCCOM_INCLUDE_VESCCOM_VESC_H_
#define VESCCOM_INCLUDE_VESCCOM_VESC_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "vesccom/packet.h"
#include "vesccom/socketcan.h"

// Before Boost.Asio 1.79.0, "boost/asio/awaitable.hpp" does not include
// <utility> causing `std::exchange` to be missing. Fixed by commit
// 71964b22c7fade69cc4caa1c869a868e3a32cc97. Backported to here.
// clang-format off
#include <utility>
#include "boost/asio.hpp"
// clang-format on

namespace vesccom {

class vesc {
 public:
  // Constructs an instance representing a serial master.
  explicit vesc(const char* device_path, int baud_rate = 115200);

  // Constructs an instance representing a slave connected to a SocketCAN
  // master.
  vesc(socketcan_master& can_master, uint8_t controller_id);

  vesc(const vesc&) = delete;

  ~vesc();

  vesc& operator=(const vesc&) = delete;

  static void start_keep_alive_thread();
  static void stop_keep_alive_thread();
  static void join_keep_alive_thread();

  bool is_slave() { return can_master_; }

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
  // Note that `payload` will be in-place modified. It is also NOT guaranteed to
  // be wrapped in a `COMM_FORWARD_CAN` packet, specifically when the master is
  // a SocketCAN master. Therefore, DO NOT assume the content of the vector
  // after calling this method.
  void send_payload_mut(std::vector<uint8_t>& payload);

  void send_payload(const uint8_t* data, size_t size);

  void set_duty_cycle(double duty_cycle);
  void set_erpm(int erpm);
  void set_current(double current);
  void set_pos(double pos);

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
  // automatically even when the master is a serial master. Thus, use with
  // caution, make sure packets targetting a slave is not send to the master
  // accidentally.
  void write(const void* buf, size_t size);

  inline static std::thread keep_alive_thread_;
  inline static bool keep_alive_thread_should_stop_ = false;
  inline static std::unordered_set<vesc*> keep_alive_instances_;
  inline static std::mutex keep_alive_state_mutex_;

  socketcan_master* can_master_ = nullptr;
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

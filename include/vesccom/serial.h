#ifndef VESCCOM_INCLUDE_VESCCOM_SERIAL_H_
#define VESCCOM_INCLUDE_VESCCOM_SERIAL_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include "vesccom/vesc.h"

// Before Boost.Asio 1.79.0, "boost/asio/awaitable.hpp" does not include
// <utility> causing `std::exchange` to be missing. Fixed by commit
// 71964b22c7fade69cc4caa1c869a868e3a32cc97. Backported to here.
// clang-format off
#include <utility>
#include "boost/asio.hpp"
// clang-format on

namespace vesccom::serial {

const size_t PACKET_TYPE_SIZE = 1;
const size_t PACKET_CHECKSUM_SIZE = 2;
const size_t PACKET_TERM_SIZE = 1;

const size_t PACKET_PAYLOAD_LEN_MAX_SIZE = 3;
const size_t PACKET_PAYLOAD_MAX_LEN = 512;

const size_t PACKET_MAX_SIZE = PACKET_TYPE_SIZE + PACKET_PAYLOAD_LEN_MAX_SIZE +
                               PACKET_PAYLOAD_MAX_LEN + PACKET_CHECKSUM_SIZE +
                               PACKET_TERM_SIZE;

struct packet_parse_state {
  int stage = 0;
  ptrdiff_t cursor = 0;

  int type;
  size_t payload_len_size;
  size_t payload_len;
};

enum packet_parse_status {
  SUCCESS,
  INSUFFICIENT_DATA,
  INVALID_TYPE,
  PAYLOAD_TOO_LONG,
  CHECKSUM_MISMATCH,
  INVALID_TERM,
};

struct packet_parse_result {
  packet_parse_status status;

  // Initialized if `status` is `packet_parse_status::INSUFFICIENT_DATA`,
  // uninitialized otherwise.
  size_t bytes_needed;
};

// Parse packet from the data provided.
//
// A default-initialized `packet_parse_state` instance MUST be provided. Payload
// is appended to `payload_out` iif the returned `packet_parse_result.status` is
// `packet_parse_status::SUCCESS`.
packet_parse_result packet_parse(const uint8_t* data, size_t size,
                                 std::vector<uint8_t>& payload_out,
                                 packet_parse_state& state);

// In-place wraps the provided payload into a packet.
//
// Note that the provided vector will be modified directly.
void packet_wrap(std::vector<uint8_t>& payload);

class vesc : public ::vesccom::vesc {
 public:
  explicit vesc(const char* device_path, int baud_rate = 115200);

  // Receives a packet. Blocks until a complete packet is received.
  //
  // Payload is appended to `payload_out` iif the returned status is
  // `packet_parse_status::SUCCESS`.
  packet_parse_status receive_to(std::vector<uint8_t>& payload_out);

  void send_payload_mut(std::vector<uint8_t>& payload) override;

  void set_duty_cycle(double duty_cycle);
  void set_erpm(int erpm);
  void set_current(double current);
  void set_pos(double pos);
  void set_pos_full(float pos);

 private:
  // Reads and blocks until `size` bytes have been read.
  std::vector<uint8_t> read(size_t size);

  // Writes `buf` to master.
  void write(const void* buf, size_t size);

  // Must be declared before `serial_` for the correct initialization order.
  boost::asio::io_context io_ctx_;

  // NEVER perform read and write directly on this object, use the `vesc::read`
  // and `vesc::write` method instead.
  boost::asio::serial_port serial_;
  std::mutex serial_read_mutex_;
  std::mutex serial_write_mutex_;
};

}  // namespace vesccom::serial

#endif

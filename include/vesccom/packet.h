#ifndef VESCCOM_INCLUDE_VESCCOM_PACKET_H_
#define VESCCOM_INCLUDE_VESCCOM_PACKET_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vesccom {

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

// In-place wraps the provided payload into a `COMM_FORWARD_CAN` payload.
//
// Note that the provided vector will be modified directly.
void forward_can_wrap(uint8_t target_controller_id,
                      std::vector<uint8_t>& payload);

}  // namespace vesccom

#endif

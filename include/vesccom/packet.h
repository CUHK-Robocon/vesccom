#ifndef VESCCOM_PACKET_H_
#define VESCCOM_PACKET_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "boost/outcome.hpp"

namespace vesccom {

struct packet_parse_state {
  int stage = 0;
  ptrdiff_t cursor = 0;

  int type;
  size_t payload_len_size;
  size_t payload_len;
};

enum packet_parse_error {
  INVALID_TYPE,
  PAYLOAD_TOO_LONG,
  CHECKSUM_MISMATCH,
  INVALID_TERM,
};

boost::outcome_v2::result<size_t, packet_parse_error> packet_parse(
    const uint8_t* data, size_t size, std::vector<uint8_t>& payload_out,
    packet_parse_state& state);

// In-place wraps the provided payload into a packet.
//
// Note that the provided vector will be modified directly.
void packet_wrap(std::vector<uint8_t>& payload);

}  // namespace vesccom

#endif

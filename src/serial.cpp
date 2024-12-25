#include "vesccom/serial.h"

#include <cstring>
#include <limits>
#include <stdexcept>

#include "bldc/datatypes.h"
#include "boost/crc.hpp"
#include "boost/endian.hpp"

namespace vesccom::serial {

const uint8_t PACKET_TERM_BYTE = 0x03;

packet_parse_result packet_parse(const uint8_t* data, size_t size,
                                 std::vector<uint8_t>& payload_out,
                                 packet_parse_state& state) {
  switch (state.stage) {
    case 0:
      if (size < state.cursor + PACKET_TYPE_SIZE) {
        return {
            .status = packet_parse_status::INSUFFICIENT_DATA,
            .bytes_needed = state.cursor + 1 - size,
        };
      }

      state.type = data[state.cursor];

      if (state.type == 2) {
        state.payload_len_size = 1;
      } else if (state.type == 3) {
        state.payload_len_size = 2;
      } else if (state.type == 4) {
        state.payload_len_size = 3;
      } else {
        return {.status = packet_parse_status::INVALID_TYPE};
      }

      ++state.cursor;
      ++state.stage;

    case 1: {
      if (size < state.cursor + state.payload_len_size) {
        return {
            .status = packet_parse_status::INSUFFICIENT_DATA,
            .bytes_needed = state.cursor + state.payload_len_size - size,
        };
      }

      boost::endian::big_uint24_buf_t payload_len_buf(0);
      memcpy(payload_len_buf.data() + sizeof(payload_len_buf) -
                 state.payload_len_size,
             data + state.cursor, state.payload_len_size);
      state.payload_len = payload_len_buf.value();

      if (state.payload_len > PACKET_PAYLOAD_MAX_LEN)
        return {.status = packet_parse_status::PAYLOAD_TOO_LONG};

      state.cursor += state.payload_len_size;
      ++state.stage;
    }

    case 2: {
      if (size < state.cursor + state.payload_len + PACKET_CHECKSUM_SIZE +
                     PACKET_TERM_SIZE) {
        return {
            .status = packet_parse_status::INSUFFICIENT_DATA,
            .bytes_needed = state.cursor + state.payload_len +
                            PACKET_CHECKSUM_SIZE + PACKET_TERM_SIZE - size,
        };
      }

      uint8_t term =
          data[state.cursor + state.payload_len + PACKET_CHECKSUM_SIZE];
      if (term != PACKET_TERM_BYTE)
        return {.status = packet_parse_status::INVALID_TERM};

      boost::crc_xmodem_t crc;
      crc.process_bytes(data + state.cursor, state.payload_len);
      uint16_t checksum = crc.checksum();

      boost::endian::big_uint16_buf_t checksum_expected_buf(0);
      memcpy(checksum_expected_buf.data(),
             data + state.cursor + state.payload_len, PACKET_CHECKSUM_SIZE);
      uint16_t checksum_expected = checksum_expected_buf.value();

      if (checksum != checksum_expected)
        return {.status = packet_parse_status::CHECKSUM_MISMATCH};

      payload_out.insert(payload_out.end(), data + state.cursor,
                         data + state.cursor + state.payload_len);

      state.cursor +=
          state.payload_len + PACKET_CHECKSUM_SIZE + PACKET_TERM_SIZE;
      state.stage = 0;

      return {.status = packet_parse_status::SUCCESS};
    }

    default:
      throw std::logic_error("Invalid packet parsing stage");
  }
}

void packet_wrap(std::vector<uint8_t>& payload) {
  size_t payload_len = payload.size();

  int type;
  size_t payload_len_size;
  if (payload_len <= std::numeric_limits<uint8_t>::max()) {
    type = 2;
    payload_len_size = 1;
  } else if (payload_len <= std::numeric_limits<uint16_t>::max()) {
    type = 3;
    payload_len_size = 2;
  } else {
    type = 4;
    payload_len_size = 3;
  }

  boost::endian::big_uint24_buf_t payload_len_buf(payload_len);

  boost::crc_xmodem_t crc;
  crc.process_bytes(payload.data(), payload_len);

  boost::endian::big_uint16_buf_t checksum_buf(crc.checksum());

  // Header, inserted before payload.
  std::vector<uint8_t> header_buf;
  header_buf.push_back(type);
  header_buf.insert(
      header_buf.end(),
      payload_len_buf.data() + sizeof(payload_len_buf) - payload_len_size,
      payload_len_buf.data() + sizeof(payload_len_buf));
  payload.insert(payload.begin(), header_buf.begin(), header_buf.end());

  // Footer, inserted after payload.
  payload.insert(payload.end(), checksum_buf.data(),
                 checksum_buf.data() + sizeof(checksum_buf));
  payload.push_back(PACKET_TERM_BYTE);
}

vesc::vesc(const char* device_path, int baud_rate)
    : serial_(io_ctx_, device_path) {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
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

void vesc::set_duty_cycle(double duty_cycle) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_DUTY);

  boost::endian::big_int32_buf_t duty_cycle_buf(duty_cycle * 1e5);
  buf.insert(buf.end(), duty_cycle_buf.data(),
             duty_cycle_buf.data() + sizeof(duty_cycle_buf));

  send_payload_mut(buf);
}

void vesc::set_erpm(int erpm) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_RPM);

  boost::endian::big_int32_buf_t erpm_buf(erpm);
  buf.insert(buf.end(), erpm_buf.data(), erpm_buf.data() + sizeof(erpm_buf));

  send_payload_mut(buf);
}

void vesc::set_current(double current) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_CURRENT);

  boost::endian::big_int32_buf_t current_buf(current * 1e3);
  buf.insert(buf.end(), current_buf.data(),
             current_buf.data() + sizeof(current_buf));

  send_payload_mut(buf);
}

void vesc::set_pos(double pos) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_POS);

  boost::endian::big_int32_buf_t pos_buf(pos * 1e6);
  buf.insert(buf.end(), pos_buf.data(), pos_buf.data() + sizeof(pos_buf));

  send_payload_mut(buf);
}

void vesc::set_pos_full(float pos) {
  std::vector<uint8_t> buf;

  buf.push_back(COMM_SET_POS_FULL);

  boost::endian::big_float32_buf_t pos_buf(pos);
  buf.insert(buf.end(), pos_buf.data(), pos_buf.data() + sizeof(float));

  send_payload_mut(buf);
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

}  // namespace vesccom::serial

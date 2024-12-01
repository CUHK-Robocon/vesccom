#ifndef VESCCOM_INCLUDE_VESCCOM_BUF_H_
#define VESCCOM_INCLUDE_VESCCOM_BUF_H_

#include <cstdint>
#include <cstring>

#include "boost/endian.hpp"

namespace vesccom::buf {

namespace internal {

template <class T, class Buf>
T get_be(void* buffer, int& index) {
  static_assert(sizeof(T) == sizeof(Buf));

  Buf buf;
  std::memcpy(buf.data(), reinterpret_cast<std::uint8_t*>(buffer) + index,
              sizeof(Buf));
  index += sizeof(Buf);
  return buf.value();
}

}  // namespace internal

constexpr auto get_int16_be =
    internal::get_be<std::int16_t, boost::endian::big_int16_buf_t>;
constexpr auto get_int32_be =
    internal::get_be<std::int32_t, boost::endian::big_int32_buf_t>;

constexpr auto get_float32_be =
    internal::get_be<float, boost::endian::big_float32_buf_t>;

};  // namespace vesccom::buf

#endif

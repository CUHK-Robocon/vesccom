#ifndef VESCCOM_INCLUDE_VESCCOM_SOCKETCAN_MASTER_H_
#define VESCCOM_INCLUDE_VESCCOM_SOCKETCAN_MASTER_H_

#include <cstddef>
#include <cstdint>

namespace vesccom {

class socketcan_master {
 public:
  explicit socketcan_master(const char* device_name);

  socketcan_master(const socketcan_master&) = delete;
  socketcan_master(socketcan_master&& other);

  ~socketcan_master();

  socketcan_master& operator=(const socketcan_master&) = delete;
  socketcan_master& operator=(socketcan_master&& other);

  void write(uint8_t controller_id, const uint8_t* data, size_t len);

 private:
  int socket_ = -1;
};

}  // namespace vesccom

#endif

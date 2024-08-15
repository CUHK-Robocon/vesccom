#ifndef VESCCOM_SOCKETCAN_MASTER_H_
#define VESCCOM_SOCKETCAN_MASTER_H_

#include <cstddef>
#include <cstdint>

namespace vesccom {

class socketcan_master {
 public:
  socketcan_master(const char* device_name);
  ~socketcan_master();

  void write(uint8_t controller_id, const uint8_t* data, size_t len);

 private:
  int socket_;
};

}  // namespace vesccom

#endif

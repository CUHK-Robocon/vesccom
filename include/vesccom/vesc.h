#ifndef VESCCOM_INCLUDE_VESCCOM_VESC_H_
#define VESCCOM_INCLUDE_VESCCOM_VESC_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

namespace vesccom {

class vesc {
 public:
  vesc();
  virtual ~vesc();

  vesc(const vesc&) = delete;
  vesc& operator=(const vesc&) = delete;

  static void start_keep_alive_thread();
  static void stop_keep_alive_thread();
  static void join_keep_alive_thread();

  // No implementation requirement is imposed. DO NOT assume the content of the
  // vector after calling this method.
  virtual void send_payload_mut(std::vector<uint8_t>& payload) = 0;

  virtual void send_payload(const uint8_t* data, size_t size);

 private:
  static void keep_alive_thread_f();

  inline static std::thread keep_alive_thread_;
  inline static bool keep_alive_thread_should_stop_ = false;
  inline static std::unordered_set<vesc*> keep_alive_instances_;
  inline static std::mutex keep_alive_state_mutex_;
};

};  // namespace vesccom

#endif

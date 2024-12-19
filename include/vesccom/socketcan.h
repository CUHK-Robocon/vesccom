#ifndef VESCCOM_INCLUDE_VESCCOM_SOCKETCAN_H_
#define VESCCOM_INCLUDE_VESCCOM_SOCKETCAN_H_

#include <linux/can.h>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

namespace vesccom {

struct socketcan_status {
  struct {
    bool ready = false;

    int rpm;
    float current;
    float duty;
  } status_1;

  struct {
    bool ready = false;

    float temp_fet;
    float temp_motor;
    float current_in;
    float pid_pos_now;
  } status_4;

  struct {
    bool ready = false;

    float v_in;
    float pid_pos_full_now;
  } status_5;
};

class socketcan_master {
 public:
  socketcan_master(const char* device_name);

  socketcan_master(const socketcan_master&) = delete;

  ~socketcan_master();

  socketcan_master& operator=(const socketcan_master&) = delete;

  void write(uint8_t controller_id, const uint8_t* data, size_t len);

  // Must be called before starting monitor thread.
  void register_slave(uint8_t controller_id);
  std::optional<socketcan_status> get_slave_status(uint8_t controller_id);

  void wait_pid_pos_full_now_all_ready();

  void start_monitor_thread();
  void stop_monitor_thread();
  void join_monitor_thread();

 private:
  // Resets monitor stop eventfd counter to zero.
  void reset_monitor_stop_efd();

  bool is_pid_pos_full_now_all_ready_unlocked();
  void process_can_frame(can_frame frame);

  void monitor_thread_f();

  int socket_;

  std::thread monitor_thread_;
  int monitor_stop_efd_;
  std::unordered_map<uint8_t, socketcan_status> slaves_status_;
  int pid_pos_full_now_ready_count_ = 0;
  std::condition_variable pid_pos_full_now_all_ready_cv_;
  // This mutex is guarding both slaves status and the CV's condition as we want
  // the slaves status to be visible on the waiting thread when notified.
  std::mutex monitor_mutex_;
};

}  // namespace vesccom

#endif

#ifndef VESCCOM_INCLUDE_VESCCOM_SOCKETCAN_H_
#define VESCCOM_INCLUDE_VESCCOM_SOCKETCAN_H_

#include <linux/can.h>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "vesccom/vesc.h"

namespace vesccom::socketcan {

struct slave_status {
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

class master {
 public:
  explicit master(const char* device_name);

  master(const master&) = delete;

  ~master();

  master& operator=(const master&) = delete;

  void write(uint8_t controller_id, const uint8_t* data, size_t len);

  // Allocates state resources for the slave.
  //
  // This method is for VESC instances to late-initialize. Therefore, the method
  // SHOULD NOT be called directly.
  //
  // All slaves MUST be registered before starting the monitor thread.
  void register_slave(uint8_t controller_id);

  // Returns the status of the corresponding slave.
  //
  // Throws `std::logic_error` if the slave is not registered.
  slave_status get_slave_status(uint8_t controller_id);

  // Blocks until all status messages are received from all slaves.
  void wait_all_ready();

  void start_monitor_thread();
  void stop_monitor_thread();
  void join_monitor_thread();

 private:
  // Resets monitor stop eventfd counter to zero.
  void reset_monitor_stop_efd();

  bool is_all_ready_unlocked();
  void process_can_frame(can_frame frame);

  void monitor_thread_f();

  int socket_;
  std::string device_name_;

  std::thread monitor_thread_;
  int monitor_stop_efd_;
  std::unordered_map<uint8_t, slave_status> slaves_status_;
  int status_1_ready_count_ = 0;
  int status_4_ready_count_ = 0;
  int status_5_ready_count_ = 0;
  std::condition_variable is_all_ready_cv_;
  // This mutex is guarding both slaves status and the CV's condition as we want
  // the slaves status to be visible on the waiting thread when notified.
  std::mutex mutex_;
};

class slave : public vesc {
 public:
  slave(master& can_master, uint8_t controller_id);

  void send_payload_mut(std::vector<uint8_t>& payload) override;
  void send_payload(const uint8_t* data, size_t size) override;

  void set_duty_cycle(double duty_cycle) override;
  void set_erpm(int erpm) override;
  void set_current(double current) override;
  void set_pos_abs(double pos) override;
  void set_pos_full_abs(float pos) override;

  int get_erpm() override;
  float get_current() override;
  float get_duty() override;
  float get_temp_fet() override;
  float get_temp_motor() override;
  float get_current_in() override;
  float get_pid_pos_abs() override;
  float get_v_in() override;
  float get_pid_pos_full_abs() override;

 private:
  socketcan::slave_status get_status();

  socketcan::master* can_master_;
  uint8_t controller_id_;
};

}  // namespace vesccom::socketcan

#endif

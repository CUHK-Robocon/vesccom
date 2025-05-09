#include "vesccom/vesc.h"

#include "bldc/datatypes.h"
#include "vesccom/util.h"

namespace vesccom {

vesc::vesc() {
  std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
  keep_alive_instances_.insert(this);
}

vesc::~vesc() {
  std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
  keep_alive_instances_.erase(this);
}

void vesc::start_keep_alive_thread() {
  if (keep_alive_thread_.joinable()) return;
  keep_alive_thread_ = std::thread(keep_alive_thread_f);
}

void vesc::stop_keep_alive_thread() {
  std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);
  keep_alive_thread_should_stop_ = true;
}

void vesc::join_keep_alive_thread() { keep_alive_thread_.join(); }

void vesc::send_payload(const uint8_t* data, size_t size) {
  std::vector<uint8_t> buf;
  buf.insert(buf.end(), data, data + size);

  send_payload_mut(buf);
}

void vesc::set_pos(double pos) {
  double pos_full = zero_full + pos;
  double pos_norm = norm_0i_360e(pos_full);
  set_pos_abs(pos_norm);
}

void vesc::set_pos_full(double pos) { set_pos_full_abs(zero_full + pos); }

double vesc::get_pid_pos() {
  return norm_0i_360e(get_pid_pos_abs() - zero_full);
}

double vesc::get_pid_pos_full() { return get_pid_pos_full_abs() - zero_full; }

void vesc::set_zero(double current_angle) {
  zero_full = get_pid_pos_full_abs() - current_angle;
}

void vesc::reset_zero() { zero_full = 0; }

void vesc::keep_alive_thread_f() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(keep_alive_state_mutex_);

      if (keep_alive_thread_should_stop_) break;

      for (vesc* instance : keep_alive_instances_) {
        uint8_t keep_alive_payload[] = {COMM_ALIVE};
        instance->send_payload(keep_alive_payload, sizeof(keep_alive_payload));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

}  // namespace vesccom

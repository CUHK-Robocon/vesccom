#ifndef VESCCOM_INCLUDE_VESCCOM_SYNC_H_
#define VESCCOM_INCLUDE_VESCCOM_SYNC_H_

#include <condition_variable>
#include <mutex>

namespace vesccom::sync {

class completion_notifier {
 public:
  void notify() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = true;
    }
    cv_.notify_all();
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return ready_; });
  }

 private:
  std::condition_variable cv_;
  std::mutex mutex_;
  bool ready_ = false;
};

}  // namespace vesccom::sync

#endif

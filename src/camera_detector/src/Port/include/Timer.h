#ifndef SRC_TIMER_H
#define SRC_TIMER_H

#include <iostream>
#include <thread>
#include <atomic>
#include <functional>
#include <vector>

class Timer {
public:
    Timer() : is_running(false) {}
    void start();
    void stop();
    void addTimer(std::function<void()> callback, int interval);
private:
    struct TimerInfo {
        std::function<void()> callback;
        int interval;
        std::chrono::time_point<std::chrono::steady_clock> lastTime;
    };
    std::atomic<bool> is_running;
    std::vector<TimerInfo> timers;
};


#endif //SRC_TIMER_H

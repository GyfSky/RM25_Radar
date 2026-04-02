#include "../include/Timer.h"

void Timer::start() {
    is_running = true;
    for(auto& timer : timers){
        timer.lastTime = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 小睡10毫秒
    }
    while (is_running){
        for(auto& timer : timers){
            if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer.lastTime) >= std::chrono::milliseconds(timer.interval)) {
                timer.callback();
                timer.lastTime = std::chrono::steady_clock::now();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 小睡10毫秒
    }
}

void Timer::stop(){
    is_running = false;
}

void Timer::addTimer(std::function<void()> callback, int interval) {
    TimerInfo timerInfo;
    timerInfo.callback = callback;
    timerInfo.interval = interval;
    timerInfo.lastTime = std::chrono::steady_clock::now();
    timers.push_back(timerInfo);
}

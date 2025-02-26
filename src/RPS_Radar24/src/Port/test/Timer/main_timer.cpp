//
// Created by plusseven on 24-6-30.
//
#include "../../include/Timer.h"

int main() {
    Timer timer;

    timer.addTimer([]() {
        std::cout << "Timer 1 callback called." << std::endl;
    }, 1000); // 每隔1秒触发一次回调函数1

    timer.addTimer([]() {
        std::cout << "Timer 2 callback called." << std::endl;
    }, 2000); // 每隔2秒触发一次回调函数2

    std::thread timerThread(&Timer::start, &timer);
    timerThread.join();

    return 0;

}



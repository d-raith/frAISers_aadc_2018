//
// Created by aadc on 15.09.18.
//
#pragma once

#include <chrono>
#include "string"
#include "iostream"

#ifndef AADC_USER_STOPWATCH_H
#define AADC_USER_STOPWATCH_H

#endif //AADC_USER_STOPWATCH_H

class StopWatch {

public:

    explicit StopWatch(bool start_measure = true, bool isNano = false) : is_nano(isNano) {
        if (start_measure) {
            beginMeasure();
        }

    }

    double measure() const {
        if (!is_nano) {
            auto now = std::chrono::high_resolution_clock::now();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            return milliseconds.count();
        } else {
            auto now = std::chrono::high_resolution_clock::now();
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start);
            return nanoseconds.count();
        }
    }


    bool didMillisecondsPass(int milliseconds) const {
        if (is_nano) {
            return measure() > milliseconds * 1000;
        }
        return measure() > milliseconds;
    }

    bool didSecondsPass(float seconds) const {
        return didMillisecondsPass(seconds * 1000);
    }

    void beginMeasure() {
        start = std::chrono::high_resolution_clock::now();
    }

    void reset() {
        beginMeasure();
    }

    void print_measurement(std::string log_text, double millsec_threshold = 0) const {
        double result = measure();
        if (is_nano) {
            millsec_threshold *= 1000;
        }
        if (result >= millsec_threshold) {
            if (is_nano) {
                std::cout << log_text << "  | time: " << result << "ns" << std::endl;
            } else {
                std::cout << log_text << "  | time: " << result << "ms" << std::endl;
            }
        }
    }

private:
    std::chrono::high_resolution_clock::time_point start;
    bool is_nano;
};


class Timer {


    StopWatch timer;
    float seconds;
    bool is_active;


public:
    explicit Timer(float seconds) : seconds(seconds), is_active(false) {

    }


    void start() {
        is_active = true;
        timer.reset();
    }


    float getDuration() const {
        return seconds;
    }

    void addDuration(float seconds) {
        this->seconds += seconds;
    }


    float getRemainingMs() const {
        return seconds * 1000 - timer.measure();
    }

    float getElapsedMs() const {
        return static_cast<float>(timer.measure());
    }

    bool isElapsed(float seconds) {
        /*if (!is_active) {
            return true;
        }*/

        bool elapsed = timer.didSecondsPass(seconds);
        is_active = !elapsed;
        return elapsed;

    }

    bool isDone() {
        return isElapsed(seconds);
    }

    bool isActive() {
        return is_active;
    }

};
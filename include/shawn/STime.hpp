#ifndef SHAWN_TIME_HPP
#define SHAWN_TIME_HPP

#include <chrono>
#include <vector>
#include <ctime>
#include <stdint.h>
#include <SVector.hpp>

namespace shawn {

class STime {
public:
    SVector shawn_vector;

    std::vector<size_t> error_positions;

    std::chrono::time_point<std::chrono::steady_clock> simple_start_time_point;

    std::vector<std::chrono::time_point<std::chrono::steady_clock>> last_timestamps;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> last_last_timestamps;
    std::vector<double> timestamps_duration_ms;
    std::vector<double> timestamps_till_now_ms;

    double outputTimestampsDurationMs(size_t p_position) {
        if (p_position < 0) {
            return 0;
        }
        if (this->timestamps_duration_ms.size() < (p_position + 1)) {
            return 0;
        }
        return this->timestamps_duration_ms[p_position];
    }

    double outputTimestampsTillNowMs(size_t p_position) {
        if (p_position < 0) {
            return 0;
        }
        if (this->timestamps_till_now_ms.size() < (p_position + 1)) {
            return 0;
        }
        return this->timestamps_till_now_ms[p_position];
    }


    std::chrono::time_point<std::chrono::steady_clock> getNowTimePoint() {
        return std::chrono::steady_clock::now();
    };

    void pushTimestamp(size_t p_position, std::chrono::time_point<std::chrono::steady_clock> p_time_point = std::chrono::steady_clock::now()) {
        if (p_position < 0) {
            return;
        }
        if (this->last_timestamps.size() < (p_position + 1)) {
            this->last_timestamps.resize(p_position + 1);
        }
        shawn_vector.unifySize(this->last_timestamps, this->last_last_timestamps);
        this->last_last_timestamps[p_position] = this->last_timestamps[p_position];
        this->last_timestamps[p_position] = p_time_point;
    }

    double getTimestampsDurationMs(size_t p_position) {
        if (p_position < 0) {
            return 0;
        }
        if (this->last_timestamps.size() < (p_position + 1)) {
            return 0;
        }
        return 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(this->last_timestamps[p_position] - this->last_last_timestamps[p_position]).count();
    }

    double getTimestampsTillNowMs(size_t p_position) {
        if (p_position < 0) {
            return 0;
        }
        if (this->last_timestamps.size() < (p_position + 1)) {
            return 0;
        }
        return 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - this->last_timestamps[p_position]).count();
    }

    void setSimpleStartTimePoint() {
        this->simple_start_time_point = std::chrono::steady_clock::now();
    }

    double getSimpleDurationMs() {
        return 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - this->simple_start_time_point).count();
    }

    void updateTimestampsDuration() {
//        shawn_vector.unifySize(this->last_timestamps, this->last_last_timestamps);
        shawn_vector.unifySize(this->last_timestamps, this->timestamps_duration_ms);
        for (size_t i = 0; i < this->last_timestamps.size(); ++i) {
            this->timestamps_duration_ms[i] = 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(this->last_timestamps[i] - this->last_last_timestamps[i]).count();
        }
    }

    void updateTimestampsTillNow(std::chrono::time_point<std::chrono::steady_clock> p_now = std::chrono::steady_clock::now()) {
        shawn_vector.unifySize(this->last_timestamps, this->timestamps_till_now_ms);
        for (size_t i = 0; i < this->last_timestamps.size(); ++i) {
            this->timestamps_till_now_ms[i] = 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(p_now - this->last_timestamps[i]).count();
        }
    }

    bool checkTimestampsDuration(double p_min = -1, double p_max = -1) {
        this->updateTimestampsDuration();
        if (this->timestamps_duration_ms.empty()) {
            return false;
        }
        this->error_positions.clear();
        if (p_min > 0) {
            for (size_t i = 0; i < this->timestamps_duration_ms.size(); ++i) {
                if (this->timestamps_duration_ms[i] < p_min) {
                    this->error_positions.emplace_back(i);
                }
            }
        }
        if (p_max > 0) {
            for (size_t i = 0; i < this->timestamps_duration_ms.size(); ++i) {
                if (this->timestamps_duration_ms[i] > p_max) {
                    this->error_positions.emplace_back(i);
                }
            }
        }
        return this->error_positions.empty();
    }

    bool checkTimestampsTillNow(double p_min = -1, double p_max = -1) {
        this->updateTimestampsTillNow();
        if (this->timestamps_till_now_ms.empty()) {
            return false;
        }
        this->error_positions.clear();
        if (p_min > 0) {
            for (size_t i = 0; i < this->timestamps_till_now_ms.size(); ++i) {
                if (this->timestamps_till_now_ms[i] < p_min) {
                    this->error_positions.emplace_back(i);
                }
            }
        }
        if (p_max > 0) {
            for (size_t i = 0; i < this->timestamps_till_now_ms.size(); ++i) {
                if (this->timestamps_till_now_ms[i] > p_max) {
                    this->error_positions.emplace_back(i);
                }
            }
        }
        return this->error_positions.empty();
    }

//    bool checkSingleTimestampDuration (size_t p_position, double p_min = -1, double p_max = -1) {
//        if (p_position < 0) {
//            return false;
//        }
//        if (!shawn_vector.checkSizeSame(this->last_timestamps, this->last_last_timestamps)) {
//            return false;
//        }
//        if (this->last_timestamps.size() < (p_position + 1)) {
//            return false;
//        }
//        if (p_min > 0) {
//        }
//        if (p_max > 0) {
//        }
//    }
};

}

#endif //SHAWN_TIME_HPP
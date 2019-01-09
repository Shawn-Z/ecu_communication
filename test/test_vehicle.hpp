#ifndef ECU_COMMUNICATION_TEST_VEHICLE_HPP
#define ECU_COMMUNICATION_TEST_VEHICLE_HPP

#include <vector>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace three_one_test {
    std::vector<double_t> accCurveGenerate(double_t p_udp_send_period_s, double_t p_acc, double_t p_start_speed, double_t p_end_speed) {
        std::vector<double_t> speed_curve;
        speed_curve.emplace_back(p_start_speed);
        while (speed_curve.back() < p_end_speed) {
            speed_curve.emplace_back(speed_curve.back() + p_acc * p_udp_send_period_s);
        }
        for (auto &cell: speed_curve) {
            cell = std::max(cell, 0.0);
        }
        return speed_curve;
    }

    std::vector<double_t> decCurveGenerate(double_t p_udp_send_period_s, double_t p_dec, double_t p_start_speed, double_t p_end_speed) {
        std::vector<double_t> speed_curve;
        speed_curve.emplace_back(p_start_speed);
        while (speed_curve.back() > p_end_speed) {
            speed_curve.emplace_back(speed_curve.back() + p_dec * p_udp_send_period_s);
        }
        for (auto &cell: speed_curve) {
            cell = std::max(cell, 0.0);
        }
        return speed_curve;
    }

    std::vector<double_t> uniformCurveGenerate(double_t p_udp_send_period_s, double_t p_time, double_t p_speed) {
        std::vector<double_t> speed_curve((long long int)(p_time / p_udp_send_period_s), p_speed);
        return speed_curve;
    }

    std::vector<double_t> uniform_acc_uniform_dec(double_t p_udp_send_period_s, double_t p_max_speed, double_t p_min_speed, double_t p_uniform_time) {
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        speed_curve.emplace_back(p_min_speed);
        std::vector<double_t> acc_list;
        for (int i = 1; i < 26; ++i) {
            acc_list.emplace_back(0.1 * i);
        }
        for (int j = 0; j < acc_list.size(); ++j) {
            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_uniform_time, speed_curve.back());
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = accCurveGenerate(p_udp_send_period_s, acc_list[j], p_min_speed, p_max_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_uniform_time, speed_curve.back());
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = decCurveGenerate(p_udp_send_period_s, -acc_list[j], p_max_speed, p_min_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }

    std::vector<double_t> random_uniform_acc_uniform_dec(double_t p_udp_send_period_s, double_t p_max_speed, double_t p_min_speed, double_t p_uniform_time) {
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        speed_curve.emplace_back(p_min_speed);
        std::vector<double_t> acc_list;
        for (int i = 1; i < 26; ++i) {
            acc_list.emplace_back(0.1 * i);
        }
        std::random_shuffle(acc_list.begin(), acc_list.end());
        for (int j = 0; j < acc_list.size(); ++j) {
            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_uniform_time, speed_curve.back());
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = accCurveGenerate(p_udp_send_period_s, acc_list[j], p_min_speed, p_max_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_uniform_time, speed_curve.back());
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = decCurveGenerate(p_udp_send_period_s, -acc_list[j], p_max_speed, p_min_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }

    std::vector<double_t> acc_dec(double_t p_udp_send_period_s, double_t p_max_speed, double_t p_min_speed, double_t p_uniform_time) {
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        speed_curve.emplace_back(p_min_speed);
        std::vector<double_t> acc_list;
        for (int i = 1; i < 26; ++i) {
            acc_list.emplace_back(0.1 * i);
        }
        for (int j = 0; j < acc_list.size(); ++j) {
            tmp_vector = accCurveGenerate(p_udp_send_period_s, acc_list[j], p_min_speed, p_max_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = decCurveGenerate(p_udp_send_period_s, -acc_list[j], p_max_speed, p_min_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }

    std::vector<double_t> random_acc_dec(double_t p_udp_send_period_s, double_t p_max_speed, double_t p_min_speed, double_t p_uniform_time) {
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        speed_curve.emplace_back(p_min_speed);
        std::vector<double_t> acc_list;
        for (int i = 1; i < 26; ++i) {
            acc_list.emplace_back(0.1 * i);
        }
        std::random_shuffle(acc_list.begin(), acc_list.end());
        for (int j = 0; j < acc_list.size(); ++j) {
            tmp_vector = accCurveGenerate(p_udp_send_period_s, acc_list[j], p_min_speed, p_max_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = decCurveGenerate(p_udp_send_period_s, -acc_list[j], p_max_speed, p_min_speed);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }

    std::vector<double_t> pure_step_up(double_t p_udp_send_period_s, double_t p_time) {
        double_t speed_lists[20] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
        std::vector<double_t> speed_list;
        speed_list.assign(speed_lists, speed_lists + 20);
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        for (int i = 0; i < speed_list.size(); ++i) {
            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_time, 0.0);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_time, speed_list[i]);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }

    std::vector<double_t> step_up(double_t p_udp_send_period_s, double_t p_time) {
        double_t speed_lists[20] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
        std::vector<double_t> speed_list;
        speed_list.assign(speed_lists, speed_lists + 20);
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        for (int i = 0; i < speed_list.size(); ++i) {
            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_time, speed_list[i]);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }



    std::vector<double_t> step_down(double_t p_udp_send_period_s, double_t p_time) {
        double_t speed_lists[20] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
        std::vector<double_t> speed_list;
        speed_list.assign(speed_lists, speed_lists + 20);
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        for (int i = 0; i < speed_list.size(); ++i) {
            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_time, 0.0);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());

            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_time, speed_list[speed_list.size() - i - 1]);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }



    std::vector<double_t> pure_step_down(double_t p_udp_send_period_s, double_t p_time) {
        double_t speed_lists[20] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
        std::vector<double_t> speed_list;
        speed_list.assign(speed_lists, speed_lists + 20);
        std::vector<double_t> tmp_vector;
        std::vector<double_t> speed_curve;
        for (int i = 0; i < speed_list.size(); ++i) {
            tmp_vector = uniformCurveGenerate(p_udp_send_period_s, p_time, speed_list[speed_list.size() - i - 1]);
            speed_curve.insert(speed_curve.end(), tmp_vector.begin(), tmp_vector.end());
        }
        return speed_curve;
    }


    std::vector<double_t> test_curve() {
        std::vector<double_t> tmp;
        std::vector<double_t> speed_curve;

        tmp = three_one_test::uniform_acc_uniform_dec(0.015, 10.0, 0.0, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::random_uniform_acc_uniform_dec(0.015, 10.0, 0.0, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::acc_dec(0.015, 10.0, 0.0, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::random_acc_dec(0.015, 10.0, 0.0, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::pure_step_up(0.015, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::step_up(0.015, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::pure_step_down(0.015, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        tmp = three_one_test::step_down(0.015, 10.0);
        speed_curve.insert(speed_curve.end(), tmp.begin(), tmp.end());

        return speed_curve;
    }
}

#endif //ECU_COMMUNICATION_TEST_VEHICLE_HPP
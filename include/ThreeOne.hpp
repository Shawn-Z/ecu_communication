#ifndef ECU_COMMUNICATION_THREE_ONE_HPP
#define ECU_COMMUNICATION_THREE_ONE_HPP

namespace three_one_control {

enum class halt {
    off = 0,
    on = 1
};

enum class work_mode {
    wrong_mode = 0,
    curvature_and_vehicle_speed = 1,
    vehicle_speed = 2,
    curvature = 3,
    differential_speed = 4,
    halt = 5
};

enum class vehicle_gear {
    N = 0,
    D = 1,
    R = 2
};

enum class vehicle_turn_to {
    left = 0,
    right = 1
};

enum class left_wheel_rotate {
    forward = 0,
    retreat = 1
};

enum class right_wheel_rotate {
    forward = 0,
    retreat = 1
};

enum class parking_control {
    off = 0,
    on = 1
};

enum class ring_control {
    off = 0,
    on = 1
};

enum class forward_big_light {
    off = 0,
    on = 1,
    flash = 2
};

enum class wide_taillight {
    off = 0,
    on = 1
};

enum class turn_light {
    all_off = 0,
    left_on_right_off = 1,
    left_off_right_on = 2,
    all_on = 3,
    left_flash_right_off = 4,
    left_off_right_flash = 5,
    all_flash = 6
};

enum class cylinder_select {
    none = 0,
    left_one = 1,
    left_two = 2,
    left_three = 3,
    left_four = 4,
    right_one = 5,
    right_two = 6,
    right_three = 7,
    right_four = 8,
};

enum class suspension_select {
    none = 0,
    one = 1,
    two = 2,
    three = 3,
    four = 4,
    one_three = 5,
    one_four = 6,
    two_three = 7,
    two_four = 8
};

enum class suspension_work_mode {
    up_down = 0,
    roll = 1,
    pitch =2
};

enum class suspension_up_down {
    up = 1,
    down = 2,
    keep = 3
};

enum class suspension_roll {
    left_high_right_low = 1,
    left_low_right_high = 2,
    keep = 3
};

enum class suspension_pitch {
    forward_high_back_low = 1,
    forward_low_back_high = 2,
    keep = 3
};

enum class suspension_cylinder_select_mode {
    all = 0,
    single = 1
};

enum class suspension_cylinder_motor_control {
    off = 0,
    on = 1
};

enum class vertical_wall_mode {
    normal_driving = 0,
    up_vertical_wall = 1,
    down_vertical_wall = 2
};

enum class tailgate_control {
    open = 1,
    close = 2,
    keep = 3
};

enum class fix_two_chamber_valve {
    not_fixed = 0,
    fixed = 1
};

enum class weapon_330 {
    off = 0,
    on = 1
};

enum class entrenchment {
    disable = 0,
    enable = 1
};

enum class weapon_28 {
    off = 0,
    on = 1
};

}

namespace three_one_feedback {

enum class current_gear {
    N = 1,
    D = 2,
    R = 3,
    spin = 4
};

enum class left_wheel_rotate {
    forward = 0,
    retreat = 1
};

enum class right_wheel_rotate {
    forward = 0,
    retreat = 1
};

enum class tailgate_state {
    middle = 0,
    open = 1,
    close = 2
};

enum class vertical_wall_status {
    not_ok = 0,
    up_one_ok = 1,
    up_two_ok = 2,
    up_three_ok = 3,
    up_four_ok = 4,
    down_one_ok = 5,
    down_two_ok = 6,
    down_three_ok = 7,
    down_four_ok = 8
};

enum class park_status {
    not_parked = 0,
    parked = 1
};

enum class spin_status {
    not_spin = 0,
    counterclockwise = 1,
    clockwise = 2
};

enum class control_mode {
    ERROR = 0,
    autonomous = 1,
    remote = 2
};

enum class operation_mode {
    three_one = 0,
    ours = 1
};

enum class two_chamber_valve {
    not_fixed = 0,
    fixed = 1
};

enum class weapon_330 {
    off = 0,
    on = 1
};

enum class entrenchment {
    not_ready = 0,
    ready = 1,
};

enum class weapon_28 {
    off = 0,
    on = 1
};

}

#endif //ECU_COMMUNICATION_THREE_ONE_HPP
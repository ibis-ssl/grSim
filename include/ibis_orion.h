//
// Created by hans on 24/05/25.
//
#include <cstdint>
#include <math.h>

#ifndef GRSIM_IBIS_ORION_H
#define GRSIM_IBIS_ORION_H

#define OMNI_OUTPUT_GAIN_FF_TARGET_NOW (1.2)
#define OMNI_OUTPUT_GAIN_FF_TARGET_FINAL_DIFF (3.0)  //2.0
#define FF_TARGET_FINAL_DIFF_LIMIT (1.5)

#define OUTPUT_XY_LIMIT (10)  //

#define OMEGA_GAIN_KP (160.0)
#define OMEGA_GAIN_KD (4000.0)

#define OMEGA_LIMIT (20.0)  // ~ rad/s

// MAX
#define ACCEL_LIMIT (5.0)       // m/ss
#define ACCEL_LIMIT_BACK (4.0)  // m/ss

#define MAIN_LOOP_CYCLE (500)

using uint8_t = std::uint8_t;

enum {
    MAIN_MODE_COMBINATION_CONTROL = 0,
    MAIN_MODE_SPEED_CONTROL_ONLY,
    MAIN_MODE_CMD_DEBUG_MODE,
    MAIN_MODE_MOTOR_TEST,
    MAIN_MODE_DRIBBLER_TEST,
    MAIN_MODE_KICKER_AUTO_TEST,
    MAIN_MODE_KICKER_MANUAL,
    MAIN_MODE_NONE,
    MAIN_MODE_MOTOR_CALIBRATION,
    MAIN_MODE_ERROR,
};

typedef struct
{
    float * buffer;  // float型のデータを格納する配列
    int size;        // バッファのサイズ
    int front;       // データの先頭位置
    int rear;        // データの末尾位置
    int count;       // データの数
} RingBuffer;

typedef struct
{
    float yaw_angle, pre_yaw_angle;
    float yaw_angle_rad, pre_yaw_angle_rad;
    float yaw_angle_diff_integral;
} imu_t;

//typedef struct
//{
//    uint8_t error_no[8];
//    float motor_feedback[5];           // rps
//    float motor_feedback_velocity[5];  // m/s
//    float power_voltage[7];
//    float temperature[7];
//    float current[5];
//    uint8_t ball_detection[4];
//    uint32_t board_rx_timeout[BOARD_ID_MAX];
//} can_raw_t;

typedef struct
{
    float target_theta, global_vision_theta;
    float drible_power;
    float kick_power;
    bool chip_en;
    float local_target_speed[2];
    float local_target_speed_scalar;
    float global_robot_position[2];
    float global_target_position[2];
    float global_ball_position[2];
    uint8_t allow_local_flags;
    int ball_local_x, ball_local_y, ball_local_radius, ball_local_FPS;
    bool vision_lost_flag, local_vision_en_flag, keeper_mode_en_flag, stop_request_flag, dribbler_up_flag;
    bool pre_vision_lost_flag;
    uint32_t latency_time_ms;
} ai_cmd_t;

typedef struct
{
    float enc_angle[5];
    float pre_enc_angle[5];
    float angle_diff[4];
} motor_t;

typedef struct
{
    float velocity[2];             // m/s 速度指令値の入力
    float local_vel[2];            // m/s 上記とほぼ同じ
    float local_vel_now[2];        // 台形制御指令値
    float local_vel_ff_factor[2];  // 最終指令速度への追従を高めるためのFF項目
    float global_vel_now[2];       // ターゲットグローバル速度
    //float global_pos[2];           // 上記で移動するX,Y座標
} target_t;

typedef struct
{
    float vel_error_xy[2];
    float vel_error_scalar, vel_error_rad;
} accel_vector_t;

typedef struct
{
    float velocity[2];
    float raw_odom[2];
    float floor_odom[2];
    float odom[2];
    int16_t raw[2];
    float raw_diff[2];
    uint16_t quality;
    int integral_loop_cnt, loop_cnt_debug;
    float pre_yaw_angle_rad, diff_yaw_angle_rad;
} mouse_t;

typedef struct
{
    float travel_distance[2];
    //float global_odom_diff[2], robot_pos_diff[2];
    float odom[2], pre_odom[2], odom_raw[2];
    float odom_speed[2];
    float local_odom_speed[2];
    RingBuffer * local_speed_log[2];
    float local_odom_speed_mvf[2];
} omni_t;

typedef struct
{
    RingBuffer * odom_log[2];
    float global_odom_vision_diff[2];  // vision座標を基準にした移動距離(global系)
    float vision_based_position[2];    // Visionによって更新された自己位置
    float position_diff[2];            // ai_cmdとvision_based_positionの差分
    float pre_global_target_position[2];
    float move_dist;         // Visionとtargetが更新されてからの移動量
    float targed_dist_diff;  // Visionが更新された時点での現在地とtargetの距離
    float local_target_diff[2];
} integration_control_t;

typedef struct
{
    volatile float velocity[2];
    volatile float omega;
    volatile float accel[2];
    volatile float motor_voltage[4];
} output_t;

typedef struct
{
    bool connected_ai;
    bool connected_cm4;
    bool already_connected_ai;
    uint8_t check_pre;
    uint8_t check_ver;
    float cmd_rx_frq;
    uint32_t vision_update_cycle_cnt;
    uint32_t latest_ai_cmd_update_time;
    uint32_t latest_cm4_cmd_update_time;
} connection_t;

typedef struct
{
    bool error_flag, stop_flag;
    uint16_t error_id, error_info;
    float error_value;
    uint32_t error_resume_cnt;
    uint8_t main_mode;
    uint32_t system_time_ms;
    uint32_t stop_flag_request_time;
    uint16_t kick_state;
    uint32_t sw_data;
} system_t;

typedef struct
{
    volatile uint32_t print_idx;
    volatile uint32_t main_loop_cnt, true_cycle_cnt, motor_zero_cnt;
    volatile float vel_radian, out_total_spin, fb_total_spin, pre_yaw_angle;
    volatile float true_out_total_spi, true_fb_toral_spin, true_yaw_speed, limited_output;
    volatile bool print_flag, acc_step_down_flag, theta_override_flag;
    volatile bool latency_check_enabled;
    volatile int latency_check_seq_cnt;
    volatile float rotation_target_theta;
    volatile uint32_t uart_rx_itr_cnt;
} debug_t;

float normalizeAngle(float angle_rad)
{
    while (angle_rad > M_PI) {
        angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
}

float getAngleDiff(float angle_rad1, float angle_rad2)
{
    angle_rad1 = normalizeAngle(angle_rad1);
    angle_rad2 = normalizeAngle(angle_rad2);
    if (fabs(angle_rad1 - angle_rad2) > M_PI) {
        if (angle_rad1 > angle_rad2) {
            return angle_rad1 - (angle_rad2 + 2 * M_PI);
        } else {
            return (angle_rad1 + 2 * M_PI) - angle_rad2;
        }
    } else {
        return angle_rad1 - angle_rad2;
    }
}

void theta_control(float target_theta, accel_vector_t * acc_vel, output_t * output, imu_t * imu)
{
    // PID
    output->omega = (getAngleDiff(target_theta, imu->yaw_angle_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_angle_rad, imu->pre_yaw_angle_rad) * OMEGA_GAIN_KD);

    if (output->omega > OMEGA_LIMIT) {
        output->omega = OMEGA_LIMIT;
    }
    if (output->omega < -OMEGA_LIMIT) {
        output->omega = -OMEGA_LIMIT;
    }
    //output->omega = 0;
}

void local_feedback(integration_control_t * integ, imu_t * imu, system_t * sys, target_t * target, ai_cmd_t * ai_cmd)
{
    const float CMB_CTRL_FACTOR_LIMIT = (3.0);     // [m/s]
    const float CMB_CTRL_DIFF_DEAD_ZONE = (0.03);  // [m]
    const float CMB_CTRL_GAIN = (10.0);
    const float CMB_CTRL_DIFF_LIMIT = (CMB_CTRL_FACTOR_LIMIT / CMB_CTRL_GAIN);

    // グローバル→ローカル座標系
    integ->local_target_diff[0] = integ->position_diff[0] * cos(-imu->yaw_angle_rad) - integ->position_diff[1] * sin(-imu->yaw_angle_rad);
    integ->local_target_diff[1] = integ->position_diff[0] * sin(-imu->yaw_angle_rad) + integ->position_diff[1] * cos(-imu->yaw_angle_rad);

    for (int i = 0; i < 2; i++) {
        // デバッグ用にomni->odomをそのままと、target_posにsepeed使う
        // 速度制御はodomベースなのでちょっとおかしなことになる
        //integ->local_target_diff[i] = omni->odom[i] - ai_cmd->local_target_speed[i];

        // 精密性はそれほどいらないので、振動対策に不感帯入れる
        if (integ->local_target_diff[i] < CMB_CTRL_DIFF_DEAD_ZONE && integ->local_target_diff[i] > -CMB_CTRL_DIFF_DEAD_ZONE) {
            integ->local_target_diff[i] = 0;
        }

        // ゲインは x10
        // 吹き飛び対策で+-3.0 m/sを上限にする
        if (integ->local_target_diff[i] < -CMB_CTRL_DIFF_LIMIT) {
            integ->local_target_diff[i] = -CMB_CTRL_DIFF_LIMIT;
        } else if (integ->local_target_diff[i] > CMB_CTRL_DIFF_LIMIT) {
            integ->local_target_diff[i] = CMB_CTRL_DIFF_LIMIT;
        }

        if (sys->main_mode == MAIN_MODE_COMBINATION_CONTROL) {
            // 位置フィードバック項目のx10はゲイン (ベタ打ち)

            //target->velocity[i] = +(integ->local_target_diff[i] * CMB_CTRL_GAIN);  //ローカル統合制御あり(位置フィードバックのみ)
            //target->velocity[i] = ai_cmd->local_target_speed[i] * 0.5 + (integ->local_target_diff[i] * CMB_CTRL_GAIN) * 0.5;  //ローカル統合制御あり

            if (ai_cmd->local_target_speed[i] * integ->local_target_diff[i] < 0) {                                  // 位置フィードバック項が制動方向の場合
                target->velocity[i] = ai_cmd->local_target_speed[i] + (integ->local_target_diff[i] * CMB_CTRL_GAIN);  //ローカル統合制御あり
            } else {
                target->velocity[i] = ai_cmd->local_target_speed[i];  // ローカル統合制御なし
            }
            //target->velocity[i] = (integ->local_target_diff[i] * CMB_CTRL_GAIN);  //ローカル統合制御あり

        } else {
            target->velocity[i] = ai_cmd->local_target_speed[i];  // ローカル統合制御なし
        }
    }
}

void accel_control(accel_vector_t * acc_vel, output_t * output, target_t * target)
{
    target->local_vel[0] = target->velocity[0];
    target->local_vel[1] = target->velocity[1];

    // XY -> rad/scalarに変換

    for (int i = 0; i < 2; i++) {
        acc_vel->vel_error_xy[i] = target->local_vel[i] - target->local_vel_now[i];
    }
    acc_vel->vel_error_scalar = pow(pow(acc_vel->vel_error_xy[0], 2) + pow(acc_vel->vel_error_xy[1], 2), 0.5);
    if (acc_vel->vel_error_xy[0] != 0 || acc_vel->vel_error_xy[1] != 0) {
        acc_vel->vel_error_rad = atan2(acc_vel->vel_error_xy[1], acc_vel->vel_error_xy[0]);
    }

    // 目標速度と差が小さい場合は目標速度をそのまま代入する
    // 目標速度が連続的に変化する場合に適切でないかも
    if (acc_vel->vel_error_scalar < ACCEL_LIMIT / MAIN_LOOP_CYCLE) {
        target->local_vel_now[0] = target->local_vel[0];
        target->local_vel_now[1] = target->local_vel[1];
        output->accel[0] = 0;
        output->accel[1] = 0;
        return;
    }

    // スカラは使わず、常に最大加速度
    output->accel[0] = cos(acc_vel->vel_error_rad) * ACCEL_LIMIT / MAIN_LOOP_CYCLE;
    output->accel[1] = sin(acc_vel->vel_error_rad) * ACCEL_LIMIT / MAIN_LOOP_CYCLE;

    // バック方向だけ加速度制限
    if (output->accel[0] < -(ACCEL_LIMIT_BACK / MAIN_LOOP_CYCLE)) {
        output->accel[0] = -(ACCEL_LIMIT_BACK / MAIN_LOOP_CYCLE);
    }

    // 減速方向は制動力2倍
    // 2倍は流石に無理があるので1.8
    for (int i = 0; i < 2; i++) {
        if (target->local_vel_now[i] * output->accel[i] < 0) {
            output->accel[i] *= 2.0;
        }

        // 目標座標を追い越した場合、加速度を2倍にして現実の位置に追従
        // 現在座標も速度制御されたタイヤで見ているので、あまりｱﾃにならない
        /*if ((omni->robot_pos_diff[i] > 0 && output->accel[i] > 0) || (omni->robot_pos_diff[i] < 0 && output->accel[i] < 0)) {
          //output->accel[i] *= 1.5;
        }*/
    }
}

void speed_control(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
    //target->local_vel[0] = target->velocity[0];
    //target->local_vel[1] = target->velocity[1];

    // 500Hz, m/s -> m / cycle
    for (int i = 0; i < 2; i++) {
        target->local_vel_now[i] += output->accel[i];
    }

    // ローカル→グローバル座標系
    // ロボットが回転しても、慣性はグローバル座標系に乗るので、加速度はグローバル座標系に変換してから加算
    target->global_vel_now[0] += (output->accel[0]) * cos(imu->yaw_angle_rad) - (output->accel[1]) * sin(imu->yaw_angle_rad);
    target->global_vel_now[1] += (output->accel[0]) * sin(imu->yaw_angle_rad) + (output->accel[1]) * cos(imu->yaw_angle_rad);

    // 次回の計算のためにローカル座標系での速度も更新
    target->local_vel_now[0] = target->global_vel_now[0] * cos(-imu->yaw_angle_rad) - target->global_vel_now[1] * sin(-imu->yaw_angle_rad);
    target->local_vel_now[1] = target->global_vel_now[0] * sin(-imu->yaw_angle_rad) + target->global_vel_now[1] * cos(-imu->yaw_angle_rad);

    // 速度次元での位置フィードバックは不要になったので、global_posまわりは使わない
    //target->global_pos[0] += target->global_vel_now[0] / MAIN_LOOP_CYCLE;  // speed to position
    //target->global_pos[1] += target->global_vel_now[1] / MAIN_LOOP_CYCLE;  // speed to position

    // ここから位置制御
    for (int i = 0; i < 2; i++) {
        // targetとodomの差分に上限をつける(吹っ飛び対策)
        // 出力が上限に張り付いたら、出力制限でそれ以上の加速度は出しようがないのでそれに合わせる
        /*float odom_diff_max = (float)OUTPUT_XY_LIMIT / OMNI_OUTPUT_GAIN_KP;
        if (target->global_pos[i] - omni->odom[i] > odom_diff_max) {
          target->global_pos[i] = omni->odom[i] + odom_diff_max;
        } else if (target->global_pos[i] - omni->odom[i] < -odom_diff_max) {
          target->global_pos[i] = omni->odom[i] - odom_diff_max;
        }*/

        // 速度に対する応答性を稼ぐ
        target->local_vel_ff_factor[i] = (target->local_vel[i] - omni->local_odom_speed_mvf[i]) * OMNI_OUTPUT_GAIN_FF_TARGET_FINAL_DIFF;
        float tmp = fabs(output->accel[i] * MAIN_LOOP_CYCLE / ACCEL_LIMIT) * FF_TARGET_FINAL_DIFF_LIMIT;
        if (target->local_vel_ff_factor[i] > tmp) {
            target->local_vel_ff_factor[i] = tmp;
        } else if (target->local_vel_ff_factor[i] < -tmp) {
            target->local_vel_ff_factor[i] = -tmp;
        }
    }

    // odom基準の絶対座標系
    // omni->global_odom_diff[i] = omni->odom[i] - target->global_pos[i];

    // グローバル→ローカル座標系
    /* omni->robot_pos_diff[0] = omni->global_odom_diff[0] * cos(-imu->yaw_angle_rad) - omni->global_odom_diff[1] * sin(-imu->yaw_angle_rad);
    omni->robot_pos_diff[1] = omni->global_odom_diff[0] * sin(-imu->yaw_angle_rad) + omni->global_odom_diff[1] * cos(-imu->yaw_angle_rad);
    */
    // local_vel_ff_factorに含まれるので要らなくなった
    /* - omni->local_odom_speed[0] * OMNI_OUTPUT_GAIN_KD */
    /* - omni->local_odom_speed[1] * OMNI_OUTPUT_GAIN_KD */

    // 位置フィードバックは速度指令にいれるので、速度制御には関与させない
    /* -omni->robot_pos_diff[0] * OMNI_OUTPUT_GAIN_KP + */
    /* omni->robot_pos_diff[1] * OMNI_OUTPUT_GAIN_KP + */

    output->velocity[0] = target->local_vel_now[0] * OMNI_OUTPUT_GAIN_FF_TARGET_NOW + target->local_vel_ff_factor[0];
    output->velocity[1] = target->local_vel_now[1] * OMNI_OUTPUT_GAIN_FF_TARGET_NOW + target->local_vel_ff_factor[1];
}

void output_limit(output_t * output, debug_t * debug)
{
    if (debug->acc_step_down_flag) {
        debug->limited_output = 0;  //スリップしてたら移動出力を0にする(仮)
    } else {
        debug->limited_output = OUTPUT_XY_LIMIT;
    }

    float limit_gain = 0;
    if (output->velocity[0] > debug->limited_output) {
        limit_gain = output->velocity[0] / debug->limited_output;
        output->velocity[0] = debug->limited_output;
        output->velocity[1] /= limit_gain;
    } else if (output->velocity[0] < -debug->limited_output) {
        limit_gain = -output->velocity[0] / debug->limited_output;
        output->velocity[0] = -debug->limited_output;
        output->velocity[1] /= limit_gain;
    }

    if (output->velocity[1] > debug->limited_output) {
        limit_gain = output->velocity[1] / debug->limited_output;
        output->velocity[1] = debug->limited_output;
        output->velocity[0] /= limit_gain;
    } else if (output->velocity[1] < -debug->limited_output) {
        limit_gain = -output->velocity[1] / debug->limited_output;
        output->velocity[1] = -debug->limited_output;
        output->velocity[0] /= limit_gain;
    }
}

#endif //GRSIM_IBIS_ORION_H

/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ibis_command_receiver.h"

#include <cmath>
#include <algorithm>

// Byte offsets in the 64-byte RobotCommandSerializedV2 (from crane's robot_packet.h)
enum IbisAddress {
    HEADER              = 0,
    CHECK_COUNTER       = 1,
    VISION_GLOBAL_X_H   = 2,
    VISION_GLOBAL_X_L   = 3,
    VISION_GLOBAL_Y_H   = 4,
    VISION_GLOBAL_Y_L   = 5,
    VISION_GLOBAL_TH_H  = 6,
    VISION_GLOBAL_TH_L  = 7,
    TARGET_GLOBAL_TH_H  = 8,
    TARGET_GLOBAL_TH_L  = 9,
    KICK_POWER          = 10,
    DRIBBLE_POWER       = 11,
    ACCEL_LIMIT_H       = 12,
    ACCEL_LIMIT_L       = 13,
    LINEAR_VEL_LIMIT_H  = 14,
    LINEAR_VEL_LIMIT_L  = 15,
    ANGULAR_VEL_LIMIT_H = 16,
    ANGULAR_VEL_LIMIT_L = 17,
    LATENCY_MS_H        = 18,
    LATENCY_MS_L        = 19,
    ELAPSED_VISION_H    = 20,
    ELAPSED_VISION_L    = 21,
    FLAGS               = 22,
    CONTROL_MODE        = 23,
    CONTROL_MODE_ARGS   = 24,
    // MODE_ARGS_SIZE = 8, so args end at offset 31
    TARGET_POS_X_H      = 32,  // CONTROL_MODE_ARGS + 8
    TARGET_POS_X_L      = 33,
    TARGET_POS_Y_H      = 34,
    TARGET_POS_Y_L      = 35,
    TERMINAL_VEL_H      = 36,
    TERMINAL_VEL_L      = 37,
};

enum IbisFlagBit {
    IS_VISION_AVAILABLE = 0,
    ENABLE_CHIP         = 1,
    STOP_EMERGENCY      = 3,
};

static const double IBIS_DT = 1.0 / 30.0;

IbisCommandReceiver::IbisCommandReceiver(ConfigWidget* cfg)
    : cfg_(cfg)
{
}

float IbisCommandReceiver::decodeTwoByte(uint8_t high, uint8_t low, float range)
{
    uint16_t two_byte = (static_cast<uint16_t>(high) << 8) | low;
    return static_cast<float>(two_byte - 32767.f) / 32767.f * range;
}

IbisCommandReceiver::IbisCommand IbisCommandReceiver::deserialize(const uint8_t* d)
{
    IbisCommand cmd;
    cmd.check_counter        = d[CHECK_COUNTER];
    cmd.vision_global_pos[0] = decodeTwoByte(d[VISION_GLOBAL_X_H], d[VISION_GLOBAL_X_L], 32.767f);
    cmd.vision_global_pos[1] = decodeTwoByte(d[VISION_GLOBAL_Y_H], d[VISION_GLOBAL_Y_L], 32.767f);
    cmd.target_global_theta  = decodeTwoByte(d[TARGET_GLOBAL_TH_H], d[TARGET_GLOBAL_TH_L], static_cast<float>(M_PI));
    cmd.kick_power          = d[KICK_POWER] / 20.f;
    cmd.dribble_power       = d[DRIBBLE_POWER] / 20.f;
    cmd.acceleration_limit  = decodeTwoByte(d[ACCEL_LIMIT_H], d[ACCEL_LIMIT_L], 32.767f);
    cmd.linear_velocity_limit  = decodeTwoByte(d[LINEAR_VEL_LIMIT_H], d[LINEAR_VEL_LIMIT_L], 32.767f);
    cmd.angular_velocity_limit = decodeTwoByte(d[ANGULAR_VEL_LIMIT_H], d[ANGULAR_VEL_LIMIT_L], 32.767f);

    uint8_t flags = d[FLAGS];
    cmd.enable_chip     = (flags >> ENABLE_CHIP) & 0x01;
    cmd.stop_emergency  = (flags >> STOP_EMERGENCY) & 0x01;

    // POLAR_VELOCITY_TARGET_MODE args at CONTROL_MODE_ARGS (offset 24)
    cmd.polar_velocity_r     = decodeTwoByte(d[CONTROL_MODE_ARGS + 0], d[CONTROL_MODE_ARGS + 1], 32.767f);
    cmd.polar_velocity_theta = decodeTwoByte(d[CONTROL_MODE_ARGS + 2], d[CONTROL_MODE_ARGS + 3], 32.767f);

    return cmd;
}

double IbisCommandReceiver::calculateAccelerationLimit(double current_speed, double target_speed) const
{
    // Matches sim_sender.cpp SenderBase::calculateAccelerationLimit logic:
    // lower limit when braking at high speed, higher limit when accelerating
    const double ACC_SPEEDUP = cfg_->robotSettings.AccSpeedupAbsoluteMax;
    const double ACC_BRAKE   = cfg_->robotSettings.AccBrakeAbsoluteMax;
    if (target_speed < current_speed) {
        return ACC_BRAKE;
    }
    return ACC_SPEEDUP;
}

void IbisCommandReceiver::applyCommand(int robot_id, const IbisCommand& cmd, Robot* robot)
{
    PerRobotState& state = robot_states_[robot_id];

    if (cmd.stop_emergency) {
        robot->setSpeed(0.0, 0.0, 0.0);
        state.prev_vx = 0.0;
        state.prev_vy = 0.0;
        return;
    }

    // Current robot orientation in radians (grSim convention: negative of degrees)
    double current_theta = -robot->getDir() * M_PI / 180.0;

    double theta_error = cmd.target_global_theta - current_theta;
    while (theta_error >  M_PI) theta_error -= 2.0 * M_PI;
    while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

    double omega = IBIS_THETA_P_GAIN * theta_error;
    omega = std::max(-static_cast<double>(cmd.angular_velocity_limit),
                     std::min(omega, static_cast<double>(cmd.angular_velocity_limit)));

    // Predict current theta with omega delay (matches sim_sender delay_s logic)
    double predicted_theta = current_theta + omega * IBIS_DT;

    // Convert global polar velocity to robot-local Cartesian
    double velocity_theta = cmd.polar_velocity_theta - predicted_theta;
    double target_vx = cmd.polar_velocity_r * std::cos(velocity_theta);
    double target_vy = cmd.polar_velocity_r * std::sin(velocity_theta);

    double current_speed = std::hypot(state.prev_vx, state.prev_vy);
    double target_speed  = std::hypot(target_vx, target_vy);
    double acc_limit = calculateAccelerationLimit(current_speed, target_speed);

    // Use packet acceleration_limit if provided and tighter
    if (cmd.acceleration_limit > 0.0f && cmd.acceleration_limit < static_cast<float>(acc_limit)) {
        acc_limit = cmd.acceleration_limit;
    }

    double delta_vx = target_vx - state.prev_vx;
    double delta_vy = target_vy - state.prev_vy;
    double delta_norm = std::hypot(delta_vx, delta_vy);
    double max_delta = acc_limit * IBIS_DT;

    double out_vx, out_vy;
    if (delta_norm > max_delta && delta_norm > 1e-9) {
        out_vx = state.prev_vx + (delta_vx / delta_norm) * max_delta;
        out_vy = state.prev_vy + (delta_vy / delta_norm) * max_delta;
    } else {
        out_vx = target_vx;
        out_vy = target_vy;
    }

    double out_speed = std::hypot(out_vx, out_vy);
    if (cmd.linear_velocity_limit > 0.0f && out_speed > cmd.linear_velocity_limit) {
        out_vx = out_vx / out_speed * cmd.linear_velocity_limit;
        out_vy = out_vy / out_speed * cmd.linear_velocity_limit;
    }

    robot->setSpeed(static_cast<dReal>(out_vx), static_cast<dReal>(out_vy), static_cast<dReal>(omega));

    state.prev_vx = out_vx;
    state.prev_vy = out_vy;

    if (cmd.kick_power > 0.001f) {
        double kick_speed = IBIS_MAX_KICK_SPEED * cmd.kick_power;
        if (cmd.enable_chip) {
            robot->kicker->kick(
                static_cast<dReal>(kick_speed * 0.5 * std::cos(IBIS_CHIP_ANGLE_RAD)),
                static_cast<dReal>(kick_speed * 0.5 * std::sin(IBIS_CHIP_ANGLE_RAD)));
        } else {
            robot->kicker->kick(static_cast<dReal>(kick_speed), 0.0);
        }
    }

    robot->kicker->setRoller(cmd.dribble_power > 0.001f ? 1 : 0);
}

void IbisCommandReceiver::processPacket(const QByteArray& data, Robot** robots, int robotCount)
{
    if (data.size() != IBIS_PACKET_SIZE) {
        return;
    }

    const uint8_t* buf = reinterpret_cast<const uint8_t*>(data.constData());

    for (int slot = 0; slot < IBIS_ROBOT_SLOTS; ++slot) {
        int offset = slot * IBIS_SLOT_SIZE;
        uint8_t robot_id = buf[offset];

        if (robot_id >= static_cast<uint8_t>(robotCount)) {
            continue;
        }

        const uint8_t* cmd_data = buf + offset + 1;

        // Skip duplicate packets (check_counter unchanged)
        if (cmd_data[CHECK_COUNTER] == robot_states_[robot_id].last_check_counter) {
            continue;
        }

        IbisCommand cmd = deserialize(cmd_data);

        // Auto-detect team: find robot whose position matches vision_global_pos in the packet.
        // crane's vision_global_pos is in metres using the same coordinate system as grSim.
        Robot* target = nullptr;
        constexpr double POSITION_MATCH_THRESHOLD = 0.5;  // metres
        for (int t = 0; t < TEAM_COUNT; ++t) {
            int idx = (t == 0) ? robot_id : (robotCount + robot_id);
            if (idx >= MAX_ROBOT_COUNT * 2 || robots[idx] == nullptr) {
                continue;
            }
            Robot* r = robots[idx];
            dReal rx, ry;
            r->getXY(rx, ry);
            double dx = static_cast<double>(rx) - static_cast<double>(cmd.vision_global_pos[0]);
            double dy = static_cast<double>(ry) - static_cast<double>(cmd.vision_global_pos[1]);
            if (std::hypot(dx, dy) < POSITION_MATCH_THRESHOLD) {
                target = r;
                break;
            }
        }
        if (!target) {
            continue;
        }

        robot_states_[robot_id].last_check_counter = cmd.check_counter;

        applyCommand(robot_id, cmd, target);
    }
}

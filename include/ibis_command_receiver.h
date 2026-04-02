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

#ifndef IBIS_COMMAND_RECEIVER_H
#define IBIS_COMMAND_RECEIVER_H

#include <cmath>
#include <stdint.h>
#include "robot.h"
#include "configwidget.h"
#include "config.h"

// ibis binary protocol constants
// Packet layout: 11 slots x (1 byte robot_id + 64 bytes command) = 715 bytes
constexpr int IBIS_ROBOT_SLOTS = 11;
constexpr int IBIS_CMD_SIZE = 64;
constexpr int IBIS_SLOT_SIZE = IBIS_CMD_SIZE + 1;
constexpr int IBIS_PACKET_SIZE = IBIS_SLOT_SIZE * IBIS_ROBOT_SLOTS;
constexpr int IBIS_DEFAULT_PORT = 12345;

// Chip kick angle (matches sim_sender default)
constexpr double IBIS_CHIP_ANGLE_DEG = 30.0;
constexpr double IBIS_CHIP_ANGLE_RAD = IBIS_CHIP_ANGLE_DEG * M_PI / 180.0;
// Max kick speed in m/s (matches sim_sender)
constexpr double IBIS_MAX_KICK_SPEED = 8.0;
// Theta P gain (matches sim_sender theta_p_gain default)
constexpr double IBIS_THETA_P_GAIN = 4.0;

class IbisCommandReceiver {
public:
    explicit IbisCommandReceiver(ConfigWidget* cfg);

    // Process a 715-byte ibis broadcast packet and apply commands to robots.
    // Team is auto-detected by matching vision_global_pos from the packet to actual robot positions.
    void processPacket(const QByteArray& data, Robot** robots, int robotCount);

private:
    struct IbisCommand {
        float vision_global_pos[2];
        float target_global_theta;
        float kick_power;
        float dribble_power;
        bool enable_chip;
        bool stop_emergency;
        float acceleration_limit;
        float linear_velocity_limit;
        float angular_velocity_limit;
        float polar_velocity_r;
        float polar_velocity_theta;
        uint8_t check_counter;
    };

    struct PerRobotState {
        double prev_vx = 0.0;
        double prev_vy = 0.0;
        uint8_t last_check_counter = 0xFF;
    };

    PerRobotState robot_states_[IBIS_ROBOT_SLOTS];
    ConfigWidget* cfg_;

    static float decodeTwoByte(uint8_t high, uint8_t low, float range);
    static IbisCommand deserialize(const uint8_t* data64);
    void applyCommand(int robot_id, const IbisCommand& cmd, Robot* robot);
    double calculateAccelerationLimit(double current_speed, double target_speed) const;
};

#endif // IBIS_COMMAND_RECEIVER_H

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

#include "binary_feedback_sender.h"
#include <QHostAddress>
#include <cstring>
#include <cmath>

BinaryFeedbackSender::BinaryFeedbackSender(ConfigWidget* _cfg)
    : cfg(_cfg), enabled(false)
{
    socket = new QUdpSocket();
    memset(counters, 0, sizeof(counters));
}

BinaryFeedbackSender::~BinaryFeedbackSender()
{
    delete socket;
}

void BinaryFeedbackSender::setEnabled(bool _enabled)
{
    enabled = _enabled;
}

void BinaryFeedbackSender::buildPacket(uint8_t* buffer, int robotId, Robot* robot)
{
    // 128バイトをゼロクリア
    memset(buffer, 0, 128);

    // ヘッダー (0-1)
    buffer[0] = 0xAB;
    buffer[1] = 0xEA;

    // Robot ID (2)
    buffer[2] = static_cast<uint8_t>(robotId);

    // Counter (3)
    buffer[3] = counters[robotId];
    counters[robotId]++;

    // Yaw角 (4-7) - deg→rad変換
    dReal dir = robot->getDir();
    float yaw_rad = static_cast<float>(dir * M_PI / 180.0);
    memcpy(&buffer[4], &yaw_rad, sizeof(float));

    // 電圧 (8-11) - 固定値 24.0V
    float voltage = 24.0f;
    memcpy(&buffer[8], &voltage, sizeof(float));

    // ボール検出0-2 (12-14)
    bool ball_detected = robot->kicker->isTouchingBall();
    buffer[12] = ball_detected ? 1 : 0;
    buffer[13] = ball_detected ? 1 : 0;
    buffer[14] = ball_detected ? 1 : 0;

    // キック状態 (15)
    KickStatus kick_status = robot->kicker->isKicking();
    buffer[15] = static_cast<uint8_t>(kick_status);

    // エラー情報 (16-23) - 0 (エラーなし)
    // 既にmemsetで0初期化済み

    // モーター電流 (24-27) - 0
    // 既にmemsetで0初期化済み

    // ボール検出3 (28) - 0
    // 既にmemsetで0初期化済み

    // 温度 (29-35) - 25℃
    for (int i = 29; i <= 35; i++) {
        buffer[i] = 25;
    }

    // 角度差分 (36-39) - 0
    float angle_diff = 0.0f;
    memcpy(&buffer[36], &angle_diff, sizeof(float));

    // キャパシタ電圧 (40-43) - 200.0V
    float capacitor_voltage = 200.0f;
    memcpy(&buffer[40], &capacitor_voltage, sizeof(float));

    // オドメトリXY (44-51)
    dReal x, y;
    robot->getXY(x, y);
    float odom_x = static_cast<float>(x);
    float odom_y = static_cast<float>(y);
    memcpy(&buffer[44], &odom_x, sizeof(float));
    memcpy(&buffer[48], &odom_y, sizeof(float));

    // 速度XY (52-59)
    const dReal* linear_vel = dBodyGetLinearVel(robot->chassis->body);
    float vel_x = static_cast<float>(linear_vel[0]);
    float vel_y = static_cast<float>(linear_vel[1]);
    memcpy(&buffer[52], &vel_x, sizeof(float));
    memcpy(&buffer[56], &vel_y, sizeof(float));

    // チェックバージョン (60) - 0x01 (シミュレータ識別)
    buffer[60] = 0x01;

    // 拡張データ (61-127) - 0
    // 既にmemsetで0初期化済み
}

void BinaryFeedbackSender::sendFeedback(int robotId, Robot* robot)
{
    if (!enabled || robot == nullptr) {
        return;
    }

    uint8_t buffer[128];
    buildPacket(buffer, robotId, robot);

    // 送信先アドレスとポート
    // ポート = 50100 + robot_id
    QHostAddress addr("127.0.0.1");
    int port = 50100 + robotId;

    socket->writeDatagram(reinterpret_cast<const char*>(buffer), 128, addr, port);
}

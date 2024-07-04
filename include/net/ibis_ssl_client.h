//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    ibis_ssl_client.h
  \brief   C++ Interface: ibis_ssl_client
  \author  Stefan Zickler, 2009
  \author  Jan Segre, 2012
*/
//========================================================================
#ifndef IBIS_SSL_CLIENT_H
#define IBIS_SSL_CLIENT_H
// #include <grSim_Robotstatus.pb.h>
#include <QHostAddress>
#include <QMutex>
#include <QtNetwork>
#include <optional>
#include <QObject>
#include <QTimer>
#include <string>
#include <sstream>
#include <iostream>

#include "robot.h"
#include "robot_control.h"
#include "ring_buffer.h"

#include "ibis_robot_packet.hpp"

// #include "grSim_Packet.pb.h"

class QUdpSocket;
class QHostAddress;
class QNetworkInterface;

class RobotClient : public QObject{
  Q_OBJECT
public:
  RobotClient(QObject* parent = nullptr) : QObject(parent)
  {
  }

  ~RobotClient()
  {
    if(has_setup)
    {
      _socket->close();
      delete _socket;
    }
  }

  void setup(uint8_t id) {
    _socket = new QUdpSocket(this);
    _port = 50100 + id;
    _net_address = QHostAddress::LocalHost;
    if(not _socket->bind(QHostAddress::LocalHost, _port))
    {
      std::cout << "Failed to bind to port " << _port << std::endl;
    }else
    {
      connect(_socket, SIGNAL(readyRead()), this, SLOT(receiveAndProcess()));
    }
    robot_loop_timer = new QTimer(this);
    connect(robot_loop_timer, SIGNAL(timeout()), this, SLOT(robot_loop_timer_callback()));
    // 2ms / 500Hz
    robot_loop_timer->setInterval(2);
    robot_loop_timer->start();
    has_setup = true;
  }

  std::optional<RobotCommandV2> receive() {
    if (_socket->hasPendingDatagrams()) {
      QByteArray packet_data;
      packet_data.resize(_socket->pendingDatagramSize());
      _socket->readDatagram(packet_data.data(), packet_data.size());

      RobotCommandSerializedV2 raw;
      for (int i = 0; i < packet_data.size(); ++i) {
        if (i < 64) {
          raw.data[i] = packet_data[i];
        }
      }
      return RobotCommandSerializedV2_deserialize(&raw);
    } else {
      return std::nullopt;
    }
  }

  void setRobot(Robot *robot)
  {
    _robot = robot;
  }

private slots:
  void receiveAndProcess(){
//      std::cout << "receiveAndProcess" << std::endl;

      const double MAX_KICK_SPEED = 8.0; // m/s
      while(auto packet = receive())
      {
        if(_robot == nullptr)
        {
//            std::cout << "Robot not set" << std::endl;
            return;
        }

        {
            orion.connection.vision_update_cycle_cnt++;
            if (orion.connection.vision_update_cycle_cnt > 1000) {
                orion.connection.vision_update_cycle_cnt = 0;
            }
        }

        {  // System update
        }

        {  // AI command update
          orion.ai_cmd = packet.value();
        }



          dReal x,y;
          _robot->getXY(x, y);
          const double last_dt = 0.01;
          double kick_speed = orion.ai_cmd.kick_power * MAX_KICK_SPEED;
          _robot->kicker->kick(kick_speed,
                               orion.ai_cmd.enable_chip ? kick_speed : 0.0);
          _robot->kicker->setRoller(orion.ai_cmd.dribble_power > 0.0);
      }
  }

  void robot_loop_timer_callback() {
    if(_robot == nullptr)
    {
//      std::cout << "Robot not set" << std::endl;
      return;
    }

    {  // IMU update
      orion.imu.pre_yaw_angle = orion.imu.yaw_angle;
      orion.imu.pre_yaw_angle_rad = orion.imu.yaw_angle_rad;
      orion.imu.yaw_angle = _robot->getDir();
      orion.imu.yaw_angle_rad = orion.imu.yaw_angle * M_PI / 180.0;
      orion.imu.yaw_angle_diff_integral += orion.imu.yaw_angle - orion.imu.pre_yaw_angle;
    }

    {  // Odom update
      // https://github.com/ibis-ssl/G474_Orion_main/blob/main/Core/Src/odom.c#L32
      for(int i = 0; i < 4; i++) {
        auto & wheel = _robot->wheels[i];
        if (isnan(orion.motor.enc_angle[i])) {
          orion.motor.enc_angle[i] = 0;
        }
        orion.motor.pre_enc_angle[i] = orion.motor.enc_angle[i];
        orion.motor.enc_angle[i] += static_cast<float>(dJointGetAMotorAngleRate(wheel->motor, 0)) / MAIN_LOOP_CYCLE;
        orion.motor.angle_diff[i] = orion.motor.enc_angle[i] - orion.motor.pre_enc_angle[i];
      }

      const double omni_diameter = [&](){
          dReal radius, length;
          dGeomCylinderGetParams(_robot->wheels[0]->cyl->geom, &radius, &length);
          return radius * 2.0;
      }();
      orion.omni.travel_distance[0] = orion.motor.angle_diff[1] * omni_diameter;
      orion.omni.travel_distance[1] = orion.motor.angle_diff[2] * omni_diameter;

      // right front & left front
      orion.omni.odom_raw[0] += orion.omni.travel_distance[0] * cos(orion.imu.yaw_angle_rad) + orion.omni.travel_distance[1] * sin(orion.imu.yaw_angle_rad);
      orion.omni.odom_raw[1] += orion.omni.travel_distance[0] * sin(orion.imu.yaw_angle_rad) - orion.omni.travel_distance[1] * cos(orion.imu.yaw_angle_rad);

      orion.omni.pre_odom[0] = orion.omni.odom[0];
      orion.omni.pre_odom[1] = orion.omni.odom[1];

      orion.omni.odom[0] = ((orion.omni.odom_raw[0] * cos(M_PI * 3 / 4) - orion.omni.odom_raw[1] * sin(M_PI * 3 / 4)) / 2) + (0.107 * cos(orion.imu.yaw_angle_rad) - 0.107);
      orion.omni.odom[1] = ((orion.omni.odom_raw[0] * sin(M_PI * 3 / 4) + orion.omni.odom_raw[1] * cos(M_PI * 3 / 4)) / 2) + (0.107 * sin(orion.imu.yaw_angle_rad));

      //   omni->odom_speed[0] = (omni->odom[0] - omni->pre_odom[0]) * MAIN_LOOP_CYCLE;
      //  omni->odom_speed[1] = (omni->odom[1] - omni->pre_odom[1]) * MAIN_LOOP_CYCLE;
      orion.omni.odom_speed[0] = (orion.omni.odom[0] - orion.omni.pre_odom[0]) * MAIN_LOOP_CYCLE;
      orion.omni.odom_speed[1] = (orion.omni.odom[1] - orion.omni.pre_odom[1]) * MAIN_LOOP_CYCLE;

      // omni->local_odom_speed[0] = omni->odom_speed[0] * cos(-imu->yaw_angle_rad) - omni->odom_speed[1] * sin(-imu->yaw_angle_rad);
      // omni->local_odom_speed[1] = omni->odom_speed[0] * sin(-imu->yaw_angle_rad) + omni->odom_speed[1] * cos(-imu->yaw_angle_rad);
      orion.omni.local_odom_speed[0] = orion.omni.odom_speed[0] * cos(-orion.imu.yaw_angle_rad) - orion.omni.odom_speed[1] * sin(-orion.imu.yaw_angle_rad);
      orion.omni.local_odom_speed[1] = orion.omni.odom_speed[0] * sin(-orion.imu.yaw_angle_rad) + orion.omni.odom_speed[1] * cos(-orion.imu.yaw_angle_rad);

      // for (int i = 0; i < 2; i++) {
      //    enqueue(omni->local_speed_log[i], omni->local_odom_speed[i]);
      //    omni->local_odom_speed_mvf[i] = sumNewestN(omni->local_speed_log[i], SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE) / SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE;
      //  }
      for (int i = 0; i <2 ; ++i) {
        enqueue(orion.omni.local_speed_log[i], orion.omni.local_odom_speed[i]);
        orion.omni.local_odom_speed_mvf[i] = sumNewestN(orion.omni.local_speed_log[i], SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE) / SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE;
      }

      // local座標系で入れているodom speedを,global系に修正する
      // vision座標だけ更新されているが、vision_update_cycle_cntが0になっていない場合に、1cycleだけpositionが飛ぶ
      float latency_cycle = orion.ai_cmd.laytency_time_ms / (1000 / MAIN_LOOP_CYCLE);
      for (int i = 0; i < 2; i++) {
        enqueue(orion.integ.odom_log[i], orion.omni.odom_speed[i]);
        // メモ：connection.vision_update_cycle_cntは更新できていない
        // 実際の座標を取得できるのでこの処理はスキップ
        // orion.integ.global_odom_vision_diff[i] = sumNewestN(orion.integ.odom_log[i], latency_cycle + orion.connection.vision_update_cycle_cnt) / MAIN_LOOP_CYCLE;
        // orion.integ.vision_based_position[i] = orion.ai_cmd.global_robot_position[i] + orion.integ.global_odom_vision_diff[i];
      }
      orion.integ.position_diff[0] = orion.ai_cmd.mode_args.position.target_global_x - orion.integ.vision_based_position[0];
      orion.integ.position_diff[1] = orion.ai_cmd.mode_args.position.target_global_y - orion.integ.vision_based_position[1];


      float target_diff[2], move_diff[2];
      target_diff[0] = orion.ai_cmd.vision_global_x - orion.ai_cmd.mode_args.position.target_global_x;  // Visionが更新された時点での現在地とtargetの距離
      move_diff[0] = orion.ai_cmd.vision_global_x - orion.integ.vision_based_position[0];      // Visionとtargetが更新されてからの移動量
      target_diff[1] = orion.ai_cmd.vision_global_y - orion.ai_cmd.mode_args.position.target_global_y;  // Visionが更新された時点での現在地とtargetの距離
      move_diff[1] = orion.ai_cmd.vision_global_y - orion.integ.vision_based_position[1];      // Visionとtargetが更新されてからの移動量


      orion.integ.target_dist_diff = sqrt(pow(target_diff[0], 2) + pow(target_diff[1], 2));
      orion.integ.move_dist = sqrt(pow(orion.integ.position_diff[0], 2) + pow(orion.integ.position_diff[1], 2));
    }

    local_feedback(&orion.integ, &orion.imu, &orion.sys, &orion.target, &orion.ai_cmd, &orion.omni);
    accel_control(&orion.acc_vel, &orion.output, &orion.target, &orion.omni);
    speed_control(&orion.acc_vel, &orion.output, &orion.target, &orion.imu, &orion.omni);
    output_limit(&orion.output, &orion.debug);
    theta_control(orion.ai_cmd.target_global_theta, &orion.acc_vel, &orion.output, &orion.imu);
    auto reduce_abs = [](float val, float reduce_val){
        if(val > 0){
            val -= reduce_val;
            if(val < 0){
                val = 0;
            }
        }else{
            val += reduce_val;
            if(val > 0){
                val = 0;
            }
        }
        return val;
    };
    for(int i = 0; i < 4; i++){
        _robot->setSpeed(i, orion.output.motor_voltage[i]);
    }
//    _robot->setSpeed(reduce_abs(orion.output.velocity[0], 0.4), reduce_abs(orion.output.velocity[1], 0.4), orion.output.omega);
//    _robot->setSpeed(0.1, 0, 0);
    if(_port == 50100) {
      std::cout << orion.output.motor_voltage[0] << " " << orion.output.motor_voltage[1] << " " << orion.output.motor_voltage[2] << " " << orion.output.motor_voltage[3] << std::endl;
    }
  }

public:
  QUdpSocket *_socket;
  QMutex mutex;
  QTimer *robot_loop_timer;
  quint16 _port;
  QHostAddress _net_address;
  Robot * _robot = nullptr;
  bool has_setup = false;

  struct OrionInternal{
      OrionInternal(){
        integ.odom_log[0] = initRingBuffer(SPEED_LOG_BUF_SIZE);
        integ.odom_log[1] = initRingBuffer(SPEED_LOG_BUF_SIZE);
        omni.local_speed_log[0] = initRingBuffer(SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE);
        omni.local_speed_log[1] = initRingBuffer(SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE);
      }
      integration_control_t integ;
        imu_t imu;
        system_t sys;
        target_t target;
        RobotCommandV2 ai_cmd;
        accel_vector_t acc_vel;
        output_t output;
        omni_t omni;
        debug_t debug;
        motor_t motor;
        connection_t connection;
  } orion;

  //  QNetworkInterface *_net_interface;
};

class IbisRobotCommunicator {
public:
  IbisRobotCommunicator(bool is_yellow = false) : is_yellow(is_yellow) {
    for (int i = 0; i < 20; ++i) {
      _clients[i].setup(i);
    }
  }

  bool isYellow() const {
    return is_yellow;
  }

  std::optional<RobotCommandV2> receive(uint8_t id) {
    return _clients[id].receive();
  }

  std::array<RobotClient, 20> _clients;

  bool is_yellow = true;
};

#endif

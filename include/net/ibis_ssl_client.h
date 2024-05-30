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
#include <string>
#include <sstream>
#include <iostream>

#include "robot.h"
//#include "ibis_orion.h"
#include "ibis/management.h"
#include "ibis/robot_control.h"

#include "ibis_robot_packet.hpp"

// #include "grSim_Packet.pb.h"

class QUdpSocket;
class QHostAddress;
class QNetworkInterface;

class PIDController {
public:
  PIDController() = default;

  void setGain(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
    error_prev = 0.0f;
  }

  double update(double error, double dt) {
    double p = kp * error;
    double i = ki * (error + error_prev) * dt / 2.0f;
    double d = kd * (error - error_prev) / dt;
    error_prev = error;
    return p + i + d;
  }

private:
  double kp, ki, kd;

  double error_prev;
};

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
    theta_controller.setGain(3.0, 0.0, 0.0);
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
    has_setup = true;
  }

  std::optional<crane::RobotCommand> receive() {
    if (_socket->hasPendingDatagrams()) {
      QByteArray packet_data;
      packet_data.resize(_socket->pendingDatagramSize());
      _socket->readDatagram(packet_data.data(), packet_data.size());

      crane::RobotCommandSerialized raw;
      for (int i = 0; i < packet_data.size(); ++i) {
        if (i < static_cast<int>(crane::RobotCommandSerialized::Address::SIZE)) {
          raw.data[i] = packet_data[i];
        }
      }
      return raw;
    } else {
      return std::nullopt;
    }
  }

  double normalizeAngle(double angle_rad)
  {
      while (angle_rad > M_PI) {
          angle_rad -= 2.0f * M_PI;
      }
      while (angle_rad < -M_PI) {
          angle_rad += 2.0f * M_PI;
      }
      return angle_rad;
  }

  double getAngleDiff(double angle_rad1, double angle_rad2)
  {
      angle_rad1 = normalizeAngle(angle_rad1);
      angle_rad2 = normalizeAngle(angle_rad2);
      if (abs(angle_rad1 - angle_rad2) > M_PI) {
          if (angle_rad1 - angle_rad2 > 0) {
              return angle_rad1 - angle_rad2 - 2.0f * M_PI;
          } else {
              return angle_rad1 - angle_rad2 + 2.0f * M_PI;
          }
      } else {
          return angle_rad1 - angle_rad2;
      }
  }

  double getOmega(double current_theta, double target_theta, double dt) {
    return -theta_controller.update(getAngleDiff(current_theta, target_theta), dt);
  }

  void setRobot(Robot *robot)
  {
    _robot = robot;
  }

private slots:
  void receiveAndProcess();

public:
  PIDController theta_controller;

  QUdpSocket *_socket;
  QMutex mutex;
  quint16 _port;
  QHostAddress _net_address;
  Robot * _robot = nullptr;
  bool has_setup = false;

  struct OrionInternal{
      integration_control_t integ;
        imu_t imu;
        system_t sys;
        target_t target;
        ai_cmd_t ai_cmd;
        accel_vector_t acc_vel;
        output_t output;
        omni_t omni;
        debug_t debug;
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

  std::optional<crane::RobotCommand> receive(uint8_t id) {
    return _clients[id].receive();
  }

  double getOmega(double current_theta, double target_theta, double dt,
                  uint8_t id) {
    return _clients[id].getOmega(current_theta, target_theta, dt);
  }

  std::array<RobotClient, 20> _clients;

  bool is_yellow;
};

#endif

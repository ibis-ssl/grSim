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
  \file    robocup_ssl_client.h
  \brief   C++ Interface: robocup_ssl_client
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

#include "robot.h"

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
    theta_controller.setGain(0.5, 0.0, 0.0);
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

  double getOmega(double current_theta, double target_theta, double dt) {
    return theta_controller.update(target_theta - current_theta, dt);
  }

  void setRobot(Robot *robot)
  {
    _robot = robot;
  }

private slots:
  void receiveAndProcess()
  {
    const double MAX_KICK_SPEED = 8.0; // m/s
    while(auto packet = receive())
    {
      if(_port == 50100)
      {
        std::stringstream ss;
        ss << "vx: " << packet->VEL_LOCAL_SURGE << " vy: " << packet->VEL_LOCAL_SWAY << " theta: " << packet->TARGET_GLOBAL_THETA;
        std::cout << ss.str() << std::endl;
      }
      const double last_dt = 0.01;
      double omega = getOmega(
          _robot->getDir(), packet->TARGET_GLOBAL_THETA, last_dt);
      _robot->setSpeed(packet->VEL_LOCAL_SURGE, packet->VEL_LOCAL_SWAY,
                           omega);
      double kick_speed = packet->KICK_POWER * MAX_KICK_SPEED;
      _robot->kicker->kick(kick_speed,
                               packet->CHIP_ENABLE ? kick_speed : 0.0);
      // TODO: use dribble power as double value
      _robot->kicker->setRoller(packet->DRIBBLE_POWER > 0.0);

      // Robots_Status robotsPacket;e7
      // bool updateRobotStatus = false;
      //   int id = _port - 50100;
      //   bool isInfrared = _robot->kicker->isTouchingBall();
      //   KickStatus kicking = _robot->kicker->isKicking();
      //   if (isInfrared != lastInfraredState[team][i] ||
      //       kicking != lastKickState[team][i]) {
      //     updateRobotStatus = true;
      //     addRobotStatus(robotsPacket, i, isInfrared, kicking);
      //     // lastInfraredState[team][i] = isInfrared;
      //     // lastKickState[team][i] = kicking;
      //       }
      // }
      //
      // int size = robotsPacket.ByteSize();
      // QByteArray buffer(size, 0);
      // robotsPacket.SerializeToArray(buffer.data(), buffer.size());
      // yellowStatusSocket->writeDatagram(buffer.data(), buffer.size(), sender,
      //                                 cfg->YellowStatusSendPort());
    }
  }

public:
  PIDController theta_controller;

  QUdpSocket *_socket;
  QMutex mutex;
  quint16 _port;
  QHostAddress _net_address;
  Robot * _robot = nullptr;
  bool has_setup = false;

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
    return -_clients[id].getOmega(current_theta, target_theta, dt);
  }

  std::array<RobotClient, 20> _clients;

  bool is_yellow;
};

#endif

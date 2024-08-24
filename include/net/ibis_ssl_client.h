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
    if(_socket->bind(QHostAddress::LocalHost, _port) != 0)
    {
      std::cout << "Failed to bind to port " << _port << std::endl;
    }else
    {
      std::cout << "bind to port " << _port << std::endl;
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

      const double MAX_KICK_SPEED = 8.0; // m/s
      while(auto packet = receive())
      {
        if(_robot == nullptr)
        {
//            std::cout << "Robot not set" << std::endl;
            return;
        }

        {  // AI command update
          ai_cmd = packet.value();
        }

          double kick_speed = ai_cmd.kick_power * MAX_KICK_SPEED;
          _robot->kicker->kick(kick_speed,
                               ai_cmd.enable_chip ? kick_speed : 0.0);
          _robot->kicker->setRoller(ai_cmd.dribble_power > 0.0);

          // TODO: implement for SimpleVelocity
      }
  }

  void robot_loop_timer_callback() {
    // TODO: theta control
  }

public:
  QUdpSocket *_socket;
  QMutex mutex;
  QTimer *robot_loop_timer;
  quint16 _port;
  QHostAddress _net_address;
  Robot * _robot = nullptr;
  bool has_setup = false;

  RobotCommandV2 ai_cmd;
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

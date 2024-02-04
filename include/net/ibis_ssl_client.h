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
#include <QHostAddress>
#include <QMutex>
#include <QtNetwork>
#include <optional>
#include <string>

#include "ibis_robot_packet.hpp"

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

struct RobotClient {
  void setup(uint8_t id) {
    _socket = new QUdpSocket();
    _port = 12345;
    _net_address = new QHostAddress(
        QString::fromStdString("192.168.20." + std::to_string(100 + id)));
    _socket->bind(_port);
//    _socket->bind(_port);
    //    _net_interface =
    //        new
    //        QNetworkInterface(QNetworkInterface::interfaceFromName("eth0"));
  }

  std::optional<crane::RobotCommand> receive() {
    if (_socket->hasPendingDatagrams()) {
      crane::RobotCommandSerialized raw;
      auto packet_data = _socket->readAll();
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

  PIDController theta_controller;

  QUdpSocket *_socket;
  QMutex mutex;
  quint16 _port;
  QHostAddress *_net_address;
  //  QNetworkInterface *_net_interface;
};

class IbisRobotCommunicator {
public:
  IbisRobotCommunicator() {
    for (int i = 0; i < 20; ++i) {
      _clients[i].setup(i);
    }
  }

  std::optional<crane::RobotCommand> receive(uint8_t id) {
    return _clients[id].receive();
  }

  double getOmega(double current_theta, double target_theta, double dt,
                  uint8_t id) {
    return _clients[id].getOmega(current_theta, target_theta, dt);
  }

  std::array<RobotClient, 20> _clients;
};

#endif

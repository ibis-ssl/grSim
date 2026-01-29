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

#ifndef BINARY_FEEDBACK_SENDER_H
#define BINARY_FEEDBACK_SENDER_H

#include <QUdpSocket>
#include <stdint.h>
#include "robot.h"
#include "configwidget.h"
#include "config.h"

class BinaryFeedbackSender {
public:
    BinaryFeedbackSender(ConfigWidget* cfg);
    ~BinaryFeedbackSender();

    void sendFeedback(int robotId, Robot* robot);
    void setEnabled(bool enabled);
    bool isEnabled() const { return enabled; }

private:
    void buildPacket(uint8_t* buffer, int robotId, Robot* robot);

    QUdpSocket* socket;
    uint8_t counters[MAX_ROBOT_COUNT * 2];  // Blue and Yellow teams
    ConfigWidget* cfg;
    bool enabled;
};

#endif // BINARY_FEEDBACK_SENDER_H

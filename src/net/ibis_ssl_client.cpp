#include "ibis_ssl_client.h"
#include "robot_control.h"

void RobotClient::receiveAndProcess() {

    const double MAX_KICK_SPEED = 8.0; // m/s
    while(auto packet = receive())
    {
        if(_robot == nullptr)
        {
            std::cout << "Robot not set" << std::endl;
            return;
        }
        if(_port == 50100)
        {
            std::stringstream ss;
            ss << "vx: " << packet->VEL_LOCAL_SURGE << " vy: " << packet->VEL_LOCAL_SWAY << " theta: " << packet->TARGET_GLOBAL_THETA << " actual theta: " << _robot->getDir();
            std::cout << ss.str() << std::endl;
        }

        orion.ai_cmd.local_target_speed[0] = packet->VEL_LOCAL_SURGE;
        orion.ai_cmd.local_target_speed[1] = packet->VEL_LOCAL_SWAY;

        orion.ai_cmd.local_target_speed_scalar = sqrt(pow(packet->VEL_LOCAL_SURGE, 2.) + pow(packet->VEL_LOCAL_SWAY, 2.));
        orion.ai_cmd.global_vision_theta = packet->VISION_GLOBAL_THETA;
        orion.ai_cmd.target_theta = packet->TARGET_GLOBAL_THETA;
        orion.ai_cmd.chip_en = packet->CHIP_ENABLE;
        orion.ai_cmd.kick_power = packet->KICK_POWER;
        orion.ai_cmd.dribble_power = packet->DRIBBLE_POWER;

        auto raw_packet = static_cast<crane::RobotCommandSerialized>(*packet);
        orion.ai_cmd.allow_local_flags = raw_packet.data[static_cast<int>(crane::RobotCommandSerialized::Address::LOCAL_FLAGS)];

        orion.integ.pre_global_target_position[0] = packet->TARGET_GLOBAL_X;
        orion.integ.pre_global_target_position[1] = packet->TARGET_GLOBAL_Y;

        orion.ai_cmd.global_ball_position[0] = packet->BALL_GLOBAL_X;
        orion.ai_cmd.global_ball_position[1] = packet->BALL_GLOBAL_Y;
        orion.ai_cmd.global_robot_position[0] = packet->VISION_GLOBAL_X;
        orion.ai_cmd.global_robot_position[1] = packet->VISION_GLOBAL_Y;
        orion.ai_cmd.global_target_position[0] = packet->TARGET_GLOBAL_X;
        orion.ai_cmd.global_target_position[1] = packet->TARGET_GLOBAL_Y;

        orion.ai_cmd.vision_lost_flag = ! packet->IS_ID_VISIBLE;
//        orion.ai_cmd.local_vision_en_flag = packet->LOCAL_VISION_EN;
        orion.ai_cmd.local_vision_en_flag = false;
        orion.ai_cmd.keeper_mode_en_flag = packet->LOCAL_KEEPER_MODE_ENABLE;
        orion.ai_cmd.stop_request_flag = packet->STOP_FLAG;
        orion.ai_cmd.dribbler_up_flag = packet->IS_DRIBBLER_UP;
        const double last_dt = 0.01;
        double omega = getOmega(
                _robot->getDir() * M_PI / 180.0, packet->TARGET_GLOBAL_THETA, last_dt);
        _robot->setSpeed(packet->VEL_LOCAL_SURGE, packet->VEL_LOCAL_SWAY,
                         omega);
        double kick_speed = packet->KICK_POWER * MAX_KICK_SPEED;
        _robot->kicker->kick(kick_speed,
                             packet->CHIP_ENABLE ? kick_speed : 0.0);
        // TODO: これらは本来別のメインループで回さないといけない（実機では500Hz）
        local_feedback(&orion.integ, &orion.imu, &orion.sys, &orion.target, &orion.ai_cmd, &orion.omni);
        accel_control(&orion.acc_vel, &orion.output, &orion.target, &orion.omni);
        speed_control(&orion.acc_vel, &orion.output, &orion.target, &orion.imu, &orion.omni);
        output_limit(&orion.output, &orion.debug);
        _robot->kicker->setRoller(packet->DRIBBLE_POWER > 0.0);
    }
}

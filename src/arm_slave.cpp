#include <unitree/robot/channel/channel_subscriber.hpp>

#include "ServoAngleData.hpp"

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

#include <iostream>
#include <thread>
#include <chrono>

#define SubServoAngle_Topic "servo_angle_data"
#define SERVO_PORT_NAME "/dev/ttyS4"

using namespace unitree::robot;
using namespace unitree::common;
using namespace fsuservo;

FSUS_Protocol protocol(SERVO_PORT_NAME, FSUS_DEFAULT_BAUDRATE);
FSUS_Servo servo[7] = {
            FSUS_Servo(0,&protocol),
            FSUS_Servo(1,&protocol),
            FSUS_Servo(2,&protocol),
            FSUS_Servo(3,&protocol),
            FSUS_Servo(4,&protocol),
            FSUS_Servo(5,&protocol),
            FSUS_Servo(6,&protocol),
        };

void subServoAngle_callback(const void* msg)
{
    const ServoAngleData* pm = (const ServoAngleData*) msg;
    float maxalpha = 286.4789 / 4;
    float maxomega = 28.64789 * 4;
    float maxangle[7] = { 135,  90,  90,  135,  90,  135,  50};
    float minangle[7] = {-135, -90, -90, -135, -90, -135, -20};

    for(int i = 0; i < 7; i++)
    {
        float angle = pm->angles()[i];
        float curAngle = servo[i].queryRawAngle();
        int delay_ms = pm->delays_ms()[i] == 0 ? 1 : pm->delays_ms()[i];

        if((curAngle <= 180) && (curAngle >= -180))
        {
            angle = angle > maxangle[i] ? maxangle[i] : angle;
            angle = angle < minangle[i] ? minangle[i] : angle;
            float omega = abs(angle - curAngle) / delay_ms * 1000;
            omega = omega > maxomega ? maxomega : omega;
            float alpha = omega / delay_ms * 1000;
            alpha = alpha > maxalpha ? maxalpha : alpha;

            int tardelay_ms = (int)(sqrt(abs(angle - curAngle) * 2 / alpha) * 1000);
            int testdelay_ms = (int)(((sqrt(omega * omega + abs(angle - curAngle) * 2 * alpha) - omega) / alpha) * 1800);
            testdelay_ms = testdelay_ms > 0 ? testdelay_ms : 0;

            servo[i].setRawAngle(angle, testdelay_ms);
            std::cout << "id:" << i << ", curangle:" << curAngle << ", tarangle:" << angle << std::endl;
        }
        else
        {
            servo[i].setDamping(500);
        }
    }

}

int main()
{
    ChannelFactory::Instance()->Init(0);

    ChannelSubscriber<ServoAngleData> subServoAngle_subscriber(SubServoAngle_Topic);
    subServoAngle_subscriber.InitChannel(subServoAngle_callback, 10);

    while (true)
    {
        sleep(10);
    }

    return 0;
}

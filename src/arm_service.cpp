#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

#include "ServoAngleData.hpp"
#include "ServoData.hpp"
#include "ServoPower.hpp"

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

#include <iostream>
#include <thread>
#include <chrono>

#define PubServoAngle_Topic "servo_angle_data"
#define PubServo_Topic "servo_data"
#define SubServoAngle_Topic "servo_angle_command"
#define DeactivateServos_Topic "deactivate_servos"
#define SERVO_PORT_NAME "/dev/ttyS4"

using namespace unitree::robot;
using namespace unitree::common;
using namespace fsuservo;

std::mutex myMutex;

ChannelPublisherPtr<ServoAngleData> pubServoAngle_publisher;
ChannelPublisherPtr<ServoData> pubServo_publisher;

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

class Timer_ {
    public:
        Timer_(std::function<void()> callback, int interval) :
            callback_(callback),
            interval_(interval),
            running_(false) {}

        void start() {
            running_ = true;
            timer_thread_ = std::thread([this]() {
                while (running_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(interval_));
                    if (callback_ != nullptr) {
                        callback_();
                    }
                }
            });
        }

        void stop() {
            running_ = false;
            if (timer_thread_.joinable()) {
                timer_thread_.join();
            }
        }

    private:
        std::function<void()> callback_;
        int interval_;
        bool running_;
        std::thread timer_thread_;
};

void subServoAngle_callback(const void* msg)
{
    const ServoAngleData* pm = (const ServoAngleData*) msg;
    float maxalpha = 286.4789 / 4;
    float maxomega = 28.64789 * 4;
    float maxangle[7] = { 135,  90,  90,  135,  90,  135,  50};
    float minangle[7] = {-135, -90, -90, -135, -90, -135, -20};

    std::cout << "Sending Servo Commands" << std::endl;

    myMutex.lock();
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
            // std::cout << "id:" << i << ", curangle:" << curAngle << ", tarangle:" << angle << std::endl;
        }
        else
        {
            servo[i].setDamping(500);
        }
    }
    myMutex.unlock();

}

void pubServoAngle_callback()
{
    ServoAngleData servoInfo_msg{};

    // std::string msg = "Servo Info: ";

    for (int i = 0 ; i < 7; i++){
        myMutex.lock();
        servoInfo_msg.angles()[i] = servo[i].queryRawAngle();
        myMutex.unlock();
        servoInfo_msg.delays_ms()[i] = 10;
        // msg += std::to_string(servoInfo_msg.angles()[i]) + ", ";
    }

    // std::cout << msg << std::endl;

    pubServoAngle_publisher->Write(servoInfo_msg, 0);
}

void pubServo_callback()
{
    ServoData servoInfo_msg{};

    for (int i = 0 ; i < 7; i++){
        servoInfo_msg.temperature()[i] = servo[i].queryTemperature();
        servoInfo_msg.voltage()[i] = servo[i].queryVoltage();
        servoInfo_msg.current()[i] = servo[i].queryCurrent();
        servoInfo_msg.power()[i] = servo[i].queryPower();
    }

    pubServo_publisher->Write(servoInfo_msg, 0);
}

void DeactivateServos_callback(const void* msg)
{
    const ServoPower* pm = (const ServoPower*) msg;

    std::cout << "Deactivating Servos" << std::endl;

    myMutex.lock();
    for(int i = 0; i < 7; i++)
    {
        servo[i].setDamping(pm->power()[i]);
    }
    myMutex.unlock();
}


int main()
{
    ChannelFactory::Instance()->Init(0);

    pubServoAngle_publisher.reset(new ChannelPublisher<ServoAngleData>(PubServoAngle_Topic));
    pubServoAngle_publisher->InitChannel();
    pubServo_publisher.reset(new ChannelPublisher<ServoData>(PubServo_Topic));
    pubServo_publisher->InitChannel();

    ChannelSubscriber<ServoAngleData> subServoAngle_subscriber(SubServoAngle_Topic);
    subServoAngle_subscriber.InitChannel(subServoAngle_callback, 10);
    ChannelSubscriber<ServoPower> DeactivateServos_subscriber(DeactivateServos_Topic);
    DeactivateServos_subscriber.InitChannel(DeactivateServos_callback, 10);

    Timer_ timer(pubServoAngle_callback, 10);
    timer.start();
    Timer_ timer2(pubServo_callback, 100);
    timer2.start();

    while (true)
    {
        sleep(10);
    }

    timer.stop();
    timer2.stop();

    return 0;
}

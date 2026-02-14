#ifndef CYBERGEAR_HPP
#define CYBERGEAR_HPP

#include <vector>
#include <cstdint>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <unordered_map>
#include <string>

const uint8_t CMD_POSITION = 1;
const uint8_t CMD_RESPONSE = 2;
const uint8_t CMD_ENABLE = 3;
const uint8_t CMD_STOP = 4;
const uint8_t CMD_SET_MECH_POSITION_ZERO = 6;
const uint8_t CMD_SET_CAN_ID = 7;
const uint8_t CMD_GET_STATUS = 15;
const uint8_t CMD_RAM_READ = 17;
const uint8_t CMD_RAM_WRITE = 18;
const uint8_t CMD_GET_MOTOR_FAIL = 21;

// Address Constants
const uint16_t ADDR_RUN_MODE = 0x7005;
const uint16_t ADDR_SPEED_REF = 0x700A;
const uint16_t ADDR_POSITION_REF = 0x7016;
const uint16_t ADDR_LIMIT_SPEED = 0x7017;
const uint16_t ADDR_ROTATION = 0x701D;
const uint16_t ADDR_IQ_REF = 0x7006;
const uint16_t ADDR_IQF = 0x701A;
const uint16_t ADDR_MECH_VEL = 0x701B;

// Mode Constants
const uint8_t MODE_MOTION = 0x00;
const uint8_t MODE_POSITION = 0x01;
const uint8_t MODE_SPEED = 0x02;
const uint8_t MODE_CURRENT = 0x03;

// Parameter Limits
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -30.0f;
const float V_MAX = 30.0f;
const float T_MIN = -12.0f;
const float T_MAX = 12.0f;
const float IQ_MIN = -27.0f;
const float IQ_MAX = 27.0f;

class CyberGear
{
public:
    CyberGear(const std::vector<std::pair<int, int>> &motor_list, const std::string &com_port = "/dev/ttyUSB0");
    void init_motor(int number, uint8_t mode);
    void enable_motor(int number);
    void disable_motor(int number);
    void set_mechpos_zero(int number);
    void set_limit_speed(int number, float speed);
    void set_run_mode(int number, uint8_t mode);
    void set_position(int number, float position);
    void set_speed(int number, float speed);
    void set_current(int number, float current);
    void request_status(int number);
    void request_current(int number);
    void request_mechvel(int number);
    void get_status();
    void print_list();
    ssize_t readFromPort(char *buffer, size_t bufferSize);

    float getPosition(int number) const { return position.at(number); }
    float getSpeed(int number) const { return speed.at(number); }
    float getTorque(int number) const { return torque.at(number); }
    float getTemperature(int number) const { return temperature.at(number); }

private:
    float uint_to_float(uint16_t x, float x_min, float x_max);
    void send_can_package(uint8_t can_id, uint8_t cmd_id, uint8_t option, const std::vector<uint8_t> &data);
    void send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max);

    std::unordered_map<int, int> _cybergear_can_id;
    int _master_can_id;
    std::unordered_map<int, uint8_t> _run_mode;
    int serial_fd;
    std::unordered_map<int, float> position;
    std::unordered_map<int, float> speed;
    std::unordered_map<int, float> torque;
    std::unordered_map<int, float> temperature;
    std::unordered_map<int, float> iqf;
    std::unordered_map<int, float> mechvel;
    std::unordered_map<int, float> mechpos;
    std::unordered_map<int, float> VBUS;
};

#endif // CYBERGEAR_HPP
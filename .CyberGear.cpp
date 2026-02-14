#include "cybergear_driver/CyberGear.hpp"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <thread>
#include <sys/ioctl.h>

CyberGear::CyberGear(const std::vector<std::pair<int, int>> &motor_list, const std::string &com_port)
    : _master_can_id(0)
{
    serial_fd = open(com_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0)
    {
        throw std::runtime_error("Failed to open serial port");
    }

    // Print port information
    printf("Serial port opened: %s\n", com_port.c_str());

    termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(serial_fd, &tty);

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8-bit chars
    tty.c_cflag &= ~PARENB;  // no parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // no HW flow control

    tcsetattr(serial_fd, TCSANOW, &tty);

    _master_can_id = 0;
    for (const auto &motor : motor_list)
    {
        int key = motor.first;
        int cybergear_can_id = motor.second;
        _cybergear_can_id[key] = cybergear_can_id;
        _run_mode[key] = MODE_POSITION;
        position[key] = 0;
        speed[key] = 0;
        torque[key] = 0;
        temperature[key] = 0;
        iqf[key] = 0;
        mechvel[key] = 0;
        mechpos[key] = 0;
        VBUS[key] = 0;
    }
}

void CyberGear::init_motor(int number, uint8_t mode)
{
    disable_motor(number);
    set_run_mode(number, mode);
}

void CyberGear::enable_motor(int number)
{
    std::vector<uint8_t> data(8, 0x00);
    send_can_package(_cybergear_can_id[number], CMD_ENABLE, _master_can_id, data);
}

void CyberGear::disable_motor(int number)
{
    std::vector<uint8_t> data(8, 0x00);
    send_can_package(_cybergear_can_id[number], CMD_STOP, _master_can_id, data);
}

void CyberGear::set_run_mode(int number, uint8_t mode)
{
    _run_mode[number] = mode;
    std::vector<uint8_t> data(8, 0x00);
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = mode;
    send_can_package(_cybergear_can_id[number], CMD_RAM_WRITE, _master_can_id, data);
}

void CyberGear::set_mechpos_zero(int number)
{
    std::vector<uint8_t> data(8, 0x00);
    data[0] = 1; // 1 byte for the command
    send_can_package(_cybergear_can_id[number], CMD_SET_MECH_POSITION_ZERO, _master_can_id, data);
}

void CyberGear::set_limit_speed(int number, float speed)
{
    send_can_float_package(_cybergear_can_id[number], ADDR_LIMIT_SPEED, speed, 0.0, V_MAX);
}
void CyberGear::set_position(int number, float position)
{
    send_can_float_package(_cybergear_can_id[number], ADDR_POSITION_REF, position, P_MIN, P_MAX);
}

void CyberGear::set_speed(int number, float speed)
{
    send_can_float_package(_cybergear_can_id[number], ADDR_SPEED_REF, speed, V_MIN, V_MAX);
}

void CyberGear::set_current(int number, float current)
{
    send_can_float_package(_cybergear_can_id[number], ADDR_IQ_REF, current, IQ_MIN, IQ_MAX);
}

void CyberGear::request_status(int number)
{
    std::vector<uint8_t> data(8, 0x00);
    send_can_package(_cybergear_can_id[number], CMD_GET_STATUS, _master_can_id, data);
}

void CyberGear::request_current(int number)
{
    std::vector<uint8_t> data(8, 0x00);
    data[0] = ADDR_IQF & 0x00FF;
    data[1] = ADDR_IQF >> 8;
    send_can_package(_cybergear_can_id[number], CMD_RAM_READ, _master_can_id, data);
}

void CyberGear::request_mechvel(int number)
{
    std::vector<uint8_t> data(8, 0x00);
    data[0] = ADDR_MECH_VEL & 0x00FF;
    data[1] = ADDR_MECH_VEL >> 8;
    send_can_package(_cybergear_can_id[number], CMD_RAM_READ, _master_can_id, data);
}

void CyberGear::get_status()
{
    std::vector<uint8_t> byte_list;
    uint8_t buffer[256];
    ssize_t bytes_read = 0;

    // 読み取り
    while (true)
    {
        bytes_read = read(serial_fd, buffer, sizeof(buffer));
        if (bytes_read > 0)
        {
            byte_list.insert(byte_list.end(), buffer, buffer + bytes_read);
        }
        else
        {
            break; // これ以上受信データがない
        }
    }

    // 17バイトごとにパース
    while (byte_list.size() >= 17)
    {
        if (byte_list[0] == 0x41 && byte_list[1] == 0x54)
        {
            std::vector<uint8_t> current_data(byte_list.begin(), byte_list.begin() + 17);
            byte_list.erase(byte_list.begin(), byte_list.begin() + 17);

            for (const auto &motor : _cybergear_can_id)
            {
                int key = motor.first;
                int can_id = motor.second;
                int extracted_can_id = (current_data[4] >> 3) | (current_data[3] << 5);
                extracted_can_id &= 0xFF;

                if (extracted_can_id == can_id)
                {
                    uint16_t raw_position = current_data[8] | (current_data[7] << 8);
                    uint16_t raw_speed = current_data[10] | (current_data[9] << 8);
                    uint16_t raw_torque = current_data[12] | (current_data[11] << 8);
                    uint16_t raw_temperature = current_data[14] | (current_data[13] << 8);

                    position[key] = uint_to_float(raw_position, P_MIN, P_MAX);
                    speed[key] = uint_to_float(raw_speed, V_MIN, V_MAX);
                    torque[key] = uint_to_float(raw_torque, T_MIN, T_MAX);
                    temperature[key] = raw_temperature / 10.0f;

                    // printf("Motor %d: Position: %f, Speed: %f, Torque: %f, Temperature: %f\n", key, position[key], speed[key], torque[key], temperature[key]);
                }
            }
        }
        else
        {
            byte_list.erase(byte_list.begin());
        }
    }

    if (!byte_list.empty())
    {
        std::cerr << "Received data error in get_status(): "
                  << byte_list.size() << " bytes remaining" << std::endl;
    }
}

void CyberGear::print_list()
{
    for (const auto &motor : _cybergear_can_id)
    {
        std::cout << motor.first << ", " << motor.second << std::endl;
    }
}

float CyberGear::uint_to_float(uint16_t x, float x_min, float x_max)
{
    const uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (x / static_cast<float>(type_max)) * span + x_min;
}

void CyberGear::send_can_package(uint8_t can_id, uint8_t cmd_id, uint8_t option, const std::vector<uint8_t> &data)
{
    uint32_t msg_id = (cmd_id << 27) | (option << 11) | (can_id << 3) | 0b100;

    std::vector<uint8_t> cdata = {
        0x41, 0x54, 0x00, 0x00, 0x00, 0x00,
        0x08, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x0d, 0x0a};

    cdata[2] = (msg_id >> 24) & 0xFF;
    cdata[3] = (msg_id >> 16) & 0xFF;
    cdata[4] = (msg_id >> 8) & 0xFF;
    cdata[5] = msg_id & 0xFF;

    for (size_t j = 0; j < 8; ++j)
    {
        cdata[7 + j] = data[j];
    }

    write(serial_fd, cdata.data(), cdata.size());
}

void pack_and_reverse_float(std::vector<uint8_t> &data, float value)
{
    // 4バイト分のfloatのビット表現を取得
    uint32_t float_bits;
    std::memcpy(&float_bits, &value, sizeof(float));
    // ネットワークバイトオーダーに変換
    float_bits = htonl(float_bits);
    // dataのインデックス4から4バイトにコピー
    std::memcpy(data.data() + 4, &float_bits, sizeof(uint32_t));
    // インデックス4～7までのバイトを反転
    std::reverse(data.begin() + 4, data.begin() + 8);
}

void CyberGear::send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max)
{
    std::vector<uint8_t> data(8, 0x00);
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;
    value = std::clamp(value, min, max);
    pack_and_reverse_float(data, value);
    send_can_package(can_id, CMD_RAM_WRITE, _master_can_id, data);
}

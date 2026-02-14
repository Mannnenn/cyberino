#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include <Arduino.h>
#include <mcp_can.h>

// Command Constants
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

#define MAX_MOTORS 4

struct MotorState
{
    uint8_t can_id;
    uint8_t run_mode;
    float position;
    float speed;
    float torque;
    float temperature;
    float iqf;
    float mechvel;
    bool active;
};

class CyberGear
{
public:
    CyberGear(MCP_CAN *can, uint8_t master_id = 0x00);

    // モーター管理
    bool addMotor(uint8_t motor_number, uint8_t can_id);
    void init_motor(uint8_t motor_number, uint8_t mode);
    void enable_motor(uint8_t motor_number);
    void disable_motor(uint8_t motor_number);

    // モーター設定
    void set_mechpos_zero(uint8_t motor_number);
    void set_limit_speed(uint8_t motor_number, float speed);
    void set_run_mode(uint8_t motor_number, uint8_t mode);
    void set_position(uint8_t motor_number, float position);
    void set_speed(uint8_t motor_number, float speed);
    void set_current(uint8_t motor_number, float current);

    // ステータス取得
    void request_status(uint8_t motor_number);
    void request_current(uint8_t motor_number);
    void request_mechvel(uint8_t motor_number);
    void process_can_message();

    // データアクセス
    float getPosition(uint8_t motor_number);
    float getSpeed(uint8_t motor_number);
    float getTorque(uint8_t motor_number);
    float getTemperature(uint8_t motor_number);

    void printStatus();

private:
    MCP_CAN *_can;
    uint8_t _master_can_id;
    MotorState _motors[MAX_MOTORS];

    float uint_to_float(uint16_t x, float x_min, float x_max);
    uint16_t float_to_uint(float x, float x_min, float x_max);
    void send_can_package(uint8_t can_id, uint8_t cmd_id, const uint8_t *data, uint8_t len);
    void send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min_val, float max_val);
    int8_t find_motor_index(uint8_t motor_number);
};

#endif // CYBERGEAR_H

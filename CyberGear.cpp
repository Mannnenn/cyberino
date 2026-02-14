#include "CyberGear.h"

CyberGear::CyberGear(MCP_CAN *can, uint8_t master_id)
    : _can(can), _master_can_id(master_id)
{
    // モーター状態の初期化
    for (int i = 0; i < MAX_MOTORS; i++)
    {
        _motors[i].active = false;
        _motors[i].can_id = 0;
        _motors[i].run_mode = MODE_POSITION;
        _motors[i].position = 0.0f;
        _motors[i].speed = 0.0f;
        _motors[i].torque = 0.0f;
        _motors[i].temperature = 0.0f;
        _motors[i].iqf = 0.0f;
        _motors[i].mechvel = 0.0f;
    }
}

bool CyberGear::addMotor(uint8_t motor_number, uint8_t can_id)
{
    if (motor_number >= MAX_MOTORS)
    {
        return false;
    }

    _motors[motor_number].can_id = can_id;
    _motors[motor_number].active = true;
    _motors[motor_number].run_mode = MODE_POSITION;

    return true;
}

void CyberGear::init_motor(uint8_t motor_number, uint8_t mode)
{
    disable_motor(motor_number);
    delay(10);
    set_run_mode(motor_number, mode);
    delay(10);
}

void CyberGear::enable_motor(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_package(_motors[idx].can_id, CMD_ENABLE, data, 8);
}

void CyberGear::disable_motor(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_package(_motors[idx].can_id, CMD_STOP, data, 8);
}

void CyberGear::set_run_mode(uint8_t motor_number, uint8_t mode)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    _motors[idx].run_mode = mode;

    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = mode;

    send_can_package(_motors[idx].can_id, CMD_RAM_WRITE, data, 8);
}

void CyberGear::set_mechpos_zero(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_package(_motors[idx].can_id, CMD_SET_MECH_POSITION_ZERO, data, 8);
}

void CyberGear::set_limit_speed(uint8_t motor_number, float speed)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    send_can_float_package(_motors[idx].can_id, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
}

void CyberGear::set_position(uint8_t motor_number, float position)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    send_can_float_package(_motors[idx].can_id, ADDR_POSITION_REF, position, P_MIN, P_MAX);
}

void CyberGear::set_speed(uint8_t motor_number, float speed)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    send_can_float_package(_motors[idx].can_id, ADDR_SPEED_REF, speed, V_MIN, V_MAX);
}

void CyberGear::set_current(uint8_t motor_number, float current)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    send_can_float_package(_motors[idx].can_id, ADDR_IQ_REF, current, IQ_MIN, IQ_MAX);
}

void CyberGear::request_status(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_package(_motors[idx].can_id, CMD_GET_STATUS, data, 8);
}

void CyberGear::request_current(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[0] = ADDR_IQF & 0x00FF;
    data[1] = ADDR_IQF >> 8;

    send_can_package(_motors[idx].can_id, CMD_RAM_READ, data, 8);
}

void CyberGear::request_mechvel(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return;

    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[0] = ADDR_MECH_VEL & 0x00FF;
    data[1] = ADDR_MECH_VEL >> 8;

    send_can_package(_motors[idx].can_id, CMD_RAM_READ, data, 8);
}

void CyberGear::process_can_message()
{
    unsigned long can_id;
    uint8_t len;
    uint8_t buf[8];

    // CAN メッセージの受信チェック
    if (_can->checkReceive() != CAN_MSGAVAIL)
    {
        return;
    }

    if (_can->readMsgBuf(&can_id, &len, buf) != CAN_OK)
    {
        return;
    }

    // 拡張フレームの場合、CAN IDからコマンドとモーターIDを抽出
    if (len == 8)
    {
        uint8_t cmd_id = (can_id >> 24) & 0x1F;
        uint8_t motor_can_id = (can_id >> 8) & 0xFF;

        // レスポンスメッセージの処理
        if (cmd_id == CMD_RESPONSE)
        {
            // モーターを検索
            for (int i = 0; i < MAX_MOTORS; i++)
            {
                if (_motors[i].active && _motors[i].can_id == motor_can_id)
                {
                    // ステータスデータのデコード
                    uint16_t raw_position = (buf[1] << 8) | buf[0];
                    uint16_t raw_speed = (buf[3] << 8) | buf[2];
                    uint16_t raw_torque = (buf[5] << 8) | buf[4];
                    uint16_t raw_temperature = (buf[7] << 8) | buf[6];

                    _motors[i].position = uint_to_float(raw_position, P_MIN, P_MAX);
                    _motors[i].speed = uint_to_float(raw_speed, V_MIN, V_MAX);
                    _motors[i].torque = uint_to_float(raw_torque, T_MIN, T_MAX);
                    _motors[i].temperature = raw_temperature / 10.0f;

                    break;
                }
            }
        }
    }
}

float CyberGear::getPosition(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return 0.0f;
    return _motors[idx].position;
}

float CyberGear::getSpeed(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return 0.0f;
    return _motors[idx].speed;
}

float CyberGear::getTorque(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return 0.0f;
    return _motors[idx].torque;
}

float CyberGear::getTemperature(uint8_t motor_number)
{
    int8_t idx = find_motor_index(motor_number);
    if (idx < 0)
        return 0.0f;
    return _motors[idx].temperature;
}

void CyberGear::printStatus()
{
    for (int i = 0; i < MAX_MOTORS; i++)
    {
        if (_motors[i].active)
        {
            Serial.print("Motor ");
            Serial.print(i);
            Serial.print(" (CAN ID: 0x");
            Serial.print(_motors[i].can_id, HEX);
            Serial.print("): Pos=");
            Serial.print(_motors[i].position, 3);
            Serial.print(" Spd=");
            Serial.print(_motors[i].speed, 3);
            Serial.print(" Trq=");
            Serial.print(_motors[i].torque, 3);
            Serial.print(" Temp=");
            Serial.println(_motors[i].temperature, 1);
        }
    }
}

// ========== プライベート関数 ==========

float CyberGear::uint_to_float(uint16_t x, float x_min, float x_max)
{
    const uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return ((float)x / (float)type_max) * span + x_min;
}

uint16_t CyberGear::float_to_uint(float x, float x_min, float x_max)
{
    const uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    float offset = x - x_min;
    return (uint16_t)((offset / span) * type_max);
}

void CyberGear::send_can_package(uint8_t can_id, uint8_t cmd_id, const uint8_t *data, uint8_t len)
{
    // CAN IDの構築: (cmd_id << 24) | (master_id << 8) | motor_can_id
    uint32_t msg_id = ((uint32_t)cmd_id << 24) | ((uint32_t)_master_can_id << 8) | can_id;

    // 拡張フレーム(Extended)で送信
    _can->sendMsgBuf(msg_id, 1, len, (uint8_t *)data);
}

void CyberGear::send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min_val, float max_val)
{
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // アドレスの設定
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;

    // 値のクランプ
    if (value < min_val)
        value = min_val;
    if (value > max_val)
        value = max_val;

    // float を uint32_t のビットパターンとしてコピー
    uint32_t float_bits;
    memcpy(&float_bits, &value, sizeof(float));

    // リトルエンディアンでデータ配列に格納
    data[4] = (float_bits >> 0) & 0xFF;
    data[5] = (float_bits >> 8) & 0xFF;
    data[6] = (float_bits >> 16) & 0xFF;
    data[7] = (float_bits >> 24) & 0xFF;

    send_can_package(can_id, CMD_RAM_WRITE, data, 8);
}

int8_t CyberGear::find_motor_index(uint8_t motor_number)
{
    if (motor_number >= MAX_MOTORS)
    {
        return -1;
    }

    if (!_motors[motor_number].active)
    {
        return -1;
    }

    return motor_number;
}

/* CyberGear Motor Control via CAN
 * This example controls a CyberGear motor using MCP2515 CAN controller
 * Features:
 * - Motor initialization and enable/disable
 * - Position control
 * - Status monitoring (position, speed, torque, temperature)
 */

#include <mcp_can.h>
#include <SPI.h>

// ESPr Developer S3 向けのピン設定
#define SPI_SCK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define CAN_CS_PIN 10
#define CAN_INT_PIN 9

// CyberGear Command Constants
#define CMD_POSITION 1
#define CMD_RESPONSE 2
#define CMD_ENABLE 3
#define CMD_STOP 4
#define CMD_SET_MECH_POSITION_ZERO 6
#define CMD_GET_STATUS 15
#define CMD_RAM_READ 17
#define CMD_RAM_WRITE 18

// CyberGear Address Constants
#define ADDR_RUN_MODE 0x7005
#define ADDR_POSITION_REF 0x7016
#define ADDR_LIMIT_SPEED 0x7017

// Mode Constants
#define MODE_MOTION 0x00
#define MODE_POSITION 0x01
#define MODE_SPEED 0x02
#define MODE_CURRENT 0x03

// Parameter Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

// Motor Configuration
#define MOTOR_CAN_ID 0x01  // CyberGearのCAN ID
#define MASTER_CAN_ID 0x00 // マスター(Arduino)のCAN ID

// Motor State Variables
float motor_position = 0.0f;
float motor_speed = 0.0f;
float motor_torque = 0.0f;
float motor_temperature = 0.0f;

// Control Variables
unsigned long prevTX = 0;
const unsigned int invlTX = 100; // 100ms interval
float target_position = 0.0f;
bool motor_enabled = false;

// CAN Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];
char msgString[128];

MCP_CAN CAN0(CAN_CS_PIN);

// ========== Utility Functions ==========

// Convert uint16_t to float based on min/max range
float uint_to_float(uint16_t x, float x_min, float x_max)
{
  const uint16_t type_max = 0xFFFF;
  float span = x_max - x_min;
  return (x / (float)type_max) * span + x_min;
}

// Convert float to uint16_t based on min/max range
uint16_t float_to_uint(float x, float x_min, float x_max)
{
  const uint16_t type_max = 0xFFFF;
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * type_max / span);
}

// Constrain float value
float constrain_float(float value, float min_val, float max_val)
{
  if (value < min_val)
    return min_val;
  if (value > max_val)
    return max_val;
  return value;
}

// ========== CyberGear CAN Functions ==========

// Send CAN package to CyberGear motor
void send_can_package(uint8_t can_id, uint8_t cmd_id, uint8_t option, uint8_t *data)
{
  // CyberGear CAN ID format: cmd_id(5bit) + option(8bit) + can_id(8bit) + reserved(3bit) + 0b100
  uint32_t msg_id = ((uint32_t)cmd_id << 24) | ((uint32_t)option << 16) | ((uint32_t)can_id << 8) | 0x04;

  byte sndStat = CAN0.sendMsgBuf(msg_id, 1, 8, data); // 1 = Extended Frame

  if (sndStat != CAN_OK)
  {
    Serial.println("Error sending CAN message");
  }
}

// Send float value to CyberGear motor
void send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min_val, float max_val)
{
  uint8_t data[8] = {0};
  data[0] = addr & 0xFF;
  data[1] = (addr >> 8) & 0xFF;

  // Constrain value
  value = constrain_float(value, min_val, max_val);

  // Convert float to bytes (little endian)
  union
  {
    float f;
    uint8_t bytes[4];
  } float_union;
  float_union.f = value;

  data[4] = float_union.bytes[0];
  data[5] = float_union.bytes[1];
  data[6] = float_union.bytes[2];
  data[7] = float_union.bytes[3];

  send_can_package(can_id, CMD_RAM_WRITE, MASTER_CAN_ID, data);
}

// Enable motor
void enable_motor()
{
  uint8_t data[8] = {0};
  send_can_package(MOTOR_CAN_ID, CMD_ENABLE, MASTER_CAN_ID, data);
  motor_enabled = true;
  Serial.println("Motor enabled");
}

// Disable motor
void disable_motor()
{
  uint8_t data[8] = {0};
  send_can_package(MOTOR_CAN_ID, CMD_STOP, MASTER_CAN_ID, data);
  motor_enabled = false;
  Serial.println("Motor disabled");
}

// Set run mode
void set_run_mode(uint8_t mode)
{
  uint8_t data[8] = {0};
  data[0] = ADDR_RUN_MODE & 0xFF;
  data[1] = (ADDR_RUN_MODE >> 8) & 0xFF;
  data[4] = mode;
  send_can_package(MOTOR_CAN_ID, CMD_RAM_WRITE, MASTER_CAN_ID, data);
  Serial.print("Run mode set to: ");
  Serial.println(mode);
}

// Set mechanical position to zero
void set_mechpos_zero()
{
  uint8_t data[8] = {0};
  data[0] = 1;
  send_can_package(MOTOR_CAN_ID, CMD_SET_MECH_POSITION_ZERO, MASTER_CAN_ID, data);
  Serial.println("Mechanical position zeroed");
}

// Set limit speed
void set_limit_speed(float speed)
{
  send_can_float_package(MOTOR_CAN_ID, ADDR_LIMIT_SPEED, speed, 0.0, V_MAX);
  Serial.print("Limit speed set to: ");
  Serial.println(speed);
}

// Set target position
void set_position(float position)
{
  send_can_float_package(MOTOR_CAN_ID, ADDR_POSITION_REF, position, P_MIN, P_MAX);
}

// Request motor status
void request_status()
{
  uint8_t data[8] = {0};
  send_can_package(MOTOR_CAN_ID, CMD_GET_STATUS, MASTER_CAN_ID, data);
}

// Initialize motor with proper sequence
void init_motor()
{
  Serial.println("Initializing motor...");

  // 1. Disable motor first
  disable_motor();
  delay(100);

  // 2. Set run mode to position control
  set_run_mode(MODE_POSITION);
  delay(100);

  // 3. Set mechanical position zero
  set_mechpos_zero();
  delay(100);

  // 4. Set limit speed
  set_limit_speed(0.5f);
  delay(100);

  // 5. Enable motor
  enable_motor();
  delay(100);

  Serial.println("Motor initialization complete");
}

// Parse received CAN message
void parse_motor_feedback()
{
  if (!digitalRead(CAN_INT_PIN))
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // Extract CAN ID from extended frame
    uint8_t received_can_id = (rxId >> 8) & 0xFF;
    uint8_t cmd = (rxId >> 24) & 0x1F; // 5-bit command

    if (received_can_id == MOTOR_CAN_ID && len == 8)
    {
      // Parse motor feedback data (little endian)
      uint16_t raw_position = rxBuf[1] | (rxBuf[0] << 8);
      uint16_t raw_speed = rxBuf[3] | (rxBuf[2] << 8);
      uint16_t raw_torque = rxBuf[5] | (rxBuf[4] << 8);
      uint16_t raw_temperature = rxBuf[7] | (rxBuf[6] << 8);

      motor_position = uint_to_float(raw_position, P_MIN, P_MAX);
      motor_speed = uint_to_float(raw_speed, V_MIN, V_MAX);
      motor_torque = uint_to_float(raw_torque, T_MIN, T_MAX);
      motor_temperature = raw_temperature / 10.0f;
    }
  }
}

// Print motor status
void print_motor_status()
{
  Serial.print("Pos: ");
  Serial.print(motor_position, 3);
  Serial.print(" rad, Spd: ");
  Serial.print(motor_speed, 2);
  Serial.print(" rad/s, Trq: ");
  Serial.print(motor_torque, 2);
  Serial.print(" Nm, Temp: ");
  Serial.print(motor_temperature, 1);
  Serial.println(" C");
}

// ========== Arduino Setup ==========

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== CyberGear Motor Control ===");

  // 1. ESP32-S3のピン設定に合わせてSPIバスを初期化
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN_CS_PIN);

  // 2. MCP2515の初期化 (COMMUモジュールは8MHzの水晶発振器を搭載)
  // CyberGearは1Mbps CANバスを使用
  if (CAN_OK == CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ))
  {
    Serial.println("CAN Init OK!");
  }
  else
  {
    Serial.println("CAN Init Failed...");
    while (1)
      ; // エラー時は停止
  }

  // ノーマルモードに設定
  CAN0.setMode(MCP_NORMAL);

  // 割り込みピンの入力設定
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  // CyberGearモーターの初期化（cybergear_driver.cpp準拠）
  delay(500);
  init_motor();

  Serial.println("\nCommands:");
  Serial.println("  e - Enable motor");
  Serial.println("  d - Disable motor");
  Serial.println("  z - Set zero position");
  Serial.println("  p<value> - Set position (e.g., p1.5)");
  Serial.println("  s - Print status");
  Serial.println();
}

void loop()
{
  // Parse incoming CAN messages
  parse_motor_feedback();

  // Periodic motor control (cybergear_driver.cpp style)
  if (millis() - prevTX >= invlTX)
  {
    prevTX = millis();

    if (motor_enabled)
    {
      // Request motor status
      request_status();
      delay(10);

      // Set target position
      set_position(target_position);
    }
  }

  // Handle serial commands
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "e")
    {
      enable_motor();
    }
    else if (cmd == "d")
    {
      disable_motor();
    }
    else if (cmd == "z")
    {
      set_mechpos_zero();
    }
    else if (cmd == "s")
    {
      print_motor_status();
    }
    else if (cmd.startsWith("p"))
    {
      float pos = cmd.substring(1).toFloat();
      target_position = constrain_float(pos, P_MIN, P_MAX);
      Serial.print("Target position set to: ");
      Serial.println(target_position, 3);
    }
    else if (cmd.length() > 0)
    {
      Serial.println("Unknown command");
    }
  }
}

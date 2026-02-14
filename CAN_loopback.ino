#include <SPI.h>
#include <mcp_can.h>
#include "CyberGear.h"

// ESPr Developer S3 向けのピン設定
#define SPI_SCK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define CAN_CS_PIN 10
#define CAN_INT_PIN 9

MCP_CAN CAN(CAN_CS_PIN);

// --- CyberGear用の設定 ---
uint8_t MOTOR_ID = 0x01;  // ターゲットのモーターID (購入時のデフォルトは0x7Fが多いです)
uint8_t MASTER_ID = 0x00; // ESP32側のID (0x00などでOK)

// CyberGearクラスのインスタンス
CyberGear *cybergear;

void setup()
{
  Serial.begin(115200);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN_CS_PIN);

  // 【重要】CyberGearは1Mbps通信。COMMUは8MHz水晶なので「CAN_1000KBPS, MCP_8MHZ」を指定
  if (CAN_OK == CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ))
  {
    Serial.println("CAN Init OK!");
  }
  else
  {
    Serial.println("CAN Init Failed...");
    while (1)
      ;
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  delay(1000);

  // CyberGearクラスを使用した初期化
  cybergear = new CyberGear(&CAN, MASTER_ID);

  // モーターを登録（モーター番号0にCAN ID 0x01のモーターを登録）
  cybergear->addMotor(0, MOTOR_ID);

  // モーターを初期化（ポジションモード）
  cybergear->init_motor(0, MODE_POSITION);
  delay(100);

  // ゼロ位置を設定
  cybergear->set_mechpos_zero(0);
  delay(100);

  // 速度制限を設定（rad/s）
  cybergear->set_limit_speed(0, 5.0);
  delay(100);

  // モーターを有効化
  cybergear->enable_motor(0);
  delay(100);

  Serial.println("CyberGear Motor Initialized!");
}

void loop()
{
  static unsigned long lastTime = 0;
  static float targetPosition = 0.0;
  static bool direction = true;

  unsigned long currentTime = millis();

  // 受信したCANメッセージを処理
  cybergear->process_can_message();

  // 3秒ごとに位置を変更
  if (currentTime - lastTime >= 3000)
  {
    lastTime = currentTime;

    // 位置を-3.0から+3.0 rad の間で往復させる
    if (direction)
    {
      targetPosition += 3.0;
      if (targetPosition >= 3.0)
      {
        targetPosition = 3.0;
        direction = false;
      }
    }
    else
    {
      targetPosition -= 3.0;
      if (targetPosition <= -3.0)
      {
        targetPosition = -3.0;
        direction = true;
      }
    }

    // 目標位置を設定
    cybergear->set_position(0, targetPosition);
    Serial.print("Target Position: ");
    Serial.println(targetPosition, 3);
  }

  // 100msごとにステータスを要求して表示
  static unsigned long lastStatusTime = 0;
  if (currentTime - lastStatusTime >= 100)
  {
    lastStatusTime = currentTime;

    // ステータスを要求
    cybergear->request_status(0);

    // 少し待ってからステータスを表示
    delay(10);
    cybergear->process_can_message();

    Serial.print("Position: ");
    Serial.print(cybergear->getPosition(0), 3);
    Serial.print(" rad, Speed: ");
    Serial.print(cybergear->getSpeed(0), 3);
    Serial.print(" rad/s, Torque: ");
    Serial.print(cybergear->getTorque(0), 3);
    Serial.print(" Nm, Temp: ");
    Serial.print(cybergear->getTemperature(0), 1);
    Serial.println(" C");
  }

  delay(10);
}

// 以下は従来の関数（参考用に残しています）

// モーター有効化 (コマンドモード: 1)
void enableMotor(uint8_t motor_id, uint8_t master_id)
{
  // CAN IDの構築: (コマンド=1 << 24) | (マスターID << 8) | モーターID
  uint32_t can_id = (1 << 24) | (master_id << 8) | motor_id;

  // データ長は8バイト、中身はすべて0
  uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // 送信: 第2引数の「1」は拡張フレーム(Extended)を意味する
  byte sndStat = CAN.sendMsgBuf(can_id, 1, 8, data);
  if (sndStat == CAN_OK)
  {
    Serial.println("Motor Enabled!");
  }
}

// モーター停止 (コマンドモード: 3)
void stopMotor(uint8_t motor_id, uint8_t master_id)
{
  // CAN IDの構築: (コマンド=3 << 24) | (マスターID << 8) | モーターID
  uint32_t can_id = (3 << 24) | (master_id << 8) | motor_id;

  uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  byte sndStat = CAN.sendMsgBuf(can_id, 1, 8, data);
  if (sndStat == CAN_OK)
  {
    Serial.println("Motor Stopped!");
  }
}
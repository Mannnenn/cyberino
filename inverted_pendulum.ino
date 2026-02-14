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
  Serial.begin(921600);
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

  // モーターを初期化（速度モード）
  cybergear->init_motor(0, MODE_SPEED);
  delay(100);

  // ゼロ位置を設定
  cybergear->set_mechpos_zero(0);
  delay(100);

  // 速度制限を設定（rad/s）
  cybergear->set_limit_speed(0, 10.0);
  delay(100);

  // モーターを有効化
  cybergear->enable_motor(0);
  delay(100);

  Serial.println("CyberGear Motor Initialized (Speed Mode)!");
}

// === 調整可能なパラメータ ===
const float SINE_WAVE_PERIOD_SEC = 5.0;                 // sin波の周期（秒）
const float MAX_SPEED_AMPLITUDE = 5.0;                  // 最大速度の振幅（rad/s）
const unsigned long SPEED_UPDATE_INTERVAL_US = 5000;   // 速度更新間隔（マイクロ秒）10ms=10000us
const unsigned long STATUS_REQUEST_INTERVAL_US = 5000; // ステータス要求間隔（マイクロ秒）10ms=10000us
const unsigned long LOOP_DELAY_US = 1000;               // loop最後のdelay（マイクロ秒）1ms=1000us

void loop()
{
  static unsigned long lastSpeedUpdateTime = 0;
  static unsigned long lastStatusTime = 0;

  unsigned long currentTime = micros();

  // 受信したCANメッセージを処理
  cybergear->process_can_message();

  // 指定間隔ごとにsin波で速度を更新
  if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL_US)
  {
    lastSpeedUpdateTime = currentTime;

    // 現在時刻からsin波の位相を計算
    float timeInSeconds = millis() / 1000.0;
    float phase = (2.0 * PI * timeInSeconds) / SINE_WAVE_PERIOD_SEC;

    // sin波で目標速度を計算
    float targetSpeed = MAX_SPEED_AMPLITUDE * sin(phase);

    // 目標速度を設定
    cybergear->set_speed(0, targetSpeed);
    Serial.print(targetSpeed, 3);
    Serial.print(", ");
  }

  // 指定間隔ごとにステータスを要求して表示
  if (currentTime - lastStatusTime >= STATUS_REQUEST_INTERVAL_US)
  {
    lastStatusTime = currentTime;

    // ステータスを要求
    cybergear->request_status(0);

    // 少し待ってからステータスを表示
    delayMicroseconds(250);
    cybergear->process_can_message();

    Serial.println(cybergear->getSpeed(0), 3);
  }

  delayMicroseconds(LOOP_DELAY_US);
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
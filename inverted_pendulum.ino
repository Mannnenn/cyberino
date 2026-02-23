#include <SPI.h>
#include <mcp_can.h>
#include "CyberGear.h"
#include "ICM42688.h"

// ESPr Developer S3 向けのピン設定
#define SPI_SCK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define CAN_CS_PIN 10
#define CAN_INT_PIN 9
#define IMU_CS_PIN 20

MCP_CAN CAN(CAN_CS_PIN);

// --- CyberGear用の設定 ---
uint8_t MOTOR_ID_1 = 0x01; // 1つ目のモーターID
uint8_t MOTOR_ID_2 = 0x02; // 2つ目のモーターID
uint8_t MASTER_ID = 0x00;  // ESP32側のID (0x00などでOK)

// CyberGearクラスのインスタンス
CyberGear *cybergear;

// ICM42688センサーのインスタンス
ICM42688 IMU(SPI, IMU_CS_PIN);
void setup()
{

  Serial.begin(2000000);
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
  cybergear->addMotor(0, MOTOR_ID_1);
  cybergear->addMotor(1, MOTOR_ID_2);

  // モーターを初期化（速度モード）
  cybergear->init_motor(0, MODE_SPEED);
  delay(100);
  cybergear->init_motor(1, MODE_SPEED);
  delay(100);

  // モーターのゼロ位置を設定
  cybergear->set_mechpos_zero(0);
  delay(100);
  cybergear->set_mechpos_zero(1);
  delay(100);

  // モーターの速度制限を設定（rad/s）
  cybergear->set_limit_speed(0, 10.0);
  delay(100);
  cybergear->set_limit_speed(1, 10.0);
  delay(100);

  // モーターを有効化
  cybergear->enable_motor(0);
  delay(100);
  cybergear->enable_motor(1);
  delay(100);

  Serial.println("CyberGear Motors Initialized (Speed Mode)!");

  // IMUの初期化
  int status = IMU.begin();
  if (status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }

  // IMUの設定
  IMU.setAccelFS(ICM42688::gpm2);    // 加速度計のフルスケール: +/-2G
  IMU.setGyroFS(ICM42688::dps62_5);  // ジャイロのフルスケール: +/-62.5 deg/s
  IMU.setAccelODR(ICM42688::odr100); // 加速度計のサンプルレート: 100Hz
  IMU.setGyroODR(ICM42688::odr100);  // ジャイロのサンプルレート: 100Hz

  Serial.println("IMU Initialized!");
}

// === 調整可能なパラメータ ===
const float SINE_WAVE_PERIOD_SEC = 5.0;                // sin波の周期（秒）
const float MAX_SPEED_AMPLITUDE = 5.0;                 // 最大速度の振幅（rad/s）
const unsigned long SPEED_UPDATE_INTERVAL_US = 7500;   // 速度更新間隔（マイクロ秒）10ms=10000us
const unsigned long STATUS_REQUEST_INTERVAL_US = 7500; // ステータス要求間隔（マイクロ秒）10ms=10000us
const unsigned long IMU_UPDATE_INTERVAL_US = 10000;    // IMU読み取り間隔（マイクロ秒）10ms=10000us
const unsigned long LOOP_DELAY_US = 1000;              // loop最後のdelay（マイクロ秒）1ms=1000us

void loop()
{
  static unsigned long lastSpeedUpdateTime = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastIMUTime = 0;

  unsigned long currentTime = micros();

  // 受信したCANメッセージを処理
  cybergear->process_can_message();

  // 指定間隔ごとにsin波/cos波で速度を更新
  if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL_US)
  {
    lastSpeedUpdateTime = currentTime;

    // 現在時刻から位相を計算
    float timeInSeconds = millis() / 1000.0;
    float phase = (2.0 * PI * timeInSeconds) / SINE_WAVE_PERIOD_SEC;

    // モーター1: sin波で目標速度を計算
    float targetSpeed1 = MAX_SPEED_AMPLITUDE * sin(phase);
    // モーター2: cos波で目標速度を計算
    float targetSpeed2 = MAX_SPEED_AMPLITUDE * cos(phase);

    // 目標速度を設定
    cybergear->set_speed(0, targetSpeed1);
    cybergear->set_speed(1, targetSpeed2);

    // Serial.print(targetSpeed1, 3);
    // Serial.print(", ");
    // Serial.print(targetSpeed2, 3);
    // Serial.print(", ");
  }

  // 指定間隔ごとにステータスを要求して表示
  if (currentTime - lastStatusTime >= STATUS_REQUEST_INTERVAL_US)
  {
    lastStatusTime = currentTime;

    // モーター1のステータスを要求
    cybergear->request_status(0);
    delayMicroseconds(250);
    cybergear->process_can_message();

    // モーター2のステータスを要求
    cybergear->request_status(1);
    delayMicroseconds(250);
    cybergear->process_can_message();

    // Serial.print(cybergear->getSpeed(0), 3);
    // Serial.print(", ");
    // Serial.print(cybergear->getSpeed(1), 3);
    // Serial.print(", ");
  }

  // 指定間隔ごとにIMUデータを読み取って表示
  if (currentTime - lastIMUTime >= IMU_UPDATE_INTERVAL_US)
  {
    lastIMUTime = currentTime;

    // IMUデータを読み取る
    IMU.getAGT();

    // IMUデータを出力（加速度、ジャイロ、温度）
    // Serial.print(IMU.accX(), 3);
    // Serial.print(", ");
    // Serial.print(IMU.accY(), 3);
    // Serial.print(", ");
    // Serial.print(IMU.accZ(), 3);
    // Serial.print(", ");
    Serial.print(IMU.gyrX(), 3);
    Serial.print(", ");
    Serial.print(IMU.gyrY(), 3);
    Serial.print(", ");
    Serial.println(IMU.gyrZ(), 3);
    // Serial.print(", ");
    // Serial.println(IMU.temp(), 3);
  }

  delayMicroseconds(LOOP_DELAY_US);
}

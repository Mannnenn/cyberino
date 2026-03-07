#include <WiFi.h>
#include <NetworkUdp.h>
#include <SPI.h>
#include <mcp_can.h>
#include <math.h>
#include "CyberGear.h"
#include "ICM42688.h"
#include "madwick_1axis.h"
#include "sbus.h"
#include "RobotState.h"
#include "StateFeedback.h"

// --- 設定: WiFi & UDP ---
const char *networkName = "Coron28";
const char *networkPswd = "masa1222";
const char *udpAddress = "192.168.0.89";
const int udpPort = 3333;

NetworkUDP udp;
boolean wifiConnected = false;

// WiFiイベントハンドラ
void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());
    udp.begin(WiFi.localIP(), udpPort);
    wifiConnected = true;
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    wifiConnected = false;
    break;
  default:
    break;
  }
}

void connectToWiFi(const char *ssid, const char *pwd)
{
  Serial.println("Connecting to WiFi network: " + String(ssid));
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, pwd);
}

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
Madgwick1Axis madwickFilter(0.3f, 1.0f / 0.9935f); // フィルタゲイン0.3, 加速度スケール補正

// === 調整可能なパラメータ ===
const unsigned long SPEED_UPDATE_INTERVAL_US = 2500;                   // 速度更新間隔（マイクロ秒）10ms=10000us
const unsigned long STATUS_REQUEST_INTERVAL_US = 2500;                 // ステータス要求間隔（マイクロ秒）10ms=10000us
const float MOTOR_UPDATE_DT = STATUS_REQUEST_INTERVAL_US / 1000000.0f; // モーター更新の周期（秒）
const unsigned long IMU_UPDATE_INTERVAL_US = 1000;                     // IMU読み取り間隔（マイクロ秒）10ms=10000us
const unsigned long PRINT_INTERVAL_US = 10000;                         // データ保存間隔（マイクロ秒）
const int BUFFER_SIZE = 20;                                            // 送信バッファサイズ

struct LogData
{
  uint32_t timestamp;
  float pendulumAngle;
  float pendulumAngularVelocity;
  float wheelSpeed;
  float wheelAngle;
  float targetTorque;
};

LogData logBuffer[BUFFER_SIZE];
int bufferIndex = 0;

const float FORWARD_COMMAND_SCALE = 4.0f; // 前後の指令のスケーリング係数,m/s
const float TURN_COMMAND_SCALE = 8.0f;    // 左右の指令のスケーリング係数,deg/s
const float WHEEL_BASE = 0.1772f;         // 車輪間距離（m）
const float WHEEL_RADIUS = 0.108f;        // 車輪半径（m）
const float TORQUE_CONSTANT = 0.87f;      // モーターのトルク定数（Nm/A）※実際のモーターに合わせて調整が必要

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, 6, 7, true);
/* SBUS data */
bfs::SbusData data;

RobotState robotState;
StateFeedback control(-14.33549606f, -2.55884175f, 0.70710678f, 0.08855246f);

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
  delay(100);
  cybergear->addMotor(1, MOTOR_ID_2);
  delay(100);

  // モーターを初期化（速度モード）
  cybergear->init_motor(0, MODE_CURRENT);
  delay(100);
  cybergear->init_motor(1, MODE_CURRENT);
  delay(100);

  // モーターのゼロ位置を設定
  cybergear->set_mechpos_zero(0);
  delay(100);
  cybergear->set_mechpos_zero(1);
  delay(100);

  // モーターの速度制限を設定（rad/s）
  cybergear->set_limit_speed(0, 15.0);
  delay(100);
  cybergear->set_limit_speed(1, 15.0);
  delay(100);

  // モーターを有効化
  cybergear->enable_motor(0);
  delay(100);
  cybergear->enable_motor(1);
  delay(100);

  // Serial.println("CyberGear Motors Initialized!"); // enable_motorはloop内で開始後N秒で行う

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
  IMU.setAccelFS(ICM42688::gpm2);   // 加速度計のフルスケール: +/-2G
  IMU.setGyroFS(ICM42688::dps250);  // ジャイロのフルスケール: +/-62.5 deg/s
  IMU.setAccelODR(ICM42688::odr1k); // 加速度計のサンプルレート: 100Hz
  IMU.setGyroODR(ICM42688::odr1k);  // ジャイロのサンプルレート: 100Hz

  Serial.println("IMU Initialized!");

  sbus_rx.Begin();

  // WiFi接続開始
  connectToWiFi(networkName, networkPswd);
}

void loop()
{
  static unsigned long lastSpeedUpdateTime = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastIMUTime = 0;
  static unsigned long lastPrintTime = 0;
  static StateFeedback::Torque target_torque_sf = {0.0f, 0.0f};
  static float forward_velocity_command = 0.0f;
  static float turn_velocity_command = 0.0f;
  static bool enabled_motor_command = false;
  static unsigned long controller_timestamp_ms = 0;

  unsigned long currentTime = micros();

  // 指定間隔ごとに速度を更新
  if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL_US)
  {
    lastSpeedUpdateTime = currentTime;

    controller_timestamp_ms = millis();

    target_torque_sf = control.calculateTorque(robotState);

    // 目標速度を設定 (最初のN秒間は指令を出さない)
    if (enabled_motor_command)
    {
      cybergear->set_current(0, -target_torque_sf.left / TORQUE_CONSTANT); // 左右の車輪で逆符号の速度指令を出すことで回転も可能にする
      cybergear->set_current(1, target_torque_sf.right / TORQUE_CONSTANT); // 左右の車輪で逆符号の速度指令を出すことで回転も可能にする
    }
    else
    {
      cybergear->set_current(0, 0.0f);
      cybergear->set_current(1, 0.0f);
    }
  }

  // 指定間隔ごとにステータスを要求して表示
  if (currentTime - lastStatusTime >= STATUS_REQUEST_INTERVAL_US)
  {
    lastStatusTime = currentTime;

    // モーター1のステータスを要求
    cybergear->request_status(0);
    delayMicroseconds(250);
    cybergear->process_can_message();
    delayMicroseconds(100);
    // モーター2のステータスを要求
    cybergear->request_status(1);
    delayMicroseconds(250);
    cybergear->process_can_message();

    robotState.updateWheel(cybergear->getSpeed(0), -cybergear->getSpeed(1), MOTOR_UPDATE_DT);
  }

  // 指定間隔ごとにIMUデータを読み取って表示
  if (currentTime - lastIMUTime >= IMU_UPDATE_INTERVAL_US)
  {
    lastIMUTime = currentTime;

    // IMUデータを読み取る
    IMU.getAGT();

    // Madgwickフィルタを更新
    madwickFilter.update(IMU.accZ(), IMU.accX(), -IMU.gyrY(), currentTime);

    robotState.updatePendulumAngle(madwickFilter.getAngle());
    robotState.updatePendulumAngularVelocity(-IMU.gyrY(), IMU.gyrZ()); // ジャイロのY軸を振子の角速度、Z軸をヨーレイトとして更新
  }

  if (sbus_rx.Read())
  {
    /* Grab the received data */
    data = sbus_rx.data();
    float forward_command = static_cast<float>(map(data.ch[2], 368, 1680, -1000, 1000)) / 1000.0f; // 前後の指令を-1.0から1.0の範囲にマッピング
    float turn_command = static_cast<float>(map(data.ch[0], 368, 1680, -1000, 1000)) / 1000.0f;    // 左右の指令を-1.0から1.0の範囲にマッピング
    enabled_motor_command = (data.ch[4] > 1000);                                                   // チャンネル4の値が1000を超えていればモーターを有効化する

    forward_velocity_command = FORWARD_COMMAND_SCALE * forward_command / WHEEL_RADIUS; // 前後の指令を-1.0から1.0の範囲にマッピングしたものを速度指令に変換し，車輪半径で割って角速度指令に変換
    turn_velocity_command = TURN_COMMAND_SCALE * turn_command;                         // 左右の指令を-1.0から1.0の範囲にマッピングしたものを速度指令に変換し，
  }

  if (currentTime - lastPrintTime >= PRINT_INTERVAL_US)
  {
    lastPrintTime = currentTime;

    // バッファにデータを保存
    logBuffer[bufferIndex].timestamp = controller_timestamp_ms;
    logBuffer[bufferIndex].pendulumAngle = robotState.getPendulumAngle();
    logBuffer[bufferIndex].pendulumAngularVelocity = robotState.getPendulumAngularVelocity();
    logBuffer[bufferIndex].wheelSpeed = robotState.getWheelSpeed();
    logBuffer[bufferIndex].wheelAngle = robotState.getWheelAngle();
    logBuffer[bufferIndex].targetTorque = target_torque_sf.left;
    bufferIndex++;

    // バッファが一杯になったら送信
    if (bufferIndex >= BUFFER_SIZE)
    {
      if (wifiConnected)
      {
        udp.beginPacket(udpAddress, udpPort);
        for (int i = 0; i < BUFFER_SIZE; i++)
        {
          udp.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                     logBuffer[i].timestamp,
                     logBuffer[i].pendulumAngle,
                     logBuffer[i].pendulumAngularVelocity,
                     logBuffer[i].wheelSpeed,
                     logBuffer[i].wheelAngle,
                     logBuffer[i].targetTorque);
        }
        udp.endPacket();
      }
      bufferIndex = 0;
    }
  }
}

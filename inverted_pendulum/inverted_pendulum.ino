#include <SPI.h>
#include <mcp_can.h>
#include <math.h>
#include "CyberGear.h"
#include "ICM42688.h"
#include "madwick_1axis.h"
#include "PID.h"
#include "sbus.h"
#include "RobotState.h"

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

// カスケードPIDコントローラーのインスタンス
PID velocityPI(0.03f, 0.1f, 0.0f, 0.0f, -0.5f, 0.5f);   // 入力速度のPI制御（目標速度 - 実測速度）用，出力目標角度制限は±45度（0.785 rad/s）程度を想定
PID posturePD(10.0f, 0.0f, 0.02f, 0.0f, -12.0f, 12.0f); // 姿勢制御用のPDコントローラー（目標角度 - 実測角度）用，直接IMUの角度微分をD項に使用するため不完全微分は不要、出力は速度指令なので±30 rad/s程度を想定
PID yawRatePI(0.005f, 0.0f, 0.0f, 0.0f, -12.0f, 12.0f); // ヨーレイト制御用のPDコントローラー（目標ヨーレイト - 実測ヨーレイト）用，直接IMUの角速度をD項に使用するため不完全微分は不要、出力は速度トルク指令なので±10 Nm程度を想定

// ICM42688センサーのインスタンス
ICM42688 IMU(SPI, IMU_CS_PIN);
Madgwick1Axis madwickFilter(0.3f, 1.0f / 0.9935f); // フィルタゲイン0.3, 加速度スケール補正

// === 調整可能なパラメータ ===
const float INITIAL_WAIT_SEC = 1.0;                                    // 制御開始前の待ち時間（秒）
const unsigned long SPEED_UPDATE_INTERVAL_US = 2500;                   // 速度更新間隔（マイクロ秒）10ms=10000us
const unsigned long STATUS_REQUEST_INTERVAL_US = 2500;                 // ステータス要求間隔（マイクロ秒）10ms=10000us
const float MOTOR_UPDATE_DT = STATUS_REQUEST_INTERVAL_US / 1000000.0f; // モーター更新の周期（秒）
const unsigned long IMU_UPDATE_INTERVAL_US = 1000;                     // IMU読み取り間隔（マイクロ秒）10ms=10000us
const unsigned long PRINT_INTERVAL_US = 100000;                        // データ出力間隔（マイクロ秒）10ms=10000us
const float FORWARD_COMMAND_SCALE = 4.0f;                              // 前後の指令のスケーリング係数,m/s
const float TURN_COMMAND_SCALE = 8.0f * 180.0f / M_PI;                 // 左右の指令のスケーリング係数,deg/s
const float WHEEL_BASE = 0.1772f;                                      // 車輪間距離（m）
const float WHEEL_RADIUS = 0.108f;                                     // 車輪半径（m）
const float TORQUE_CONSTANT = 0.87f;                                   // モーターのトルク定数（Nm/A）※実際のモーターに合わせて調整が必要

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, 6, 7, true);
/* SBUS data */
bfs::SbusData data;

RobotState robotState;

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
}

void loop()
{
  static unsigned long lastSpeedUpdateTime = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastIMUTime = 0;
  static unsigned long lastPrintTime = 0;
  static bool motorEnabled = false;
  static float target_angle = 0.0f;
  static float target_torque = 0.0f;
  static float target_yaw_torque = 0.0f;
  static float forward_command = 0.0f;
  static float turn_command = 0.0f;
  static float forward_velocity_command = 0.0f;
  static float turn_velocity_command = 0.0f;
  static bool enabled_motor_command = false;

  unsigned long currentTime = micros();

  // 指定時間を経過したらモーターを有効化する
  if (!motorEnabled && millis() >= INITIAL_WAIT_SEC * 1000.0)
  {
    cybergear->enable_motor(0);
    cybergear->enable_motor(1);
    motorEnabled = true;
    Serial.println("Motor Enabled!");
  }

  // 指定間隔ごとにsin波/cos波で速度を更新
  if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL_US)
  {
    lastSpeedUpdateTime = currentTime;

    target_angle = velocityPI.update_pi(forward_velocity_command, robotState.getWheelSpeed(), SPEED_UPDATE_INTERVAL_US / 1000000.0f);                                                   // 目標速度は0、実測速度はモーターから取得
    target_torque = posturePD.update_pd_measurement_deriv(target_angle, robotState.getPendulumAngle(), robotState.getPendulumAngularVelocity(), SPEED_UPDATE_INTERVAL_US / 1000000.0f); // 目標角度 - 実測角度を入力、ジャイロの角速度をD項に使用
    target_yaw_torque = yawRatePI.update_pi(turn_velocity_command, robotState.getYawRate(), SPEED_UPDATE_INTERVAL_US / 1000000.0f);                                                     // 目標ヨーレイト - 実測ヨーレイトを入力

    // 目標速度を設定 (最初のN秒間は指令を出さない)
    if (motorEnabled && enabled_motor_command)
    {
      cybergear->set_current(0, -(target_torque + target_yaw_torque) / TORQUE_CONSTANT); // 左右の車輪で逆符号の速度指令を出すことで回転も可能にする
      cybergear->set_current(1, (target_torque - target_yaw_torque) / TORQUE_CONSTANT);  // 左右の車輪で逆符号の速度指令を出すことで回転も可能にする
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

    robotState.updateWheel((cybergear->getSpeed(0) - cybergear->getSpeed(1)) / 2.0f, MOTOR_UPDATE_DT); // 左右の車輪の速度から平均前進速度を計算して更新
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
    forward_command = static_cast<float>(map(data.ch[2], 368, 1680, -1000, 1000)) / 1000.0f; // 前後の指令を-1.0から1.0の範囲にマッピング
    turn_command = static_cast<float>(map(data.ch[0], 368, 1680, -1000, 1000)) / 1000.0f;    // 左右の指令を-1.0から1.0の範囲にマッピング
    enabled_motor_command = (data.ch[4] > 1000);                                             // チャンネル4の値が1000を超えていればモーターを有効化する

    forward_velocity_command = FORWARD_COMMAND_SCALE * forward_command / WHEEL_RADIUS; // 前後の指令を-1.0から1.0の範囲にマッピングしたものを速度指令に変換し，車輪半径で割って角速度指令に変換
    turn_velocity_command = TURN_COMMAND_SCALE * turn_command;                         // 左右の指令を-1.0から1.0の範囲にマッピングしたものを速度指令に変換し，
  }

  if (currentTime - lastPrintTime >= PRINT_INTERVAL_US)
  {
    lastPrintTime = currentTime;
    // 重力加速度から角度を計算する

    // angle, angular_velocity, motor_speed, target_angle, target_torque
    Serial.print(robotState.getPendulumAngle(), 3);
    Serial.print(", ");
    Serial.print(robotState.getPendulumAngularVelocity(), 3);
    Serial.print(", ");
    Serial.print(robotState.getWheelSpeed(), 3);
    Serial.print(", ");
    Serial.print(target_angle, 3);
    Serial.print(", ");
    Serial.print(target_torque, 3);
    Serial.print(", ");
    Serial.print(forward_command);
    Serial.print(", ");
    Serial.print(turn_command);
    Serial.print(", ");
    Serial.print(target_torque);
    Serial.print(", ");
    Serial.print(target_yaw_torque);
    Serial.println();
  }
}

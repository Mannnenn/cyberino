#include <SPI.h>
#include <mcp_can.h>
#include "CyberGear.h"
#include "ICM42688.h"
#include "madwick_1axis.h"
#include "PID.h"

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
uint8_t MASTER_ID = 0x00;  // ESP32側のID (0x00などでOK)

// CyberGearクラスのインスタンス
CyberGear *cybergear;

// カスケードPIDコントローラーのインスタンス
PID velocityPI(0.1f, 0.01f, 0.0f, 0.0f, -0.785f, 0.785f); // 入力速度のPI制御（目標速度 - 実測速度）用，出力目標角度制限は±45度（0.785 rad/s）程度を想定
PID posturePD(20.0f, 0.0f, 0.25f, 0.0f, -30.0f, 30.0f);   // 姿勢制御用のPDコントローラー（目標角度 - 実測角度）用，直接IMUの角度微分をD項に使用するため不完全微分は不要、出力は速度指令なので±30 rad/s程度を想定

// ICM42688センサーのインスタンス
ICM42688 IMU(SPI, IMU_CS_PIN);
Madgwick1Axis madwickFilter(0.3f, 1.0f / 0.9935f); // フィルタゲイン0.3, 加速度スケール補正

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

  // モーターを初期化（速度モード）
  cybergear->init_motor(0, MODE_SPEED);
  delay(100);

  // モーターのゼロ位置を設定
  cybergear->set_mechpos_zero(0);
  delay(100);

  // モーターの速度制限を設定（rad/s）
  cybergear->set_limit_speed(0, 30.0);
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
}

// === 調整可能なパラメータ ===
const float INITIAL_WAIT_SEC = 5.0;                     // 制御開始前の待ち時間（秒）
const unsigned long SPEED_UPDATE_INTERVAL_US = 7500;    // 速度更新間隔（マイクロ秒）10ms=10000us
const unsigned long STATUS_REQUEST_INTERVAL_US = 10000; // ステータス要求間隔（マイクロ秒）10ms=10000us
const unsigned long IMU_UPDATE_INTERVAL_US = 1000;      // IMU読み取り間隔（マイクロ秒）10ms=10000us
const unsigned long LOOP_DELAY_US = 10;                 // loop最後のdelay（マイクロ秒）1ms=1000us
const unsigned long PRINT_INTERVAL_US = 10000;          // データ出力間隔（マイクロ秒）10ms=10000us

void loop()
{
  static unsigned long lastSpeedUpdateTime = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastIMUTime = 0;
  static unsigned long lastPrintTime = 0;
  static bool motorEnabled = false;
  static float target_angle = 0.0f;
  static float target_speed = 0.0f;

  unsigned long currentTime = micros();

  // 指定時間を経過したらモーターを有効化する
  if (!motorEnabled && millis() >= INITIAL_WAIT_SEC * 1000.0)
  {
    cybergear->enable_motor(0);
    motorEnabled = true;
    Serial.println("Motor Enabled!");
  }

  // 受信したCANメッセージを処理
  cybergear->process_can_message();

  // 指定間隔ごとにsin波/cos波で速度を更新
  if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL_US)
  {
    lastSpeedUpdateTime = currentTime;

    target_angle = velocityPI.update_pi(0.0f, cybergear->getSpeed(0), SPEED_UPDATE_INTERVAL_US / 1000000.0f);                                         // 目標速度は0、実測速度はモーターから取得
    target_speed = posturePD.update_pd_measurement_deriv(target_angle, madwickFilter.getAngle(), -IMU.gyrZ(), SPEED_UPDATE_INTERVAL_US / 1000000.0f); // 目標角度 - 実測角度を入力、ジャイロの角速度をD項に使用

    // 目標速度を設定 (最初のN秒間は指令を出さない)
    if (motorEnabled)
    {
      cybergear->set_speed(0, target_speed);
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
  }

  // 指定間隔ごとにIMUデータを読み取って表示
  if (currentTime - lastIMUTime >= IMU_UPDATE_INTERVAL_US)
  {
    lastIMUTime = currentTime;

    // IMUデータを読み取る
    IMU.getAGT();

    // Madgwickフィルタを更新
    madwickFilter.update(-IMU.accX(), -IMU.accY(), -IMU.gyrZ(), currentTime);
  }

  if (currentTime - lastPrintTime >= PRINT_INTERVAL_US)
  {
    lastPrintTime = currentTime;
    // angle, angular_velocity, motor_speed, target_angle, target_speed
    Serial.print(madwickFilter.getAngle(), 3);
    Serial.print(", ");
    Serial.print(-IMU.gyrZ(), 3);
    Serial.print(", ");
    Serial.print(cybergear->getSpeed(0), 3);
    Serial.print(", ");
    Serial.print(target_angle, 3);
    Serial.print(", ");
    Serial.println(target_speed, 3);
  }
}

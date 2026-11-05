#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

struct LauncherConfig {
  uint8_t servoPitchPin  = 9;
  uint8_t servoRollPin   = 10;
  uint8_t motorPin       = 7;
  uint8_t switchPin      = 4;
  uint8_t pauseButtonPin = 3;

  float smoothTime       = 0.05f;
  float leadScale        = 1.0f;
  float kp               = 1.0f;
  float ki               = 0.0f;
  float kd               = 0.03f;
  int   signTop          = 1;
  int   signBottom       = 1;
  bool  swapMotors       = false;
  bool  tiltUsesRoll     = true;
  float targetTop        = 0;
  float targetBottom     = 0;
  bool  levelFromCurrentAim = true;
  float deadbandDeg      = 0.4f;
  float maxCorrectionDeg = 40.0f;
  float iLimit           = 10.0f;
  float maxServoDps      = 500.0f;
  float rateSmoothS      = 0.02f;
  float tiltTauS         = 1.0f;
  float accelTrustMs2    = 0.8f;
  float upTauS           = 0.5f;
  float stillDps         = 0.6f;
  float biasLearn        = 0.5f;
  float yawMinDps        = 0.15f;
  float maxYawDeg        = 40.0f;
  int   minStepUs        = 2;

  bool  zeroAccelOnCalibrate = true;
  float calMaxRangeRadS      = 0.15f;
  unsigned long calSettleMs  = 600;

  uint8_t staleLimit = 40;

  int minAngle   = 40;
  int maxAngle   = 140;
  int neutral    = 90;
  int manualStep = 5;

  unsigned long loopMs          = 5;
  unsigned long shotTimeoutMs   = 6000;
  unsigned long shotGapMs       = 300;
  unsigned long levelSettleMs   = 250;
  unsigned long debounceMs      = 60;
  unsigned long startIgnoreMs   = 150;
  unsigned long switchConfirmMs = 5;
  uint8_t       maxQueue        = 10;
};

enum ServoMode : uint8_t { SERVO_HOLD, SERVO_MANUAL, SERVO_LEVEL };
enum ShotState : uint8_t { SHOT_IDLE, SHOT_RUNNING, SHOT_PAUSED };

class LauncherCore {
 public:
  explicit LauncherCore(const LauncherConfig &cfg = LauncherConfig());

  void begin(Print *logStream = &Serial);
  void update();

  bool start();
  bool toggleImuDebug();
  bool manualMode();
  bool manualMove(int dPitch, int dRoll);
  bool levelMode();
  bool holdMode();
  bool fire(uint8_t count = 1);
  bool togglePause();
  void stopAll();

  int       manualStep() const    { return cfg_.manualStep; }
  bool      started() const       { return started_; }
  ServoMode servoMode() const     { return servoMode_; }
  ShotState shotState() const     { return shotState_; }
  uint8_t   shotsQueued() const   { return shotsQueued_; }
  float     pitchDeg() const      { return pitchEst_; }
  float     rollDeg() const       { return rollEst_; }

 private:
  static void isrSwitch();
  void onSwitchIsr();
  void pollSwitch();

  bool calibrateGyro();
  void seedOrientation();
  bool updateOrientation(float dt);
  void updateLevel(float dt);
  float pidAxis(float target, float angle, float rate, float &iTerm, float dt) const;
  void resetLevelState();
  void captureAim();
  void writeServos(bool force = false);
  void writeServo(Servo &s, float angle, int &lastUs, bool force);
  void releaseServos();
  bool sameAsLast(const sensors_event_t &a, const sensors_event_t &g) const;
  void remember(const sensors_event_t &a, const sensors_event_t &g);
  void checkI2cTimeout();
  void printImuDiag(const sensors_event_t &a, const sensors_event_t &g);

  void startShot();
  void resumeShot();
  void finishSequenceIfDone();
  void resumeLevelNow();
  void handleSwitchEdge();
  void updateShot();
  void checkPauseButton();
  void setMotor(bool on);

  static LauncherCore *instance_;

  LauncherConfig cfg_;
  Print *log_ = nullptr;

  Adafruit_MPU6050 mpu_;
  Servo servoPitch_, servoRoll_;

  bool started_ = false;
  bool mpuReady_ = false;
  ServoMode servoMode_ = SERVO_HOLD;
  bool resumeLevelAfterShots_ = false;
  bool levelSettling_ = false;
  unsigned long settleStart_ = 0;

  float pitchEst_ = 0, rollEst_ = 0;
  float gyroBias_[3] = {0, 0, 0};
  float upX_ = 0, upY_ = 0, upZ_ = 9.81f;
  float yaw_ = 0, yawRateF_ = 0, tiltRateF_ = 0;
  float iTop_ = 0, iBottom_ = 0;
  float outTop_ = 0, outBottom_ = 0;
  float velTop_ = 0, velBottom_ = 0;
  float pitchOff_ = 0, rollOff_ = 0;
  float pitchCmd_ = 90, rollCmd_ = 90;
  float baseTop_ = 90, baseBottom_ = 90;
  float tiltRef_ = 0;
  int lastPitchUs_ = -1, lastRollUs_ = -1;
  unsigned long lastLoopUs_ = 0;

  float lastRaw_[5] = {0, 0, 0, 0, 0};
  uint8_t sameCount_ = 0;
  unsigned long lastReadUs_ = 0;
  uint16_t i2cTimeouts_ = 0;
  bool imuDebug_ = false;
  unsigned long lastDebugMs_ = 0;

  volatile ShotState shotState_ = SHOT_IDLE;
  volatile unsigned long shotStart_ = 0;
  volatile bool switchEdge_ = false;
  volatile unsigned long lastEdgeMs_ = 0;
  bool useIrq_ = false;
  int lastSwitch_ = HIGH;
  unsigned long pressSeenAt_ = 0;

  uint8_t shotsQueued_ = 0;
  bool crankLeftHome_ = false;
  bool gapPending_ = false;
  unsigned long lastShotEnd_ = 0;

  int lastButton_ = HIGH;
  unsigned long lastButtonMs_ = 0;
};

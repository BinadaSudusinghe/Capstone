#include "LauncherCore.h"

LauncherCore *LauncherCore::instance_ = nullptr;

namespace {
class NullPrint : public Print {
 public:
  size_t write(uint8_t) override { return 1; }
};
NullPrint nullPrint;

void readAngles(const sensors_event_t &a, float &pitchAcc, float &rollAcc) {
  pitchAcc = atan2(-a.acceleration.x,
             sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  rollAcc  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
}

float softDeadband(float e, float db) {
  float a = fabs(e);
  if (a <= db) return 0.0f;
  return (e < 0) ? -(a - db) : (a - db);
}

float smoothDamp(float current, float target, float &vel, float st, float maxSpeed, float dt) {
  st = max(st, 0.001f);
  float omega = 2.0f / st;
  float x = omega * dt;
  float expo = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
  float change = current - target;
  float maxChange = maxSpeed * st;
  change = constrain(change, -maxChange, maxChange);
  float wanted = current - change;
  float temp = (vel + omega * change) * dt;
  vel = (vel - omega * temp) * expo;
  float result = wanted + (change + temp) * expo;
  if ((target - current > 0) == (result > target)) {
    result = target;
    vel = 0;
  }
  return result;
}
}

LauncherCore::LauncherCore(const LauncherConfig &cfg) : cfg_(cfg) {
  pitchCmd_ = cfg_.neutral;
  rollCmd_  = cfg_.neutral;
  log_ = &nullPrint;
}

void LauncherCore::begin(Print *logStream) {
  log_ = logStream ? logStream : &nullPrint;
  instance_ = this;

  pinMode(cfg_.motorPin, OUTPUT);
  digitalWrite(cfg_.motorPin, LOW);
  pinMode(cfg_.switchPin, INPUT_PULLUP);
  pinMode(cfg_.pauseButtonPin, INPUT_PULLUP);
  int irq = digitalPinToInterrupt(cfg_.switchPin);
  useIrq_ = (irq != NOT_AN_INTERRUPT);
  if (useIrq_) attachInterrupt(irq, isrSwitch, FALLING);
  lastSwitch_ = digitalRead(cfg_.switchPin);

  Wire.begin();
  Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif
}

void LauncherCore::update() {
  pollSwitch();
  checkPauseButton();
  handleSwitchEdge();
  updateShot();

  if (levelSettling_) {
    if (!resumeLevelAfterShots_) {
      levelSettling_ = false;
    } else if (shotState_ == SHOT_IDLE && shotsQueued_ == 0 &&
               millis() - settleStart_ >= cfg_.levelSettleMs) {
      resumeLevelNow();
    }
  }

  unsigned long now = millis();
  if (imuDebug_ && started_ && shotState_ == SHOT_IDLE && now - lastDebugMs_ >= 250) {
    lastDebugMs_ = now;
    sensors_event_t a, g, temp;
    unsigned long t0 = micros();
    mpu_.getEvent(&a, &g, &temp);
    lastReadUs_ = micros() - t0;
    checkI2cTimeout();
    printImuDiag(a, g);
  }
  unsigned long nowUs = micros();
  if (started_ && servoMode_ == SERVO_LEVEL && nowUs - lastLoopUs_ >= cfg_.loopMs * 1000UL) {
    float dt = (nowUs - lastLoopUs_) * 1e-6f;
    lastLoopUs_ = nowUs;
    if (dt <= 0 || dt > 0.05f) dt = cfg_.loopMs * 1e-3f;
    if (updateOrientation(dt)) updateLevel(dt);
  }
}

bool LauncherCore::start() {
  if (started_) { log_->println(F("Already started.")); return false; }

  servoPitch_.attach(cfg_.servoPitchPin);
  servoRoll_.attach(cfg_.servoRollPin);
  pitchCmd_ = cfg_.neutral;
  rollCmd_  = cfg_.neutral;
  writeServos(true);
  delay(cfg_.calSettleMs);

  if (!mpuReady_) {
    if (!mpu_.begin()) {
      log_->println(F("MPU6050 not found, check SDA pin 20 and SCL pin 21."));
      releaseServos();
      return false;
    }
    Wire.setClock(400000);
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
#ifdef WIRE_HAS_TIMEOUT
    Wire.setWireTimeout(3000, true);
#else
    log_->println(F("Warning: this board package has no Wire timeout (update Arduino AVR Boards to 1.8.3+). An I2C glitch can freeze the sketch."));
#endif
    mpuReady_ = true;
  }
  if (!calibrateGyro()) {
    releaseServos();
    return false;
  }
  started_ = true;
  servoMode_ = SERVO_HOLD;
  log_->println(F("STARTED. m manual, l level, f fire, p pause, h hold, x stop."));
  return true;
}

bool LauncherCore::manualMode() {
  if (!started_) { log_->println(F("Press b first.")); return false; }
  servoMode_ = SERVO_MANUAL;
  resumeLevelAfterShots_ = false;
  log_->println(F("MANUAL: w/s pitch, a/d roll."));
  return true;
}

bool LauncherCore::manualMove(int dPitch, int dRoll) {
  if (!started_) { log_->println(F("Press b first.")); return false; }
  if (servoMode_ != SERVO_MANUAL) { log_->println(F("Press m first.")); return false; }
  pitchCmd_ = constrain(pitchCmd_ + dPitch, cfg_.minAngle, cfg_.maxAngle);
  rollCmd_  = constrain(rollCmd_  + dRoll,  cfg_.minAngle, cfg_.maxAngle);
  writeServos(true);
  log_->print(F("Pitch ")); log_->print((int)pitchCmd_);
  log_->print(F("  Roll ")); log_->println((int)rollCmd_);
  return true;
}

bool LauncherCore::levelMode() {
  if (!started_) { log_->println(F("Press b first.")); return false; }
  if (shotState_ != SHOT_IDLE || shotsQueued_ > 0) {
    log_->println(F("Finish shooting first (or press x)."));
    return false;
  }
  seedOrientation();
  resetLevelState();
  captureAim();
  servoMode_ = SERVO_LEVEL;
  resumeLevelAfterShots_ = false;
  levelSettling_ = false;
  lastLoopUs_ = micros();
  log_->println(F("LEVEL on."));
  return true;
}

bool LauncherCore::holdMode() {
  if (!started_) { log_->println(F("Press b first.")); return false; }
  servoMode_ = SERVO_HOLD;
  resumeLevelAfterShots_ = false;
  levelSettling_ = false;
  log_->println(F("Sensor off, servos holding."));
  return true;
}

bool LauncherCore::toggleImuDebug() {
  if (!started_) { log_->println(F("Press b first.")); return false; }
  imuDebug_ = !imuDebug_;
  lastDebugMs_ = 0;
  log_->println(imuDebug_ ? F("IMU debug on. Press i again to stop.") : F("IMU debug off."));
  return true;
}

bool LauncherCore::fire(uint8_t count) {
  if (!started_) { log_->println(F("Press b first.")); return false; }
  if (count == 0) return false;
  bool accepted = false;
  while (count > 0 && shotsQueued_ < cfg_.maxQueue) {
    shotsQueued_++;
    count--;
    accepted = true;
  }
  if (accepted) {
    log_->print(F("Shot queued, total ")); log_->println(shotsQueued_);
  } else {
    log_->println(F("Queue full."));
  }
  return accepted;
}

bool LauncherCore::togglePause() {
  ShotState s = SHOT_IDLE;
  for (;;) {
    handleSwitchEdge();
    noInterrupts();
    if (switchEdge_) {
      interrupts();
      continue;
    }
    s = shotState_;
    if (s == SHOT_RUNNING) {
      digitalWrite(cfg_.motorPin, LOW);
      shotState_ = SHOT_PAUSED;
    }
    interrupts();
    break;
  }
  if (s == SHOT_RUNNING) {
    log_->println(F("Paused."));
    return true;
  }
  if (s == SHOT_PAUSED) {
    resumeShot();
    return true;
  }
  log_->println(F("Nothing to pause."));
  return false;
}

void LauncherCore::stopAll() {
  noInterrupts();
  shotState_ = SHOT_IDLE;
  switchEdge_ = false;
  interrupts();
  setMotor(false);
  shotsQueued_ = 0;
  gapPending_ = false;
  resumeLevelAfterShots_ = false;
  levelSettling_ = false;
  if (servoMode_ == SERVO_LEVEL) servoMode_ = SERVO_HOLD;
  log_->println(F("STOPPED. Motor off, queue cleared, servos holding."));
}

bool LauncherCore::calibrateGyro() {
  log_->println(F("Calibrating, keep launcher still and level..."));
  const int samples = 200;
  float sumGP = 0, sumGR = 0, sumGZ = 0, sumAP = 0, sumAR = 0;
  float minGP = 1e9f, maxGP = -1e9f, minGR = 1e9f, maxGR = -1e9f, minGZ = 1e9f, maxGZ = -1e9f;
  float sumAX = 0, sumAY = 0, sumAZ = 0, sMag = 0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < 30; i++) { mpu_.getEvent(&a, &g, &temp); delay(5); }
  for (int i = 0; i < samples; i++) {
    mpu_.getEvent(&a, &g, &temp);
    sumAX += a.acceleration.x;  sumAY += a.acceleration.y;  sumAZ += a.acceleration.z;
    sMag += fabs(a.acceleration.x) + fabs(a.acceleration.y) + fabs(a.acceleration.z);
    sumGP += g.gyro.y;
    sumGR += g.gyro.x;
    sumGZ += g.gyro.z;
    float pa, ra;
    readAngles(a, pa, ra);
    sumAP += pa;
    sumAR += ra;
    if (g.gyro.y < minGP) minGP = g.gyro.y;
    if (g.gyro.y > maxGP) maxGP = g.gyro.y;
    if (g.gyro.x < minGR) minGR = g.gyro.x;
    if (g.gyro.x > maxGR) maxGR = g.gyro.x;
    if (g.gyro.z < minGZ) minGZ = g.gyro.z;
    if (g.gyro.z > maxGZ) maxGZ = g.gyro.z;
    delay(5);
  }
  checkI2cTimeout();
  if (i2cTimeouts_ > 0) {
    log_->print(F("Warning: ")); log_->print((unsigned long)i2cTimeouts_);
    log_->println(F(" I2C timeout(s) during calibration. Check SDA and SCL wiring."));
  }
  if (sMag == 0) {
    log_->println(F("Calibration failed: IMU returns all zeros. Check wiring and power, then press b again."));
    return false;
  }
  if ((maxGP - minGP) > cfg_.calMaxRangeRadS || (maxGR - minGR) > cfg_.calMaxRangeRadS ||
      (maxGZ - minGZ) > cfg_.calMaxRangeRadS) {
    log_->println(F("Launcher moved during calibration. Keep it still and press b again."));
    return false;
  }
  gyroBias_[0] = (sumGR / samples) * RAD_TO_DEG;
  gyroBias_[1] = (sumGP / samples) * RAD_TO_DEG;
  gyroBias_[2] = (sumGZ / samples) * RAD_TO_DEG;
  if (cfg_.zeroAccelOnCalibrate) {
    pitchOff_ = sumAP / samples;
    rollOff_  = sumAR / samples;
    log_->print(F("Level reference: pitch ")); log_->print(pitchOff_, 1);
    log_->print(F("  roll ")); log_->println(rollOff_, 1);
    if (fabs(pitchOff_) > 20.0f || fabs(rollOff_) > 20.0f) {
      log_->println(F("Warning: far from level. Is the launcher level and the IMU z axis up?"));
    }
  } else {
    pitchOff_ = 0;
    rollOff_  = 0;
  }
  seedOrientation();
  pitchEst_ = 0;  rollEst_ = 0;
  upX_ = sumAX / samples;  upY_ = sumAY / samples;  upZ_ = sumAZ / samples;
  resetLevelState();
  log_->println(F("Calibration done."));
  return true;
}

void LauncherCore::seedOrientation() {
  sensors_event_t a, g, temp;
  mpu_.getEvent(&a, &g, &temp);
  float pa, ra;
  readAngles(a, pa, ra);
  pitchEst_ = pa - pitchOff_;
  rollEst_  = ra - rollOff_;
  upX_ = a.acceleration.x;  upY_ = a.acceleration.y;  upZ_ = a.acceleration.z;
  remember(a, g);
  sameCount_ = 0;
}

bool LauncherCore::updateOrientation(float dt) {
  sensors_event_t a, g, temp;
  unsigned long t0 = micros();
  mpu_.getEvent(&a, &g, &temp);
  lastReadUs_ = micros() - t0;
  checkI2cTimeout();

  if (sameAsLast(a, g)) {
    if (++sameCount_ >= cfg_.staleLimit) {
      sameCount_ = 0;
      holdMode();
      log_->println(F("IMU not updating, servos holding. Check wiring, then press b if needed."));
      printImuDiag(a, g);
      return false;
    }
  } else {
    sameCount_ = 0;
  }
  remember(a, g);

  float rawG[3] = { g.gyro.x * RAD_TO_DEG, g.gyro.y * RAD_TO_DEG, g.gyro.z * RAD_TO_DEG };
  float w[3] = { rawG[0] - gyroBias_[0], rawG[1] - gyroBias_[1], rawG[2] - gyroBias_[2] };

  float accMag = sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y +
                       a.acceleration.z * a.acceleration.z);
  float wMag = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  if (wMag < cfg_.stillDps && fabsf(accMag - 9.81f) < 1.0f) {
    float kb = constrain(cfg_.biasLearn * dt, 0.0f, 1.0f);
    for (int i = 0; i < 3; i++) { gyroBias_[i] += (rawG[i] - gyroBias_[i]) * kb; w[i] = rawG[i] - gyroBias_[i]; }
  }

  float pitchAcc, rollAcc;
  readAngles(a, pitchAcc, rollAcc);
  pitchAcc -= pitchOff_;
  rollAcc  -= rollOff_;
  float pitchRate = w[1];
  float rollRate  = w[0];
  float kAcc = (fabsf(accMag - 9.81f) < cfg_.accelTrustMs2) ? dt / (cfg_.tiltTauS + dt) : 0.0f;
  float pitchGyro = pitchEst_ + pitchRate * dt;
  float rollGyro  = rollEst_  + rollRate  * dt;
  pitchEst_ = pitchGyro + kAcc * (pitchAcc - pitchGyro);
  rollEst_  = rollGyro  + kAcc * (rollAcc  - rollGyro);

  float ku = 1.0f - expf(-dt / cfg_.upTauS);
  upX_ += (a.acceleration.x - upX_) * ku;
  upY_ += (a.acceleration.y - upY_) * ku;
  upZ_ += (a.acceleration.z - upZ_) * ku;
  float upMag = sqrtf(upX_ * upX_ + upY_ * upY_ + upZ_ * upZ_);
  float yawRate = (upMag > 1.0f) ? (w[0] * upX_ + w[1] * upY_ + w[2] * upZ_) / upMag : 0.0f;
  if (fabsf(yawRate) >= cfg_.yawMinDps) yaw_ += yawRate * dt;
  yaw_ = constrain(yaw_, -cfg_.maxYawDeg, cfg_.maxYawDeg);

  float rateK = 1.0f - expf(-dt / cfg_.rateSmoothS);
  yawRateF_ += (yawRate - yawRateF_) * rateK;
  float tiltRate = cfg_.tiltUsesRoll ? rollRate : pitchRate;
  tiltRateF_ += (tiltRate - tiltRateF_) * rateK;
  return true;
}

void LauncherCore::resetLevelState() {
  yaw_ = 0;  yawRateF_ = 0;  tiltRateF_ = 0;
  iTop_ = 0;  iBottom_ = 0;
  outTop_ = 0;  outBottom_ = 0;
  velTop_ = 0;  velBottom_ = 0;
}

void LauncherCore::captureAim() {
  if (!cfg_.levelFromCurrentAim) {
    baseTop_ = baseBottom_ = cfg_.neutral;
    tiltRef_ = 0;
    return;
  }
  baseTop_    = cfg_.swapMotors ? pitchCmd_ : rollCmd_;
  baseBottom_ = cfg_.swapMotors ? rollCmd_  : pitchCmd_;
  tiltRef_    = cfg_.tiltUsesRoll ? rollEst_ : pitchEst_;
}

float LauncherCore::pidAxis(float target, float angle, float rate, float &iTerm, float dt) const {
  float error = softDeadband(target - angle, cfg_.deadbandDeg);
  iTerm += cfg_.ki * error * dt;
  iTerm = constrain(iTerm, -cfg_.iLimit, cfg_.iLimit);
  float out = cfg_.kp * error + iTerm - (cfg_.kd + cfg_.leadScale * cfg_.kp * cfg_.smoothTime) * rate;
  return constrain(out, -cfg_.maxCorrectionDeg, cfg_.maxCorrectionDeg);
}

void LauncherCore::updateLevel(float dt) {
  float tilt = cfg_.tiltUsesRoll ? rollEst_ : pitchEst_;
  float wantTop    = pidAxis(cfg_.targetTop + tiltRef_, tilt, tiltRateF_, iTop_,    dt);
  float wantBottom = pidAxis(cfg_.targetBottom,         yaw_, yawRateF_,  iBottom_, dt);
  outTop_    = smoothDamp(outTop_,    wantTop,    velTop_,    cfg_.smoothTime, cfg_.maxServoDps, dt);
  outBottom_ = smoothDamp(outBottom_, wantBottom, velBottom_, cfg_.smoothTime, cfg_.maxServoDps, dt);

  float topAngle    = baseTop_    + cfg_.signTop    * outTop_;
  float bottomAngle = baseBottom_ + cfg_.signBottom * outBottom_;
  float topClamped    = constrain(topAngle,    (float)cfg_.minAngle, (float)cfg_.maxAngle);
  float bottomClamped = constrain(bottomAngle, (float)cfg_.minAngle, (float)cfg_.maxAngle);
  if (topClamped != topAngle)       { outTop_    = cfg_.signTop    * (topClamped    - baseTop_);    velTop_    = 0; }
  if (bottomClamped != bottomAngle) { outBottom_ = cfg_.signBottom * (bottomClamped - baseBottom_); velBottom_ = 0; }

  if (cfg_.swapMotors) { pitchCmd_ = topClamped;    rollCmd_ = bottomClamped; }
  else                 { pitchCmd_ = bottomClamped; rollCmd_ = topClamped; }
  writeServos();
}

bool LauncherCore::sameAsLast(const sensors_event_t &a, const sensors_event_t &g) const {
  return a.acceleration.x == lastRaw_[0] && a.acceleration.y == lastRaw_[1] && a.acceleration.z == lastRaw_[2] &&
         g.gyro.x == lastRaw_[3] && g.gyro.y == lastRaw_[4];
}

void LauncherCore::remember(const sensors_event_t &a, const sensors_event_t &g) {
  lastRaw_[0] = a.acceleration.x;
  lastRaw_[1] = a.acceleration.y;
  lastRaw_[2] = a.acceleration.z;
  lastRaw_[3] = g.gyro.x;
  lastRaw_[4] = g.gyro.y;
}

void LauncherCore::checkI2cTimeout() {
#ifdef WIRE_HAS_TIMEOUT
  if (Wire.getWireTimeoutFlag()) {
    if (i2cTimeouts_ < 65535) i2cTimeouts_++;
    Wire.clearWireTimeoutFlag();
  }
#endif
}

void LauncherCore::printImuDiag(const sensors_event_t &a, const sensors_event_t &g) {
  float pa, ra;
  readAngles(a, pa, ra);
  log_->print(F("IMU acc "));
  log_->print(a.acceleration.x, 3); log_->print(F(" ")); log_->print(a.acceleration.y, 3); log_->print(F(" ")); log_->print(a.acceleration.z, 3);
  log_->print(F(" | gyro "));
  log_->print(g.gyro.x, 3); log_->print(F(" ")); log_->print(g.gyro.y, 3); log_->print(F(" ")); log_->print(g.gyro.z, 3);
  log_->print(F(" | tilt p ")); log_->print(pa - pitchOff_, 1);
  log_->print(F(" r ")); log_->print(ra - rollOff_, 1);
  log_->print(F(" | read ")); log_->print(lastReadUs_); log_->print(F(" us"));
#ifdef WIRE_HAS_TIMEOUT
  log_->print(F(" | I2C timeouts ")); log_->println((unsigned long)i2cTimeouts_);
#else
  log_->println(F(" | no Wire timeout support"));
#endif
}

void LauncherCore::writeServos(bool force) {
  pitchCmd_ = constrain(pitchCmd_, (float)cfg_.minAngle, (float)cfg_.maxAngle);
  rollCmd_  = constrain(rollCmd_,  (float)cfg_.minAngle, (float)cfg_.maxAngle);
  writeServo(servoPitch_, pitchCmd_, lastPitchUs_, force);
  writeServo(servoRoll_,  rollCmd_,  lastRollUs_,  force);
}

void LauncherCore::writeServo(Servo &s, float angle, int &lastUs, bool force) {
  int us = (int)(544.0f + angle * (2400.0f - 544.0f) / 180.0f + 0.5f);
  if (force || lastUs < 0 || abs(us - lastUs) >= cfg_.minStepUs) {
    s.writeMicroseconds(us);
    lastUs = us;
  }
}

void LauncherCore::releaseServos() {
  servoPitch_.detach();
  servoRoll_.detach();
}

void LauncherCore::setMotor(bool on) {
  digitalWrite(cfg_.motorPin, on ? HIGH : LOW);
}

void LauncherCore::startShot() {
  if (servoMode_ == SERVO_LEVEL) {
    resumeLevelAfterShots_ = true;
    servoMode_ = SERVO_HOLD;
    log_->println(F("Sensor paused for shooting."));
  }
  levelSettling_ = false;
  gapPending_ = false;
  crankLeftHome_ = false;
  noInterrupts();
  shotStart_ = millis();
  shotState_ = SHOT_RUNNING;
  interrupts();
  setMotor(true);
  log_->println(F("Firing..."));
}

void LauncherCore::resumeShot() {
  noInterrupts();
  shotStart_ = millis();
  shotState_ = SHOT_RUNNING;
  interrupts();
  setMotor(true);
  log_->println(F("Resumed."));
}

void LauncherCore::finishSequenceIfDone() {
  if (shotsQueued_ == 0 && resumeLevelAfterShots_) {
    levelSettling_ = true;
    settleStart_ = millis();
  }
}

void LauncherCore::resumeLevelNow() {
  levelSettling_ = false;
  resumeLevelAfterShots_ = false;
  seedOrientation();
  resetLevelState();
  captureAim();
  servoMode_ = SERVO_LEVEL;
  lastLoopUs_ = micros();
  log_->println(F("LEVEL back on."));
}

void LauncherCore::isrSwitch() {
  if (instance_) instance_->onSwitchIsr();
}

void LauncherCore::onSwitchIsr() {
  unsigned long t = millis();
  if (t - lastEdgeMs_ < cfg_.debounceMs) return;
  if (shotState_ == SHOT_RUNNING) {
    if (t - shotStart_ < cfg_.startIgnoreMs) return;
    digitalWrite(cfg_.motorPin, LOW);
  }
  lastEdgeMs_ = t;
  switchEdge_ = true;
}

void LauncherCore::pollSwitch() {
  if (useIrq_) return;
  int r = digitalRead(cfg_.switchPin);
  if (r == HIGH) {
    pressSeenAt_ = 0;
  } else if (lastSwitch_ == HIGH) {
    pressSeenAt_ = millis();
    if (pressSeenAt_ == 0) pressSeenAt_ = 1;
  } else if (pressSeenAt_ != 0 && millis() - pressSeenAt_ >= cfg_.switchConfirmMs) {
    pressSeenAt_ = 0;
    onSwitchIsr();
  }
  lastSwitch_ = r;
}

void LauncherCore::handleSwitchEdge() {
  if (!switchEdge_) return;
  switchEdge_ = false;

  if (shotState_ == SHOT_RUNNING) {
    if (crankLeftHome_) {
      shotState_ = SHOT_IDLE;
      if (shotsQueued_ > 0) shotsQueued_--;
      lastShotEnd_ = millis();
      gapPending_ = true;
      log_->print(F("Shot done, ")); log_->print(shotsQueued_); log_->println(F(" left."));
      finishSequenceIfDone();
    } else {
      shotState_ = SHOT_PAUSED;
      log_->println(F("Paused by switch. Press switch or p to resume."));
    }
  } else if (shotState_ == SHOT_PAUSED) {
    resumeShot();
  }
}

void LauncherCore::updateShot() {
  if (shotState_ == SHOT_IDLE) {
    if (shotsQueued_ > 0 && (!gapPending_ || millis() - lastShotEnd_ >= cfg_.shotGapMs)) startShot();
    return;
  }
  if (shotState_ == SHOT_RUNNING) {
    if (!crankLeftHome_ && digitalRead(cfg_.switchPin) == HIGH) crankLeftHome_ = true;
    if (millis() - shotStart_ > cfg_.shotTimeoutMs) {
      setMotor(false);
      shotState_ = SHOT_IDLE;
      shotsQueued_ = 0;
      gapPending_ = false;
      log_->println(F("Timed out, check switch wiring or crank. Queue cleared."));
      finishSequenceIfDone();
    }
  }
}

void LauncherCore::checkPauseButton() {
  int r = digitalRead(cfg_.pauseButtonPin);
  if (r != lastButton_ && millis() - lastButtonMs_ > cfg_.debounceMs) {
    lastButtonMs_ = millis();
    lastButton_ = r;
    if (r == LOW && started_) togglePause();
  }
}
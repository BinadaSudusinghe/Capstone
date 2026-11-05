#include "LauncherCore.h"

LauncherCore launcher;

void printKeys() {
  Serial.println(F("b begin | m manual (w/s pitch, a/d roll) | l level | h hold | f fire | p pause | x stop | i imu debug"));
}

void handleKey(char c) {
  if (c >= 'A' && c <= 'Z') c += 32;
  const int step = launcher.manualStep();
    case 'b': launcher.start(); break;
    case 'm': launcher.manualMode(); break;
    case 'w': launcher.manualMove( step, 0); break;
    case 's': launcher.manualMove(-step, 0); break;
    case 'd': launcher.manualMove(0,  step); break;
    case 'a': launcher.manualMove(0, -step); break;
    case 'l': launcher.levelMode(); break;
    case 'h': launcher.holdMode(); break;
    case 'f': launcher.fire(1); break;
    case 'p': launcher.togglePause(); break;
    case 'x': launcher.stopAll(); break;
    case 'i': launcher.toggleImuDebug(); break;
    case '?': printKeys(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  launcher.begin(&Serial);
  Serial.println(F("Ready. Press b to begin, ? for keys."));
}

void loop() {
  while (Serial.available()) handleKey(Serial.read());
  launcher.update();
}

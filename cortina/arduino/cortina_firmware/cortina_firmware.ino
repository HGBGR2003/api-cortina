#include <Arduino.h>
#include <Stepper.h>
#include <SoftwareSerial.h>

const int STEPS_PER_REV = 200;
Stepper motor(STEPS_PER_REV, 8, 10, 9, 11);

const int PIN_LIMIT_OPEN  = 2;
const int PIN_LIMIT_CLOSE = 3;
const int PIN_LDR         = A1;

SoftwareSerial espSerial(6, 7);

const int  STEPS_PER_CYCLE   = 5;
const long LDR_HYSTERESIS_MS = 3000;
const long SYNC_INTERVAL_MS  = 5000;

const char WIFI_SSID[]   = "SUA_REDE_WIFI";
const char WIFI_PASS[]   = "SUA_SENHA_WIFI";
const char SERVER_HOST[] = "192.168.1.100";
const int  SERVER_PORT   = 8080;
const char AUTH_TOKEN[]  = "cortina-secret-token-2025";

enum CurtainState { STOPPED, OPENING, CLOSING, OPENED, CLOSED };
enum OperMode     { MODE_AUTO, MODE_MANUAL };
enum AutoRule     { RULE_STANDARD, RULE_INVERTED };
enum TargetAction { TARGET_OPEN, TARGET_CLOSE, TARGET_STOP };

CurtainState curtainState = STOPPED;
OperMode     operMode     = MODE_AUTO;
AutoRule     autoRule     = RULE_STANDARD;
TargetAction targetAction = TARGET_STOP;
int          ldrThreshold = 700;

bool          ldrAboveThreshold  = false;
bool          ldrConditionStable = false;
unsigned long ldrChangeTime      = 0;

unsigned long lastSyncTime  = 0;
bool          wifiConnected = false;

char usbBuf[128];
int  usbIdx = 0;
char espBuf[128];
int  espIdx = 0;

void motorStop();
void applyTarget();
void handleLimitSwitches(bool limitOpen, bool limitClose);
void handleAutoMode(int ldr);
void parseCommand(const char* cmd);
void readSerialCommands();
bool connectWifi();
void doWifiSync(int ldr, bool lo, bool lc);
bool sendAT(const char* cmd, const char* expect, int timeoutMs);

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  motor.setSpeed(60);

  pinMode(PIN_LIMIT_OPEN,  INPUT_PULLUP);
  pinMode(PIN_LIMIT_CLOSE, INPUT_PULLUP);

  Serial.println(F("=== Cortina Automatizada v1.0 ==="));
  Serial.println(F("Comandos Serial: CMD:M=A,R=S,T=O,TH=700"));
  Serial.println(F("Iniciando conexao Wi-Fi..."));

  wifiConnected = connectWifi();
  if (wifiConnected) {
    Serial.println(F("Wi-Fi OK!"));
  } else {
    Serial.println(F("Wi-Fi FALHOU. Modo offline."));
  }
}

void loop() {
  int  ldr        = analogRead(PIN_LDR);
  bool limitOpen  = (digitalRead(PIN_LIMIT_OPEN)  == LOW);
  bool limitClose = (digitalRead(PIN_LIMIT_CLOSE) == LOW);

  readSerialCommands();
  handleLimitSwitches(limitOpen, limitClose);

  if (operMode == MODE_AUTO) {
    handleAutoMode(ldr);
  }

  applyTarget();

  unsigned long now = millis();
  if (wifiConnected && (now - lastSyncTime >= SYNC_INTERVAL_MS)) {
    lastSyncTime = now;
    doWifiSync(ldr, limitOpen, limitClose);
  }
}

void motorStop() {
  for (int p = 8; p <= 11; p++) digitalWrite(p, LOW);
}

void applyTarget() {
  if (curtainState == OPENING) {
    motor.step(STEPS_PER_CYCLE);
  } else if (curtainState == CLOSING) {
    motor.step(-STEPS_PER_CYCLE);
  }
}

void handleLimitSwitches(bool limitOpen, bool limitClose) {
  if (curtainState == OPENING && limitOpen) {
    motorStop();
    curtainState = OPENED;
    targetAction = TARGET_STOP;
    Serial.println(F("[FIM DE CURSO] Cortina totalmente ABERTA."));
  }
  if (curtainState == CLOSING && limitClose) {
    motorStop();
    curtainState = CLOSED;
    targetAction = TARGET_STOP;
    Serial.println(F("[FIM DE CURSO] Cortina totalmente FECHADA."));
  }
}

void handleAutoMode(int ldr) {
  bool nowAbove = (ldr > ldrThreshold);

  if (nowAbove != ldrAboveThreshold) {
    ldrAboveThreshold  = nowAbove;
    ldrChangeTime      = millis();
    ldrConditionStable = false;
  }

  if (!ldrConditionStable && (millis() - ldrChangeTime >= LDR_HYSTERESIS_MS)) {
    ldrConditionStable = true;

    bool shouldOpen;
    if (autoRule == RULE_STANDARD) {
      shouldOpen = ldrAboveThreshold;
    } else {
      shouldOpen = !ldrAboveThreshold;
    }

    if (shouldOpen && curtainState != OPENED && curtainState != OPENING) {
      curtainState = OPENING;
      Serial.println(F("[AUTO] Iniciando ABERTURA."));
    } else if (!shouldOpen && curtainState != CLOSED && curtainState != CLOSING) {
      curtainState = CLOSING;
      Serial.println(F("[AUTO] Iniciando FECHAMENTO."));
    }
  }
}

void parseCommand(const char* cmd) {
  Serial.print(F("[CMD] Recebido: "));
  Serial.println(cmd);

  const char* p = strstr(cmd, "CMD:");
  if (!p) return;
  p += 4;

  char token[32];
  while (*p) {
    int i = 0;
    while (*p && *p != ',' && i < 31) { token[i++] = *p++; }
    token[i] = '\0';
    if (*p == ',') p++;

    if (strncmp(token, "M=", 2) == 0) {
      operMode = (token[2] == 'A') ? MODE_AUTO : MODE_MANUAL;
    } else if (strncmp(token, "R=", 2) == 0) {
      autoRule = (token[2] == 'S') ? RULE_STANDARD : RULE_INVERTED;
    } else if (strncmp(token, "T=", 2) == 0) {
      char t = token[2];
      if (t == 'O') {
        targetAction = TARGET_OPEN;
        if (curtainState != OPENED && curtainState != OPENING)
          curtainState = OPENING;
      } else if (t == 'C') {
        targetAction = TARGET_CLOSE;
        if (curtainState != CLOSED && curtainState != CLOSING)
          curtainState = CLOSING;
      } else {
        targetAction = TARGET_STOP;
        motorStop();
        curtainState = STOPPED;
      }
    } else if (strncmp(token, "TH=", 3) == 0) {
      ldrThreshold = atoi(token + 3);
    }
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbIdx > 0) {
        usbBuf[usbIdx] = '\0';
        parseCommand(usbBuf);
        usbIdx = 0;
      }
    } else if (usbIdx < 127) {
      usbBuf[usbIdx++] = c;
    }
  }

  while (espSerial.available()) {
    char c = espSerial.read();
    if (c == '\n' || c == '\r') {
      if (espIdx > 0) {
        espBuf[espIdx] = '\0';
        if (strstr(espBuf, "CMD:")) {
          parseCommand(espBuf);
        }
        espIdx = 0;
      }
    } else if (espIdx < 127) {
      espBuf[espIdx++] = c;
    }
  }
}

bool connectWifi() {
  delay(1000);
  sendAT("AT", "OK", 2000);
  sendAT("AT+CWMODE=1", "OK", 2000);

  char cwjap[128];
  snprintf(cwjap, sizeof(cwjap), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);
  return sendAT(cwjap, "OK", 15000);
}

void doWifiSync(int ldr, bool lo, bool lc) {
  const char* stStr;
  switch (curtainState) {
    case OPENING: stStr = "OPENING"; break;
    case CLOSING: stStr = "CLOSING"; break;
    case OPENED:  stStr = "OPENED";  break;
    case CLOSED:  stStr = "CLOSED";  break;
    default:      stStr = "STOPPED"; break;
  }

  char url[200];
  snprintf(url, sizeof(url),
    "GET /api/v1/hw/sync?ldr=%d&lo=%d&lc=%d&st=%s HTTP/1.1\r\nHost: %s:%d\r\nX-Auth-Token: %s\r\nConnection: close\r\n\r\n",
    ldr, lo ? 1 : 0, lc ? 1 : 0, stStr, SERVER_HOST, SERVER_PORT, AUTH_TOKEN);

  char cipstart[80];
  snprintf(cipstart, sizeof(cipstart), "AT+CIPSTART=\"TCP\",\"%s\",%d", SERVER_HOST, SERVER_PORT);
  if (!sendAT(cipstart, "OK", 5000)) {
    Serial.println(F("[WIFI] Falha ao conectar ao servidor."));
    return;
  }

  char cipsend[20];
  snprintf(cipsend, sizeof(cipsend), "AT+CIPSEND=%d", strlen(url));
  if (!sendAT(cipsend, ">", 3000)) return;

  espSerial.print(url);
  delay(500);

  unsigned long t = millis();
  espIdx = 0;
  while (millis() - t < 3000) {
    while (espSerial.available()) {
      char c = espSerial.read();
      if (c == '\n' || c == '\r') {
        if (espIdx > 0) {
          espBuf[espIdx] = '\0';
          if (strstr(espBuf, "CMD:")) {
            parseCommand(espBuf);
            espSerial.print("AT+CIPCLOSE\r\n");
            return;
          }
          espIdx = 0;
        }
      } else if (espIdx < 127) {
        espBuf[espIdx++] = c;
      }
    }
  }
  espSerial.print("AT+CIPCLOSE\r\n");
  Serial.println(F("[WIFI] Sync timeout."));
}

bool sendAT(const char* cmd, const char* expect, int timeoutMs) {
  while (espSerial.available()) espSerial.read();
  espSerial.print(cmd);
  espSerial.print("\r\n");

  unsigned long start = millis();
  char resp[128];
  int  ri = 0;
  while (millis() - start < (unsigned long)timeoutMs) {
    while (espSerial.available() && ri < 127) {
      resp[ri++] = espSerial.read();
    }
    resp[ri] = '\0';
    if (strstr(resp, expect)) return true;
    if (strstr(resp, "ERROR")) return false;
  }
  return false;
}

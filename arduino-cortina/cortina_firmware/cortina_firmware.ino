#include <Arduino.h>
#include <Stepper.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PIN_MOTOR_IN1 = 8;
const uint8_t PIN_MOTOR_IN2 = 9;
const uint8_t PIN_MOTOR_IN3 = 10;
const uint8_t PIN_MOTOR_IN4 = 11;

const uint8_t PIN_LIMIT_OPEN  = 2;
const uint8_t PIN_LIMIT_CLOSE = 3;
const uint8_t PIN_LDR         = A1;

const uint8_t PIN_ESP_RX = 6;
const uint8_t PIN_ESP_TX = 7;

const unsigned long SERIAL_BAUD_RATE = 9600UL;

const int           MOTOR_STEPS_PER_REV = 2048;
const long          MOTOR_SPEED_RPM     = 15;
const unsigned long STEP_INTERVAL_US    = 2500UL;
const int8_t        STEP_DIR_OPEN       = 1;
const int8_t        STEP_DIR_CLOSE      = -1;

const int           DEFAULT_LIGHT_THRESHOLD = 700;
const int           LIGHT_HYSTERESIS_MARGIN = 30;
const unsigned long LIGHT_DEBOUNCE_MS       = 3000UL;
const unsigned long LDR_SAMPLE_INTERVAL_MS  = 50UL;

const char     WIFI_SSID[]  = "SUA_REDE_WIFI";
const char     WIFI_PASS[]  = "SUA_SENHA_WIFI";
const char     API_HOST[]   = "192.168.1.100";
const uint16_t API_PORT     = 8080;
const char     AUTH_TOKEN[] = "cortina-secret-token-2025";

const unsigned long SYNC_INTERVAL_MS     = 3000UL;
const unsigned long ESP_BOOT_PROBE_MS    = 800UL;
const unsigned long ESP_CMD_TIMEOUT_MS   = 2500UL;
const unsigned long ESP_WIFI_TIMEOUT_MS  = 15000UL;
const unsigned long ESP_HTTP_TIMEOUT_MS  = 3500UL;
const unsigned long ESP_IPD_IDLE_MS      = 120UL;
const unsigned long ESP_ERROR_BACKOFF_MS = 5000UL;

const uint8_t USB_RX_BUF_SIZE = 72;
const uint8_t ESP_RX_BUF_SIZE = 128;
const uint8_t HTTP_BUF_SIZE   = 224;

enum CurtainState : uint8_t {
  STATE_STOPPED = 0,
  STATE_OPENING,
  STATE_CLOSING,
  STATE_OPENED,
  STATE_CLOSED
};

enum OperationMode : char {
  MODE_AUTO   = 'A',
  MODE_MANUAL = 'M'
};

enum AutoRule : char {
  RULE_STANDARD = 'S',
  RULE_INVERTED = 'I'
};

enum TargetAction : char {
  TARGET_OPEN  = 'O',
  TARGET_CLOSE = 'C',
  TARGET_STOP  = 'S'
};

enum LightCondition : uint8_t {
  LIGHT_NONE = 0,
  LIGHT_BRIGHT,
  LIGHT_DARK
};

enum EspState : uint8_t {
  ESP_DISABLED = 0,
  ESP_SET_CIPMUX,
  ESP_WAIT_CIPMUX,
  ESP_CHECK_WIFI,
  ESP_WAIT_CHECK_WIFI,
  ESP_CONNECT_WIFI,
  ESP_WAIT_CONNECT_WIFI,
  ESP_IDLE,
  ESP_TCP_START,
  ESP_WAIT_TCP_START,
  ESP_SEND_LEN,
  ESP_WAIT_PROMPT,
  ESP_SEND_DATA,
  ESP_WAIT_SEND_OK,
  ESP_WAIT_RESPONSE,
  ESP_TCP_CLOSE,
  ESP_WAIT_TCP_CLOSE,
  ESP_ERROR_BACKOFF
};

Stepper stepperMotor(MOTOR_STEPS_PER_REV, PIN_MOTOR_IN1, PIN_MOTOR_IN3, PIN_MOTOR_IN2, PIN_MOTOR_IN4);
SoftwareSerial espSerial(PIN_ESP_RX, PIN_ESP_TX);

CurtainState  curtainState   = STATE_STOPPED;
OperationMode operationMode  = MODE_AUTO;
AutoRule      autoRule       = RULE_STANDARD;
TargetAction  targetAction   = TARGET_STOP;
int           lightThreshold = DEFAULT_LIGHT_THRESHOLD;

int  currentLdrValue     = 0;
bool limitOpenTriggered  = false;
bool limitCloseTriggered = false;

LightCondition pendingLightState   = LIGHT_NONE;
LightCondition confirmedLightState = LIGHT_NONE;
unsigned long  lightStateStartMs   = 0UL;
unsigned long  lastLdrSampleMs     = 0UL;

unsigned long lastStepMicros = 0UL;

char    usbRxBuf[USB_RX_BUF_SIZE];
uint8_t usbRxLen = 0;

EspState      espState             = ESP_DISABLED;
char          espRxBuf[ESP_RX_BUF_SIZE];
uint8_t       espRxLen             = 0;
char          httpBuf[HTTP_BUF_SIZE];
unsigned long espStateStartMs      = 0UL;
unsigned long espLastByteMs        = 0UL;
unsigned long lastSyncMs           = 0UL;
bool          espWifiHasAp         = false;
bool          espBackoffAfterClose = false;

void disableMotorCoils();
void setCurtainState(CurtainState newState);
bool isMotorMoving();
const char* stateToStr(CurtainState st);
void updateMotor();
void resetLdrHysteresis();
void readSensors();
void requestOpen();
void requestClose();
void updateControlLogic();
bool parseCommand(const char* line);
void readUsbSerial();
void processUsbLine(char* line);
void printStatus();
void printHelp();
void clearEspRxBuffer();
void drainEspInput();
void enterEspState(EspState nextState);
bool waitEspResponseBlockingInit(const char* expected, unsigned long timeoutMs);
bool initEsp01();
void buildHttpSyncRequest();
void handleEspLine(const char* line);
void pollEspSerialNonBlocking();
void updateEspClient();

void disableMotorCoils() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}

bool isMotorMoving() {
  return (curtainState == STATE_OPENING || curtainState == STATE_CLOSING);
}

const char* stateToStr(CurtainState st) {
  switch (st) {
    case STATE_OPENED:  return "OPENED";
    case STATE_CLOSED:  return "CLOSED";
    case STATE_OPENING: return "OPENING";
    case STATE_CLOSING: return "CLOSING";
    case STATE_STOPPED:
    default:            return "STOPPED";
  }
}

void setCurtainState(CurtainState newState) {
  if (newState == STATE_STOPPED || newState == STATE_OPENED || newState == STATE_CLOSED) {
    disableMotorCoils();
  }

  if (curtainState != newState) {
    curtainState = newState;
    if (isMotorMoving()) {
      lastStepMicros = micros() - STEP_INTERVAL_US;
    }
    Serial.print(F("[MOTOR] Estado -> "));
    Serial.println(stateToStr(curtainState));
  }
}

void updateMotor() {
  if (!isMotorMoving()) {
    return;
  }

  if (curtainState == STATE_OPENING && limitOpenTriggered) {
    setCurtainState(STATE_OPENED);
    targetAction = TARGET_STOP;
    return;
  }
  if (curtainState == STATE_CLOSING && limitCloseTriggered) {
    setCurtainState(STATE_CLOSED);
    targetAction = TARGET_STOP;
    return;
  }

  unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastStepMicros) >= STEP_INTERVAL_US) {
    lastStepMicros = nowUs;
    if (curtainState == STATE_OPENING) {
      stepperMotor.step(STEP_DIR_OPEN);
    } else {
      stepperMotor.step(STEP_DIR_CLOSE);
    }
  }
}

void resetLdrHysteresis() {
  pendingLightState   = LIGHT_NONE;
  confirmedLightState = LIGHT_NONE;
  lightStateStartMs   = millis();
}

void readSensors() {
  limitOpenTriggered  = (digitalRead(PIN_LIMIT_OPEN) == LOW);
  limitCloseTriggered = (digitalRead(PIN_LIMIT_CLOSE) == LOW);

  if (limitOpenTriggered && curtainState == STATE_OPENING) {
    setCurtainState(STATE_OPENED);
    targetAction = TARGET_STOP;
  }
  if (limitCloseTriggered && curtainState == STATE_CLOSING) {
    setCurtainState(STATE_CLOSED);
    targetAction = TARGET_STOP;
  }

  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastLdrSampleMs) >= LDR_SAMPLE_INTERVAL_MS) {
    lastLdrSampleMs = nowMs;
    currentLdrValue = analogRead(PIN_LDR);

    if (operationMode == MODE_AUTO) {
      int brightLimit = lightThreshold + LIGHT_HYSTERESIS_MARGIN;
      int darkLimit   = lightThreshold - LIGHT_HYSTERESIS_MARGIN;

      LightCondition instantLight = LIGHT_NONE;
      if (currentLdrValue >= brightLimit) {
        instantLight = LIGHT_BRIGHT;
      } else if (currentLdrValue <= darkLimit) {
        instantLight = LIGHT_DARK;
      }

      if (instantLight == LIGHT_NONE) {
        pendingLightState = LIGHT_NONE;
      } else if (instantLight != pendingLightState) {
        pendingLightState = instantLight;
        lightStateStartMs = nowMs;
      } else if ((unsigned long)(nowMs - lightStateStartMs) >= LIGHT_DEBOUNCE_MS) {
        confirmedLightState = pendingLightState;
      }
    } else {
      resetLdrHysteresis();
    }
  }
}

void requestOpen() {
  if (limitOpenTriggered) {
    setCurtainState(STATE_OPENED);
    targetAction = TARGET_STOP;
  } else {
    setCurtainState(STATE_OPENING);
  }
}

void requestClose() {
  if (limitCloseTriggered) {
    setCurtainState(STATE_CLOSED);
    targetAction = TARGET_STOP;
  } else {
    setCurtainState(STATE_CLOSING);
  }
}

void updateControlLogic() {
  if (limitOpenTriggered && limitCloseTriggered) {
    if (isMotorMoving()) {
      setCurtainState(STATE_STOPPED);
      targetAction = TARGET_STOP;
    }
    return;
  }

  if (operationMode == MODE_MANUAL) {
    if (targetAction == TARGET_OPEN) {
      requestOpen();
    } else if (targetAction == TARGET_CLOSE) {
      requestClose();
    } else {
      if (isMotorMoving()) {
        setCurtainState(STATE_STOPPED);
      } else if (curtainState == STATE_OPENED && !limitOpenTriggered) {
        setCurtainState(STATE_STOPPED);
      } else if (curtainState == STATE_CLOSED && !limitCloseTriggered) {
        setCurtainState(STATE_STOPPED);
      }
    }
    return;
  }

  if (confirmedLightState == LIGHT_BRIGHT) {
    if (autoRule == RULE_STANDARD) {
      requestOpen();
    } else {
      requestClose();
    }
  } else if (confirmedLightState == LIGHT_DARK) {
    if (autoRule == RULE_STANDARD) {
      requestClose();
    } else {
      requestOpen();
    }
  }
}

bool parseCommand(const char* line) {
  if (line == nullptr) {
    return false;
  }

  const char* p = strstr_P(line, PSTR("CMD:"));
  if (p == nullptr) {
    return false;
  }
  p += 4;

  OperationMode newMode   = operationMode;
  AutoRule      newRule   = autoRule;
  TargetAction  newTarget = targetAction;
  int           newThresh = lightThreshold;
  bool          hasField  = false;

  while (*p != '\0' && *p != '\r' && *p != '\n') {
    while (*p == ',' || *p == ' ' || *p == '\t') {
      p++;
    }
    if (*p == '\0' || *p == '\r' || *p == '\n') {
      break;
    }

    if (strncmp_P(p, PSTR("M="), 2) == 0) {
      p += 2;
      if (*p != '\0' && *p != '\r' && *p != '\n') {
        char val = (char)toupper((unsigned char)*p);
        if (val == 'A') {
          newMode = MODE_AUTO;
          hasField = true;
        } else if (val == 'M') {
          newMode = MODE_MANUAL;
          hasField = true;
        }
        p++;
      }
    } else if (strncmp_P(p, PSTR("R="), 2) == 0) {
      p += 2;
      if (*p != '\0' && *p != '\r' && *p != '\n') {
        char val = (char)toupper((unsigned char)*p);
        if (val == 'S') {
          newRule = RULE_STANDARD;
          hasField = true;
        } else if (val == 'I') {
          newRule = RULE_INVERTED;
          hasField = true;
        }
        p++;
      }
    } else if (strncmp_P(p, PSTR("T="), 2) == 0) {
      p += 2;
      if (*p != '\0' && *p != '\r' && *p != '\n') {
        char val = (char)toupper((unsigned char)*p);
        if (val == 'O') {
          newTarget = TARGET_OPEN;
          hasField = true;
        } else if (val == 'C') {
          newTarget = TARGET_CLOSE;
          hasField = true;
        } else if (val == 'S') {
          newTarget = TARGET_STOP;
          hasField = true;
        }
        p++;
      }
    } else if (strncmp_P(p, PSTR("TH="), 3) == 0) {
      const char* d = p + 3;
      if (isdigit((unsigned char)*d)) {
        long val = 0;
        while (isdigit((unsigned char)*d)) {
          val = val * 10L + (*d - '0');
          if (val > 1023L) {
            val = 1023L;
          }
          d++;
        }
        newThresh = (int)val;
        hasField = true;
      }
      p = d;
    } else {
      while (*p != '\0' && *p != ',' && *p != '\r' && *p != '\n') {
        p++;
      }
    }
  }

  if (!hasField) {
    return false;
  }

  bool autoConfigChanged = (newMode == MODE_AUTO && operationMode != MODE_AUTO) ||
                           (newRule != autoRule) ||
                           (newThresh != lightThreshold);

  operationMode  = newMode;
  autoRule       = newRule;
  targetAction   = newTarget;
  lightThreshold = newThresh;

  if (autoConfigChanged || operationMode == MODE_MANUAL) {
    resetLdrHysteresis();
  }

  Serial.print(F("[CMD] Aplicado -> M="));
  Serial.print((char)operationMode);
  Serial.print(F(" R="));
  Serial.print((char)autoRule);
  Serial.print(F(" T="));
  Serial.print((char)targetAction);
  Serial.print(F(" TH="));
  Serial.println(lightThreshold);

  updateControlLogic();
  return true;
}

void printStatus() {
  Serial.println(F("---------- STATUS CORTINA ----------"));
  Serial.print(F("Estado Motor   : "));
  Serial.println(stateToStr(curtainState));
  Serial.print(F("Modo / Regra   : "));
  Serial.print(operationMode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
  Serial.print(F(" / "));
  Serial.println(autoRule == RULE_STANDARD ? F("STANDARD (Claro abre)") : F("INVERTED (Claro fecha)"));
  Serial.print(F("Alvo Manual    : "));
  Serial.println((char)targetAction);
  Serial.print(F("LDR / Limiar   : "));
  Serial.print(currentLdrValue);
  Serial.print(F(" / "));
  Serial.print(lightThreshold);
  Serial.print(F(" (+/-"));
  Serial.print(LIGHT_HYSTERESIS_MARGIN);
  Serial.println(F(")"));
  Serial.print(F("Luz Confirmada : "));
  if (confirmedLightState == LIGHT_BRIGHT) {
    Serial.println(F("CLARO"));
  } else if (confirmedLightState == LIGHT_DARK) {
    Serial.println(F("ESCURO"));
  } else {
    Serial.println(F("INDEFINIDA/AGUARDANDO"));
  }
  Serial.print(F("Fim Curso OPEN : "));
  Serial.println(limitOpenTriggered ? F("1 (ACIONADO)") : F("0 (LIVRE)"));
  Serial.print(F("Fim Curso CLOSE: "));
  Serial.println(limitCloseTriggered ? F("1 (ACIONADO)") : F("0 (LIVRE)"));
  Serial.print(F("Wi-Fi ESP-01   : "));
  if (espState == ESP_DISABLED) {
    Serial.println(F("MODO LOCAL/BANCADA (ESP-01 ausente)"));
  } else if (espState >= ESP_IDLE) {
    Serial.println(F("CONECTADO"));
  } else {
    Serial.println(F("CONECTANDO..."));
  }
  Serial.println(F("------------------------------------"));
}

void printHelp() {
  Serial.println(F("Comandos de bancada: ABRIR | FECHAR | PARAR | AUTO | INVERTER | STATUS | HELP"));
  Serial.println(F("Protocolo compacto : CMD:M=<A|M>,R=<S|I>,T=<O|C|S>,TH=<0-1023>"));
}

void processUsbLine(char* line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) {
    line[--len] = '\0';
  }
  if (len == 0) {
    return;
  }

  if (strstr_P(line, PSTR("CMD:")) != nullptr) {
    if (!parseCommand(line)) {
      Serial.println(F("[USB] Comando CMD invalido."));
      return;
    }
    printStatus();
    return;
  }

  if (strcasecmp_P(line, PSTR("ABRIR")) == 0) {
    operationMode = MODE_MANUAL;
    targetAction  = TARGET_OPEN;
    resetLdrHysteresis();
    updateControlLogic();
    printStatus();
  } else if (strcasecmp_P(line, PSTR("FECHAR")) == 0) {
    operationMode = MODE_MANUAL;
    targetAction  = TARGET_CLOSE;
    resetLdrHysteresis();
    updateControlLogic();
    printStatus();
  } else if (strcasecmp_P(line, PSTR("PARAR")) == 0) {
    operationMode = MODE_MANUAL;
    targetAction  = TARGET_STOP;
    resetLdrHysteresis();
    if (isMotorMoving()) {
      setCurtainState(STATE_STOPPED);
    } else {
      updateControlLogic();
    }
    printStatus();
  } else if (strcasecmp_P(line, PSTR("AUTO")) == 0) {
    if (operationMode != MODE_AUTO) {
      operationMode = MODE_AUTO;
      resetLdrHysteresis();
    }
    updateControlLogic();
    printStatus();
  } else if (strcasecmp_P(line, PSTR("INVERTER")) == 0) {
    autoRule = (autoRule == RULE_STANDARD) ? RULE_INVERTED : RULE_STANDARD;
    resetLdrHysteresis();
    updateControlLogic();
    printStatus();
  } else if (strcasecmp_P(line, PSTR("STATUS")) == 0) {
    printStatus();
  } else if (strcasecmp_P(line, PSTR("HELP")) == 0 || strcmp_P(line, PSTR("?")) == 0) {
    printHelp();
  } else {
    Serial.print(F("[USB] Comando desconhecido: "));
    Serial.println(line);
    printHelp();
  }
}

void readUsbSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (usbRxLen > 0) {
        usbRxBuf[usbRxLen] = '\0';
        processUsbLine(usbRxBuf);
        usbRxLen = 0;
      }
    } else if (usbRxLen < USB_RX_BUF_SIZE - 1) {
      usbRxBuf[usbRxLen++] = c;
    } else {
      usbRxLen = 0;
      Serial.println(F("[USB] Erro: linha muito longa."));
    }
  }
}

void clearEspRxBuffer() {
  espRxLen = 0;
  espRxBuf[0] = '\0';
}

void drainEspInput() {
  while (espSerial.available() > 0) {
    (void)espSerial.read();
  }
  clearEspRxBuffer();
}

void enterEspState(EspState nextState) {
  espState = nextState;
  espStateStartMs = millis();
}

bool waitEspResponseBlockingInit(const char* expected, unsigned long timeoutMs) {
  clearEspRxBuffer();
  unsigned long startMs = millis();
  while ((unsigned long)(millis() - startMs) < timeoutMs) {
    while (espSerial.available() > 0) {
      char c = (char)espSerial.read();
      if (espRxLen < ESP_RX_BUF_SIZE - 1) {
        espRxBuf[espRxLen++] = c;
        espRxBuf[espRxLen] = '\0';
      } else {
        memmove(espRxBuf, espRxBuf + (ESP_RX_BUF_SIZE / 2), (ESP_RX_BUF_SIZE / 2));
        espRxLen = (ESP_RX_BUF_SIZE / 2) - 1;
        espRxBuf[espRxLen++] = c;
        espRxBuf[espRxLen] = '\0';
      }
      if (strstr(espRxBuf, expected) != nullptr) {
        return true;
      }
    }
  }
  return false;
}

bool initEsp01() {
  drainEspInput();
  espSerial.println(F("AT"));
  if (!waitEspResponseBlockingInit("OK", ESP_BOOT_PROBE_MS)) {
    return false;
  }

  drainEspInput();
  espSerial.println(F("ATE0"));
  waitEspResponseBlockingInit("OK", 400UL);

  drainEspInput();
  espSerial.println(F("AT+CWMODE=1"));
  waitEspResponseBlockingInit("OK", 500UL);

  drainEspInput();
  return true;
}

void buildHttpSyncRequest() {
  uint8_t lo = limitOpenTriggered ? 1 : 0;
  uint8_t lc = limitCloseTriggered ? 1 : 0;
  const char* stStr = stateToStr(curtainState);

  snprintf_P(
    httpBuf,
    HTTP_BUF_SIZE,
    PSTR("GET /api/v1/hw/sync?ldr=%d&lo=%u&lc=%u&st=%s HTTP/1.1\r\n"
         "Host: %s:%u\r\n"
         "X-Auth-Token: %s\r\n"
         "Connection: close\r\n\r\n"),
    currentLdrValue,
    lo,
    lc,
    stStr,
    API_HOST,
    API_PORT,
    AUTH_TOKEN
  );
}

void handleEspLine(const char* line) {
  if (espState == ESP_WAIT_CIPMUX) {
    if (strstr_P(line, PSTR("OK")) != nullptr) {
      enterEspState(ESP_CHECK_WIFI);
    } else if (strstr_P(line, PSTR("ERROR")) != nullptr) {
      enterEspState(ESP_ERROR_BACKOFF);
    }
  } else if (espState == ESP_WAIT_CHECK_WIFI) {
    if (strstr_P(line, PSTR("+CWJAP:")) != nullptr) {
      espWifiHasAp = true;
    } else if (strstr_P(line, PSTR("OK")) != nullptr) {
      if (espWifiHasAp) {
        Serial.println(F("[ESP-01] Wi-Fi ja conectado."));
        lastSyncMs = millis();
        enterEspState(ESP_IDLE);
      } else {
        enterEspState(ESP_CONNECT_WIFI);
      }
    } else if (strstr_P(line, PSTR("ERROR")) != nullptr) {
      enterEspState(ESP_CONNECT_WIFI);
    }
  } else if (espState == ESP_WAIT_CONNECT_WIFI) {
    if (strstr_P(line, PSTR("OK")) != nullptr) {
      Serial.println(F("[ESP-01] Wi-Fi conectado!"));
      lastSyncMs = millis();
      enterEspState(ESP_IDLE);
    } else if (strstr_P(line, PSTR("FAIL")) != nullptr || strstr_P(line, PSTR("ERROR")) != nullptr) {
      Serial.println(F("[ESP-01] Falha ao conectar Wi-Fi."));
      enterEspState(ESP_ERROR_BACKOFF);
    }
  } else if (espState == ESP_WAIT_TCP_START) {
    if (strstr_P(line, PSTR("OK")) != nullptr) {
      enterEspState(ESP_SEND_LEN);
    } else if (strstr_P(line, PSTR("ALREADY CONNECTED")) != nullptr) {
      enterEspState(ESP_SEND_LEN);
    } else if (strstr_P(line, PSTR("ERROR")) != nullptr || strstr_P(line, PSTR("CLOSED")) != nullptr) {
      espBackoffAfterClose = true;
      enterEspState(ESP_TCP_CLOSE);
    }
  } else if (espState == ESP_WAIT_SEND_OK) {
    if (strstr_P(line, PSTR("SEND OK")) != nullptr) {
      enterEspState(ESP_WAIT_RESPONSE);
    } else if (strstr_P(line, PSTR("ERROR")) != nullptr || strstr_P(line, PSTR("SEND FAIL")) != nullptr) {
      espBackoffAfterClose = true;
      enterEspState(ESP_TCP_CLOSE);
    }
  } else if (espState == ESP_WAIT_RESPONSE) {
    if (strstr_P(line, PSTR("CMD:")) != nullptr) {
      parseCommand(line);
      lastSyncMs = millis();
      enterEspState(ESP_IDLE);
    } else if (strstr_P(line, PSTR("CLOSED")) != nullptr) {
      lastSyncMs = millis();
      enterEspState(ESP_IDLE);
    }
  } else if (espState == ESP_WAIT_TCP_CLOSE) {
    if (strstr_P(line, PSTR("OK")) != nullptr || strstr_P(line, PSTR("ERROR")) != nullptr) {
      enterEspState(espBackoffAfterClose ? ESP_ERROR_BACKOFF : ESP_IDLE);
    }
  }
}

void pollEspSerialNonBlocking() {
  uint8_t budget = 32;
  while (espSerial.available() > 0 && budget > 0) {
    budget--;
    char c = (char)espSerial.read();
    espLastByteMs = millis();

    if (c == '\r' || c == '\n') {
      if (espRxLen > 0) {
        espRxBuf[espRxLen] = '\0';
        handleEspLine(espRxBuf);
        clearEspRxBuffer();
      }
      continue;
    }

    if (espRxLen < ESP_RX_BUF_SIZE - 1) {
      espRxBuf[espRxLen++] = c;
      espRxBuf[espRxLen] = '\0';
    } else {
      if (espState == ESP_WAIT_RESPONSE && strstr_P(espRxBuf, PSTR("CMD:")) != nullptr) {
        parseCommand(espRxBuf);
        lastSyncMs = millis();
        enterEspState(ESP_IDLE);
      }
      clearEspRxBuffer();
    }

    if (espState == ESP_WAIT_PROMPT && c == '>') {
      clearEspRxBuffer();
      enterEspState(ESP_SEND_DATA);
      return;
    }

    if (espState == ESP_WAIT_RESPONSE && strstr_P(espRxBuf, PSTR("CLOSED")) != nullptr) {
      handleEspLine(espRxBuf);
      clearEspRxBuffer();
      return;
    }
  }

  if (espState == ESP_WAIT_RESPONSE && espRxLen > 0 &&
      (unsigned long)(millis() - espLastByteMs) >= ESP_IPD_IDLE_MS) {
    if (strstr_P(espRxBuf, PSTR("CMD:")) != nullptr) {
      parseCommand(espRxBuf);
      clearEspRxBuffer();
      lastSyncMs = millis();
      espBackoffAfterClose = false;
      enterEspState(ESP_TCP_CLOSE);
    }
  }
}

void updateEspClient() {
  if (espState == ESP_DISABLED) {
    return;
  }

  pollEspSerialNonBlocking();

  unsigned long nowMs = millis();

  switch (espState) {
    case ESP_SET_CIPMUX:
      drainEspInput();
      espSerial.println(F("AT+CIPMUX=0"));
      enterEspState(ESP_WAIT_CIPMUX);
      break;

    case ESP_WAIT_CIPMUX:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_CMD_TIMEOUT_MS) {
        enterEspState(ESP_ERROR_BACKOFF);
      }
      break;

    case ESP_CHECK_WIFI:
      drainEspInput();
      espWifiHasAp = false;
      espSerial.println(F("AT+CWJAP?"));
      enterEspState(ESP_WAIT_CHECK_WIFI);
      break;

    case ESP_WAIT_CHECK_WIFI:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_CMD_TIMEOUT_MS) {
        enterEspState(ESP_CONNECT_WIFI);
      }
      break;

    case ESP_CONNECT_WIFI:
      drainEspInput();
      Serial.println(F("[ESP-01] Conectando ao Wi-Fi..."));
      snprintf_P(httpBuf, HTTP_BUF_SIZE, PSTR("AT+CWJAP=\"%s\",\"%s\""), WIFI_SSID, WIFI_PASS);
      espSerial.println(httpBuf);
      enterEspState(ESP_WAIT_CONNECT_WIFI);
      break;

    case ESP_WAIT_CONNECT_WIFI:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_WIFI_TIMEOUT_MS) {
        Serial.println(F("[ESP-01] Timeout de Wi-Fi."));
        enterEspState(ESP_ERROR_BACKOFF);
      }
      break;

    case ESP_IDLE:
      if (!isMotorMoving() && (unsigned long)(nowMs - lastSyncMs) >= SYNC_INTERVAL_MS) {
        enterEspState(ESP_TCP_START);
      }
      break;

    case ESP_TCP_START:
      if (isMotorMoving()) {
        enterEspState(ESP_IDLE);
        break;
      }
      drainEspInput();
      snprintf_P(httpBuf, HTTP_BUF_SIZE, PSTR("AT+CIPSTART=\"TCP\",\"%s\",%u"), API_HOST, API_PORT);
      espSerial.println(httpBuf);
      enterEspState(ESP_WAIT_TCP_START);
      break;

    case ESP_WAIT_TCP_START:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_CMD_TIMEOUT_MS) {
        espBackoffAfterClose = true;
        enterEspState(ESP_TCP_CLOSE);
      }
      break;

    case ESP_SEND_LEN:
      buildHttpSyncRequest();
      drainEspInput();
      espSerial.print(F("AT+CIPSEND="));
      espSerial.println(strlen(httpBuf));
      enterEspState(ESP_WAIT_PROMPT);
      break;

    case ESP_WAIT_PROMPT:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_CMD_TIMEOUT_MS) {
        espBackoffAfterClose = true;
        enterEspState(ESP_TCP_CLOSE);
      }
      break;

    case ESP_SEND_DATA:
      clearEspRxBuffer();
      espSerial.print(httpBuf);
      enterEspState(ESP_WAIT_SEND_OK);
      break;

    case ESP_WAIT_SEND_OK:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_CMD_TIMEOUT_MS) {
        espBackoffAfterClose = true;
        enterEspState(ESP_TCP_CLOSE);
      }
      break;

    case ESP_WAIT_RESPONSE:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_HTTP_TIMEOUT_MS) {
        lastSyncMs = nowMs;
        espBackoffAfterClose = true;
        enterEspState(ESP_TCP_CLOSE);
      }
      break;

    case ESP_TCP_CLOSE:
      drainEspInput();
      espSerial.println(F("AT+CIPCLOSE"));
      enterEspState(ESP_WAIT_TCP_CLOSE);
      break;

    case ESP_WAIT_TCP_CLOSE:
      if ((unsigned long)(nowMs - espStateStartMs) >= 1000UL) {
        enterEspState(espBackoffAfterClose ? ESP_ERROR_BACKOFF : ESP_IDLE);
      }
      break;

    case ESP_ERROR_BACKOFF:
      if ((unsigned long)(nowMs - espStateStartMs) >= ESP_ERROR_BACKOFF_MS) {
        lastSyncMs = nowMs;
        enterEspState(ESP_CHECK_WIFI);
      }
      break;

    case ESP_DISABLED:
    default:
      break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  espSerial.begin(SERIAL_BAUD_RATE);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);
  disableMotorCoils();

  pinMode(PIN_LIMIT_OPEN, INPUT_PULLUP);
  pinMode(PIN_LIMIT_CLOSE, INPUT_PULLUP);
  pinMode(PIN_LDR, INPUT);

  stepperMotor.setSpeed(MOTOR_SPEED_RPM);

  limitOpenTriggered  = (digitalRead(PIN_LIMIT_OPEN) == LOW);
  limitCloseTriggered = (digitalRead(PIN_LIMIT_CLOSE) == LOW);
  currentLdrValue     = analogRead(PIN_LDR);

  if (limitOpenTriggered && !limitCloseTriggered) {
    curtainState = STATE_OPENED;
  } else if (limitCloseTriggered && !limitOpenTriggered) {
    curtainState = STATE_CLOSED;
  } else {
    curtainState = STATE_STOPPED;
  }

  Serial.println(F("\n=== CORTINA AUTOMATIZADA - FIRMWARE PRONTO ==="));
  if (initEsp01()) {
    Serial.println(F("[ESP-01] Modulo detectado! Iniciando conexao Wi-Fi..."));
    enterEspState(ESP_SET_CIPMUX);
  } else {
    Serial.println(F("[ESP-01] Modulo nao respondeu ao AT. Operando em modo local/bancada."));
    espState = ESP_DISABLED;
  }

  printHelp();
  printStatus();
}

void loop() {
  readSensors();
  readUsbSerial();
  updateControlLogic();
  updateMotor();
  updateEspClient();
}

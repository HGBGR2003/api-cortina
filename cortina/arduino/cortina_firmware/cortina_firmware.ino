/*
 * cortina_firmware.ino
 * Sistema de Cortina Automatizada - Arduino Uno R3 + ESP-01
 *
 * Pinagem:
 *   Motor de passo (via driver L298N): pinos 8, 9, 10, 11
 *   Fim de curso Aberto  : pino 2 (INPUT_PULLUP)
 *   Fim de curso Fechado : pino 3 (INPUT_PULLUP)
 *   LDR                  : A1 (analógico, 0-1023)
 *   ESP-01 SoftwareSerial: RX=6, TX=7 (3.3V – usar divisor de tensão no TX do Arduino)
 *
 * Protocolo de resposta do servidor (text/plain):
 *   CMD:M=A,R=S,T=O,TH=700
 *   M → Mode   : A=Auto  | M=Manual
 *   R → Rule   : S=Standard | I=Inverted
 *   T → Target : O=Open | C=Close | S=Stop
 *   TH→ Threshold LDR (0-1023)
 */

#include <Stepper.h>
#include <SoftwareSerial.h>

// ─── Configuração do Motor de Passo ───────────────────────────────────────────
const int STEPS_PER_REV = 200;
Stepper motor(STEPS_PER_REV, 8, 10, 9, 11); // sequência correta p/ L298N

// ─── Pinos ────────────────────────────────────────────────────────────────────
const int PIN_LIMIT_OPEN  = 2;   // Fim de curso aberto  (INPUT_PULLUP → LOW = acionado)
const int PIN_LIMIT_CLOSE = 3;   // Fim de curso fechado
const int PIN_LDR         = A1;  // Sensor de luz

// ─── SoftwareSerial ESP-01 ────────────────────────────────────────────────────
SoftwareSerial espSerial(6, 7);  // RX, TX

// ─── Constantes ───────────────────────────────────────────────────────────────
const int   STEPS_PER_CYCLE  = 5;      // Passos por ciclo de loop (não-bloqueante)
const long  LDR_HYSTERESIS_MS = 3000;  // 3 segundos de histerese
const long  SYNC_INTERVAL_MS  = 5000;  // Intervalo de sincronização com o servidor
const int   WIFI_SSID_LEN     = 64;
const int   WIFI_PASS_LEN     = 64;

// ─── Configuração Wi-Fi (editar aqui) ─────────────────────────────────────────
const char WIFI_SSID[]    = "SUA_REDE_WIFI";
const char WIFI_PASS[]    = "SUA_SENHA_WIFI";
const char SERVER_HOST[]  = "192.168.1.100"; // IP do servidor Java
const int  SERVER_PORT    = 8080;
const char AUTH_TOKEN[]   = "cortina-secret-token-2025";

// ─── Estados da Máquina ───────────────────────────────────────────────────────
enum CurtainState { STOPPED, OPENING, CLOSING, OPENED, CLOSED };
enum OperMode     { MODE_AUTO, MODE_MANUAL };
enum AutoRule     { RULE_STANDARD, RULE_INVERTED };
enum TargetAction { TARGET_OPEN, TARGET_CLOSE, TARGET_STOP };

// ─── Variáveis de Estado Global ───────────────────────────────────────────────
CurtainState curtainState = STOPPED;
OperMode     operMode     = MODE_AUTO;
AutoRule     autoRule     = RULE_STANDARD;
TargetAction targetAction = TARGET_STOP;
int          ldrThreshold = 700;

// ─── Histerese LDR ────────────────────────────────────────────────────────────
bool   ldrAboveThreshold     = false;
bool   ldrConditionStable    = false;
unsigned long ldrChangeTime  = 0;

// ─── Sync Wi-Fi ───────────────────────────────────────────────────────────────
unsigned long lastSyncTime = 0;
bool          wifiConnected = false;

// ─── Buffer de resposta AT ────────────────────────────────────────────────────
char  atBuffer[256];
int   atBufIdx = 0;

// ─── Protótipos ───────────────────────────────────────────────────────────────
void     motorStop();
void     motorStep(bool opening);
void     handleLimitSwitches();
void     handleAutoMode(int ldr);
void     applyTarget();
void     parseCommand(const char* cmd);
void     doWifiSync(int ldr, bool lo, bool lc);
bool     sendAT(const char* cmd, const char* expect, int timeoutMs);
void     readSerialCommands();
String   buildSyncUrl(int ldr, bool lo, bool lc);

// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);  // ESP-01 padrão 9600 baud

  motor.setSpeed(60);     // RPM

  pinMode(PIN_LIMIT_OPEN,  INPUT_PULLUP);
  pinMode(PIN_LIMIT_CLOSE, INPUT_PULLUP);

  Serial.println(F("=== Cortina Automatizada v1.0 ==="));
  Serial.println(F("Comandos Serial: CMD:M=A,R=S,T=O,TH=700"));
  Serial.println(F("Iniciando conexao Wi-Fi..."));

  // Conectar ESP-01
  wifiConnected = connectWifi();
  if (wifiConnected) {
    Serial.println(F("Wi-Fi OK!"));
  } else {
    Serial.println(F("Wi-Fi FALHOU. Modo offline."));
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  // 1. Leitura dos sensores
  int  ldr        = analogRead(PIN_LDR);
  bool limitOpen  = (digitalRead(PIN_LIMIT_OPEN)  == LOW);  // PULLUP: LOW = acionado
  bool limitClose = (digitalRead(PIN_LIMIT_CLOSE) == LOW);

  // 2. Verificar comandos Serial (USB e ESP-01)
  readSerialCommands();

  // 3. Travar fins de curso (segurança física)
  handleLimitSwitches(limitOpen, limitClose);

  // 4. Modo AUTO: histerese LDR
  if (operMode == MODE_AUTO) {
    handleAutoMode(ldr);
  }

  // 5. Mover motor (não-bloqueante)
  applyTarget();

  // 6. Sincronização Wi-Fi
  unsigned long now = millis();
  if (wifiConnected && (now - lastSyncTime >= SYNC_INTERVAL_MS)) {
    lastSyncTime = now;
    doWifiSync(ldr, limitOpen, limitClose);
  }
}

// ─── Motor ────────────────────────────────────────────────────────────────────
void motorStop() {
  // Cortar corrente do motor de passo (pinos em LOW)
  for (int p = 8; p <= 11; p++) digitalWrite(p, LOW);
}

void applyTarget() {
  if (curtainState == OPENING) {
    motor.step(STEPS_PER_CYCLE);
  } else if (curtainState == CLOSING) {
    motor.step(-STEPS_PER_CYCLE);
  }
  // STOPPED / OPENED / CLOSED → motor cortado no handleLimitSwitches ou por comando
}

// ─── Fins de Curso ────────────────────────────────────────────────────────────
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

// ─── Modo AUTO com Histerese ─────────────────────────────────────────────────
void handleAutoMode(int ldr) {
  bool nowAbove = (ldr > ldrThreshold);

  // Detecta mudança de condição
  if (nowAbove != ldrAboveThreshold) {
    ldrAboveThreshold = nowAbove;
    ldrChangeTime     = millis();
    ldrConditionStable = false;
  }

  // Aguarda estabilização (histerese de 3 s)
  if (!ldrConditionStable && (millis() - ldrChangeTime >= LDR_HYSTERESIS_MS)) {
    ldrConditionStable = true;

    bool shouldOpen;
    if (autoRule == RULE_STANDARD) {
      shouldOpen = ldrAboveThreshold;  // claro → abre, escuro → fecha
    } else {
      shouldOpen = !ldrAboveThreshold; // invertido
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

// ─── Parser de Comando ────────────────────────────────────────────────────────
/*
 * Formato: CMD:M=A,R=S,T=O,TH=700
 * Aceita tanto do Serial USB quanto do SoftwareSerial (ESP-01)
 */
void parseCommand(const char* cmd) {
  Serial.print(F("[CMD] Recebido: "));
  Serial.println(cmd);

  // Encontrar prefixo CMD:
  const char* p = strstr(cmd, "CMD:");
  if (!p) return;
  p += 4; // pular "CMD:"

  char token[32];
  // Iterar sobre pares chave=valor separados por vírgula
  while (*p) {
    // Extrair até próxima vírgula ou fim
    int i = 0;
    while (*p && *p != ',' && i < 31) { token[i++] = *p++; }
    token[i] = '\0';
    if (*p == ',') p++;

    // Parsear par K=V
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
      } else { // 'S'
        targetAction = TARGET_STOP;
        motorStop();
        curtainState = STOPPED;
      }
    } else if (strncmp(token, "TH=", 3) == 0) {
      ldrThreshold = atoi(token + 3);
    }
  }
}

// ─── Leitura Serial (USB + ESP-01) ───────────────────────────────────────────
char usbBuf[128];
int  usbIdx = 0;
char espBuf[128];
int  espIdx = 0;

void readSerialCommands() {
  // Serial USB
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

  // SoftwareSerial (ESP-01) - procura por "CMD:" no stream de resposta HTTP
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

// ─── Wi-Fi / ESP-01 Comandos AT ───────────────────────────────────────────────
bool connectWifi() {
  delay(1000);
  sendAT("AT", "OK", 2000);
  sendAT("AT+CWMODE=1", "OK", 2000);  // modo station
  
  // Montar comando de conexão
  char cwjap[128];
  snprintf(cwjap, sizeof(cwjap), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);
  return sendAT(cwjap, "OK", 15000);
}

void doWifiSync(int ldr, bool lo, bool lc) {
  // Monta estado atual
  const char* stStr;
  switch (curtainState) {
    case OPENING: stStr = "OPENING"; break;
    case CLOSING: stStr = "CLOSING"; break;
    case OPENED:  stStr = "OPENED";  break;
    case CLOSED:  stStr = "CLOSED";  break;
    default:      stStr = "STOPPED"; break;
  }

  // Monta URL GET
  char url[200];
  snprintf(url, sizeof(url),
    "GET /api/v1/hw/sync?ldr=%d&lo=%d&lc=%d&st=%s HTTP/1.1\r\nHost: %s:%d\r\nX-Auth-Token: %s\r\nConnection: close\r\n\r\n",
    ldr, lo ? 1 : 0, lc ? 1 : 0, stStr, SERVER_HOST, SERVER_PORT, AUTH_TOKEN);

  // Abrir conexão TCP
  char cipstart[80];
  snprintf(cipstart, sizeof(cipstart), "AT+CIPSTART=\"TCP\",\"%s\",%d", SERVER_HOST, SERVER_PORT);
  if (!sendAT(cipstart, "OK", 5000)) {
    Serial.println(F("[WIFI] Falha ao conectar ao servidor."));
    return;
  }

  // Enviar dados
  char cipsend[20];
  snprintf(cipsend, sizeof(cipsend), "AT+CIPSEND=%d", strlen(url));
  if (!sendAT(cipsend, ">", 3000)) return;

  espSerial.print(url);
  delay(500);

  // Ler resposta (procura por "CMD:" na resposta HTTP)
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
  while (espSerial.available()) espSerial.read(); // limpar buffer
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

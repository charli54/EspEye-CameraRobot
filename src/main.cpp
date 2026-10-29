// ===== T-Display S3 : télécommande + récepteur vidéo multi-sources =====
//
// BROCHAGE
//   GPIO 1  : joystick latéral (droite/gauche)      GPIO 10 : bouton 1
//   GPIO 2  : joystick avant/arrière                GPIO 11 : bouton 2 (pelle, plus tard)
//   GPIO 3  : joystick rotation                     GPIO 12 : bouton joystick gauche (armement)
//   GPIO 4  : tension batterie (interne à la carte) GPIO 13 : bouton joystick droit
//   GPIO 0  : BOOT -> source vidéo suivante         GPIO 14 : KEY  -> calibration (maintenu 3 s)
//   GPIO 17 : SCL  (réservé)                        GPIO 44 : LED moteurs armés
//   GPIO 18 : SDA  (réservé)                        GPIO 43 : LED liaison véhicule / batterie faible
//   Boutons câblés vers GND (pull-up interne). Joysticks Hall alimentés en 3,3 V.
//
// VIDÉO : le T-Display crée le Wi-Fi, les sources s'y connectent en TCP (port 5000) :
//   source -> "NOM\n" ; T-Display -> 'F' (demande d'image) ; source -> [taille 4 o][JPEG]
//
// TÉLÉCOMMANDE : paquets UDP 50 Hz vers le véhicule (port 4210), qui s'annonce par "CAR".
//   Armer / désarmer : maintenir le bouton du joystick gauche 2 s (manches au neutre).
//   Calibration : maintenir KEY 3 s (moteurs désarmés), ou KEY appuyé à la mise sous tension.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

// ---------- Broches ----------
#define PIN_POWER_ON   15
#define PIN_BAT_ADC     4
#define PIN_BTN_BOOT    0
#define PIN_BTN_KEY    14
#define PIN_JOY_X       1
#define PIN_JOY_Y       2
#define PIN_JOY_R       3
#define PIN_BTN_1      10
#define PIN_BTN_2      11
#define PIN_BTN_JOY_L  12
#define PIN_BTN_JOY_R  13
#define PIN_I2C_SCL    17   // réservé, non initialisé pour l'instant
#define PIN_I2C_SDA    18
#define PIN_LED_ARM    44
#define PIN_LED_AUX    43

// ---------- Réseau ----------
const char*    AP_SSID    = "TDISPLAY-VIDEO";
const char*    AP_PASS    = "video1234";
const uint16_t VIDEO_PORT = 5000;
const uint16_t CTRL_PORT  = 4210;

// ---------- Réglages ----------
#define MAX_SOURCES 4
const size_t   JPG_MAX          = 100 * 1024;
const uint32_t FRAME_TIMEOUT_MS = 3000;
const uint32_t ARM_HOLD_MS      = 2000;   // appui long pour armer / désarmer
const uint32_t CAL_HOLD_MS      = 3000;   // appui long sur KEY pour calibrer
const bool     JOY_INVERT[3]    = {false, false, false};  // latéral, avant/arrière, rotation
const int      JOY_DEADZONE     = 60;     // sur 1000
const int      CAL_MIN_SPAN     = 300;    // course minimale de chaque côté du neutre (sur 4095)
const int      CENTER_TOL       = 150;    // écart toléré du neutre au démarrage
const int      BAT_LOW_PCT      = 15;
const uint16_t PACKET_MAGIC     = 0xD71E;

// ---------- Paquet de commande (identique côté véhicule) ----------
struct __attribute__((packed)) DrivePacket {
  uint16_t magic;
  uint16_t seq;
  int16_t  vx, vy, w;   // -1000..+1000
  uint8_t  flags;       // bit 0 : moteurs armés
  uint8_t  buttons;     // bit 0 : bouton 1, bit 1 : bouton 2, bit 2 : joystick droit
  uint8_t  checksum;    // XOR des octets précédents
};

// ---------- Objets ----------
TFT_eSPI   tft;
WiFiServer server(VIDEO_PORT, MAX_SOURCES + 1);
WiFiUDP    ctrlUdp;
uint8_t*   jpgBuf = nullptr;

// ---------- État partagé entre la tâche télécommande et l'affichage ----------
volatile bool     armed        = false;
volatile bool     carSeen      = false;
volatile uint32_t lastCarHello = 0;
volatile uint8_t  armProgress  = 0;      // 0..100 pendant l'appui long
volatile uint8_t  msgCode      = 0;
volatile uint32_t msgTime      = 0;
volatile float    batVolt      = 0;
volatile bool     calibrating  = false;
volatile bool     calRequest   = false;

const char* MSG_TEXT[] = {
  "",
  "Manches au neutre pour armer !",
  "Vehicule non connecte",
  "Joysticks non calibres",
  "Liaison perdue : moteurs desarmes",
  "Desarmez avant de calibrer",
};
void setMsg(uint8_t code) { msgCode = code; msgTime = millis(); }

bool carLinked() { return carSeen && millis() - lastCarHello < 3000; }

// ============================================================
//  Boutons (anti-rebond logiciel, lus à 50 Hz)
// ============================================================
struct Button {
  uint8_t pin;
  bool    state = false;
  uint8_t cnt   = 0;
  Button(uint8_t p) : pin(p) {}
  void begin() { pinMode(pin, INPUT_PULLUP); }
  bool update() {                        // true = appuyé (stable sur 2 lectures)
    bool raw = digitalRead(pin) == LOW;
    if (raw != state) { if (++cnt >= 2) { state = raw; cnt = 0; } }
    else cnt = 0;
    return state;
  }
};
Button btn1(PIN_BTN_1), btn2(PIN_BTN_2), btnJoyL(PIN_BTN_JOY_L),
       btnJoyR(PIN_BTN_JOY_R), btnKey(PIN_BTN_KEY);

// ============================================================
//  Batterie (LiPo 1S, pont diviseur 1/2 sur GPIO 4)
// ============================================================
int batPercent(float v) {
  static const float V[] = {3.30, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
                            3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};
  const int N = sizeof(V) / sizeof(V[0]);  // 0 %, 5 %, ... 100 %
  if (v <= V[0]) return 0;
  if (v >= V[N - 1]) return 100;
  for (int i = 1; i < N; i++)
    if (v < V[i]) return (int)(5 * (i - 1) + 5 * (v - V[i - 1]) / (V[i] - V[i - 1]));
  return 100;
}

float readBatteryVolt() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BAT_ADC);
  return mv / 8 * 2 / 1000.0f;
}

bool onUsb() { return batVolt > 4.35f; }

// ============================================================
//  Joysticks : calibration et lecture
// ============================================================
struct AxisCal { int min, center, max; };
const uint8_t JOY_PINS[3]  = {PIN_JOY_X, PIN_JOY_Y, PIN_JOY_R};
const char*   JOY_KEYS[3]  = {"X", "Y", "R"};          // clés en flash
const char*   JOY_LABEL[3] = {"Lat", "Av", "Rot"};     // affichage
AxisCal       joyCal[3];
volatile bool joyOk = false;   // false -> aucune commande de mouvement envoyée

int readAvg(uint8_t pin, int n) {
  long s = 0;
  for (int k = 0; k < n; k++) s += analogRead(pin);
  return s / n;
}

bool calValid(const AxisCal& c) {
  return c.max - c.center > CAL_MIN_SPAN && c.center - c.min > CAL_MIN_SPAN;
}

bool allCalValid() {
  for (auto& c : joyCal) if (!calValid(c)) return false;
  return true;
}

bool loadCal() {
  Preferences p;
  if (!p.begin("joycal", true)) return false;
  bool ok = p.isKey("X_c");
  if (ok) {
    for (int a = 0; a < 3; a++) {
      String k = JOY_KEYS[a];
      joyCal[a].min    = p.getInt((k + "_min").c_str(), 0);
      joyCal[a].center = p.getInt((k + "_c").c_str(), 2048);
      joyCal[a].max    = p.getInt((k + "_max").c_str(), 4095);
    }
  }
  p.end();
  return ok;
}

void saveCal() {
  Preferences p;
  p.begin("joycal", false);
  for (int a = 0; a < 3; a++) {
    String k = JOY_KEYS[a];
    p.putInt((k + "_min").c_str(), joyCal[a].min);
    p.putInt((k + "_c").c_str(),   joyCal[a].center);
    p.putInt((k + "_max").c_str(), joyCal[a].max);
  }
  p.end();
}

// Position d'un axe entre -1000 et +1000 (course asymétrique gérée)
int16_t readAxis(int a) {
  if (!joyOk) return 0;
  const AxisCal& c = joyCal[a];
  int v    = analogRead(JOY_PINS[a]) - c.center;
  int span = (v >= 0 ? c.max - c.center : c.center - c.min) * 97 / 100;
  int out  = v * 1000 / max(1, span);
  if (abs(out) < JOY_DEADZONE) out = 0;
  out = constrain(out, -1000, 1000);
  return JOY_INVERT[a] ? -out : out;
}

bool sticksNeutral() {
  for (int a = 0; a < 3; a++) if (readAxis(a) != 0) return false;
  return true;
}

// ---------- Écrans de calibration ----------
void calHeader(const char* step, const char* l1, const char* l2) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawCentreString("Calibration joysticks", tft.width() / 2, 4, 4);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawCentreString(step, tft.width() / 2, 34, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString(l1, tft.width() / 2, 54, 2);
  if (l2) tft.drawCentreString(l2, tft.width() / 2, 72, 2);
}

// Barre horizontale : plage calibrée (verte si valide), neutre (blanc), position (jaune)
void drawAxisBar(int a, int y, int raw) {
  const int x0 = 50, w = 220, h = 12;
  auto px = [&](int v) { return x0 + (int)((long)constrain(v, 0, 4095) * w / 4095); };
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(JOY_LABEL[a], 8, y - 2, 2);
  tft.drawRect(x0 - 1, y - 1, w + 2, h + 2, TFT_DARKGREY);
  tft.fillRect(x0, y, w, h, TFT_BLACK);
  int r0 = px(joyCal[a].min), r1 = px(joyCal[a].max);
  tft.fillRect(r0, y, max(1, r1 - r0), h, calValid(joyCal[a]) ? TFT_DARKGREEN : TFT_NAVY);
  tft.drawFastVLine(px(joyCal[a].center), y - 2, h + 4, TFT_WHITE);
  tft.fillRect(px(raw) - 2, y, 4, h, TFT_YELLOW);
  tft.setTextPadding(40);
  tft.drawString(String(raw), 276, y - 2, 2);
  tft.setTextPadding(0);
}

// Une tentative complète. Renvoie true si les 3 axes sont valides.
bool runCalibration() {
  // Étape 1 : neutre
  calHeader("Etape 1/2", "Relachez les deux manches", "(position neutre)");
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextPadding(200);
    tft.drawCentreString("Mesure dans " + String(3 - (millis() - t0) / 1000) + " s",
                         tft.width() / 2, 120, 2);
    tft.setTextPadding(0);
    delay(50);
  }

  int mn[3], mx[3];
  long sum[3] = {0, 0, 0};
  for (int a = 0; a < 3; a++) { mn[a] = 4095; mx[a] = 0; }
  for (int k = 0; k < 100; k++) {                 // 0,5 s
    for (int a = 0; a < 3; a++) {
      int v = analogRead(JOY_PINS[a]);
      sum[a] += v; mn[a] = min(mn[a], v); mx[a] = max(mx[a], v);
    }
    delay(5);
  }
  for (int a = 0; a < 3; a++) {
    joyCal[a].center = sum[a] / 100;
    if (mx[a] - mn[a] > CENTER_TOL) {
      calHeader("Erreur", "Un manche a bouge pendant la mesure", nullptr);
      delay(2000);
      return false;
    }
  }

  // Étape 2 : extrémités
  calHeader("Etape 2/2", "Cercles complets en butee", "avec chaque manche");
  for (auto& c : joyCal) c.min = c.max = c.center;
  const uint32_t DURATION = 10000;
  uint32_t lastDraw = 0;
  t0 = millis();
  while (millis() - t0 < DURATION) {
    int raw[3];
    for (int a = 0; a < 3; a++) {
      raw[a] = readAvg(JOY_PINS[a], 4);
      joyCal[a].min = min(joyCal[a].min, raw[a]);
      joyCal[a].max = max(joyCal[a].max, raw[a]);
    }
    if (millis() - lastDraw > 50) {
      lastDraw = millis();
      for (int a = 0; a < 3; a++) drawAxisBar(a, 96 + a * 20, raw[a]);
      bool ok = allCalValid();
      tft.setTextColor(ok ? TFT_GREEN : TFT_CYAN, TFT_BLACK);
      tft.setTextPadding(tft.width());
      tft.drawCentreString(String(ok ? "OK - " : "") + "Reste " +
                           String((DURATION - (millis() - t0)) / 1000 + 1) + " s",
                           tft.width() / 2, 153, 2);
      tft.setTextPadding(0);
    }
    delay(2);
  }
  return allCalValid();
}

// Attend un appui sur KEY (true) ou BOOT (false)
bool waitKeyOrBoot() {
  while (digitalRead(PIN_BTN_KEY) == LOW || digitalRead(PIN_BTN_BOOT) == LOW) delay(10);
  delay(50);
  for (;;) {
    if (digitalRead(PIN_BTN_KEY) == LOW)  { delay(50); while (digitalRead(PIN_BTN_KEY) == LOW)  delay(10); return true; }
    if (digitalRead(PIN_BTN_BOOT) == LOW) { delay(50); while (digitalRead(PIN_BTN_BOOT) == LOW) delay(10); return false; }
    delay(10);
  }
}

// Procédure complète avec possibilité de recommencer ou d'annuler
void calibrationLoop() {
  calibrating = true;
  armed = false;
  AxisCal backup[3] = {joyCal[0], joyCal[1], joyCal[2]};
  bool    backupOk  = joyOk;
  joyOk = false;

  for (;;) {
    if (runCalibration()) {
      saveCal();
      joyOk = true;
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawCentreString("Calibration enregistree", tft.width() / 2, 70, 4);
      delay(1500);
      break;
    }
    calHeader("Calibration ratee", "Course insuffisante sur un axe", "ou joystick mal branche");
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString("KEY : recommencer     BOOT : annuler", tft.width() / 2, 120, 2);
    if (!waitKeyOrBoot()) {               // annulation : on revient à l'ancienne calibration
      for (int a = 0; a < 3; a++) joyCal[a] = backup[a];
      joyOk = backupOk;
      if (!joyOk) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawCentreString("Telecommande desactivee", tft.width() / 2, 70, 4);
        delay(1500);
      }
      break;
    }
  }

  for (int a = 0; a < 3; a++)
    Serial.printf("Joystick %s : min %d / neutre %d / max %d\n",
                  JOY_LABEL[a], joyCal[a].min, joyCal[a].center, joyCal[a].max);
  calibrating = false;
}

// Au démarrage : relit la calibration, remesure le neutre, ou lance la procédure
void setupJoysticks(bool forceCal) {
  analogReadResolution(12);
  delay(50);

  if (!forceCal && loadCal() && allCalValid()) {
    bool warn = false;
    for (int a = 0; a < 3; a++) {
      int c = readAvg(JOY_PINS[a], 64);
      if (abs(c - joyCal[a].center) < CENTER_TOL) joyCal[a].center = c;   // suit la dérive
      else warn = true;                                                  // manche tenu
    }
    joyOk = allCalValid();
    if (warn) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawCentreString("Manche hors neutre au demarrage", tft.width() / 2, 60, 2);
      tft.drawCentreString("Neutre memorise utilise", tft.width() / 2, 85, 2);
      delay(2000);
    }
    if (joyOk) {
      for (int a = 0; a < 3; a++)
        Serial.printf("Joystick %s : min %d / neutre %d / max %d\n",
                      JOY_LABEL[a], joyCal[a].min, joyCal[a].center, joyCal[a].max);
      return;
    }
  }
  calibrationLoop();
}

// ============================================================
//  Tâche télécommande (cœur 0, 50 Hz) : boutons, armement, batterie, LED, UDP
// ============================================================
void controlTask(void*) {
  ctrlUdp.begin(CTRL_PORT);
  IPAddress carIP;
  uint16_t  seq = 0;
  bool      holdActive = false, holdBlocked = false, holdDone = false;
  uint32_t  holdStart = 0, keyStart = 0, lastBat = 0;
  bool      keyFired = false, batInit = false;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    uint32_t now = millis();

    // 1) Annonces "CAR" du véhicule
    int sz;
    while ((sz = ctrlUdp.parsePacket()) > 0) {
      char buf[8] = {0};
      ctrlUdp.read((uint8_t*)buf, sizeof(buf));
      if (sz >= 3 && memcmp(buf, "CAR", 3) == 0) {
        if (!carSeen || carIP != ctrlUdp.remoteIP())
          Serial.printf("Vehicule detecte : %s\n", ctrlUdp.remoteIP().toString().c_str());
        carIP = ctrlUdp.remoteIP();
        carSeen = true;
        lastCarHello = millis();
      }
    }

    // 2) Boutons
    bool b1 = btn1.update(), b2 = btn2.update(), jl = btnJoyL.update(),
         jr = btnJoyR.update(), key = btnKey.update();
    bool linked = carLinked();

    // 3) Sécurités
    if (armed && !linked) { armed = false; setMsg(4); Serial.println("Liaison perdue : desarme"); }
    if (calibrating) armed = false;

    // 4) Armement / désarmement : appui long sur le bouton du joystick gauche
    if (jl && !calibrating) {
      if (!holdActive) {
        holdActive = true; holdStart = now; holdDone = false; holdBlocked = false;
        if (!armed) {
          uint8_t m = !joyOk ? 3 : !linked ? 2 : !sticksNeutral() ? 1 : 0;
          if (m) { setMsg(m); holdBlocked = true; }
        }
      }
      if (!holdBlocked && !holdDone) {
        uint32_t el = now - holdStart;
        if (el >= ARM_HOLD_MS) {
          armed = !armed;
          holdDone = true;
          armProgress = 0;
          Serial.println(armed ? "Moteurs ARMES" : "Moteurs DESARMES");
        } else {
          armProgress = (uint8_t)max<uint32_t>(1, el * 100 / ARM_HOLD_MS);
        }
      }
    } else {
      holdActive = false;
      armProgress = 0;
    }

    // 5) KEY maintenu 3 s -> demande de calibration
    if (key) {
      if (keyStart == 0) keyStart = now;
      if (!keyFired && now - keyStart >= CAL_HOLD_MS) {
        keyFired = true;
        if (armed) setMsg(5);
        else if (!calibrating) calRequest = true;
      }
    } else {
      keyStart = 0;
      keyFired = false;
    }

    // 6) Batterie (filtrée, toutes les 500 ms)
    if (!calibrating && now - lastBat >= 500) {
      lastBat = now;
      float v = readBatteryVolt();
      batVolt = batInit ? batVolt * 0.8f + v * 0.2f : v;
      batInit = true;
    }

    // 7) Envoi au véhicule
    if (linked) {
      bool move = armed && !calibrating;
      DrivePacket p;
      p.magic   = PACKET_MAGIC;
      p.seq     = seq++;
      p.vx      = move ? readAxis(0) : 0;
      p.vy      = move ? readAxis(1) : 0;
      p.w       = move ? readAxis(2) : 0;
      p.flags   = armed ? 0x01 : 0x00;
      p.buttons = (b1 ? 0x01 : 0) | (b2 ? 0x02 : 0) | (jr ? 0x04 : 0);
      uint8_t c = 0;
      const uint8_t* b = (const uint8_t*)&p;
      for (size_t i = 0; i < sizeof(p) - 1; i++) c ^= b[i];
      p.checksum = c;
      ctrlUdp.beginPacket(carIP, CTRL_PORT);
      ctrlUdp.write((const uint8_t*)&p, sizeof(p));
      ctrlUdp.endPacket();
    }

    // 8) LED
    //   LED 44 : éteinte = désarmé, clignote vite = appui long en cours, fixe = armé
    //   LED 43 : fixe = véhicule connecté, clignote = batterie faible
    if (armProgress > 0) digitalWrite(PIN_LED_ARM, (now / 100) % 2);
    else                 digitalWrite(PIN_LED_ARM, armed ? HIGH : LOW);

    bool lowBat = batVolt > 1.0f && !onUsb() && batPercent(batVolt) < BAT_LOW_PCT;
    if (lowBat) digitalWrite(PIN_LED_AUX, (now / 250) % 2);
    else        digitalWrite(PIN_LED_AUX, linked ? HIGH : LOW);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
  }
}

// ============================================================
//  Affichage : bandeaux d'état
// ============================================================
void drawBattery() {
  float v = batVolt;
  String s;
  uint16_t col;
  if (v < 1.0f)   { s = "Bat ?"; col = TFT_DARKGREY; }
  else if (onUsb()) { s = "USB";   col = TFT_CYAN; }
  else {
    int pct = batPercent(v);
    s   = String(pct) + "% " + String(v, 2) + "V";
    col = pct > 50 ? TFT_GREEN : pct > 20 ? TFT_YELLOW : TFT_RED;
  }
  tft.setTextColor(col, TFT_BLACK);
  tft.setTextPadding(100);
  tft.drawRightString(s, tft.width() - 2, 2, 2);
  tft.setTextPadding(0);
}

void drawBottomStatus() {
  const int y = tft.height() - 18;
  uint8_t prog = armProgress;
  if (prog > 0) {                         // barre de progression de l'appui long
    tft.fillRect(0, y - 2, tft.width(), 20, TFT_BLACK);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString(armed ? "Desarmement" : "Armement", 2, y, 2);
    const int bx = 100, bw = 214;
    tft.drawRect(bx, y + 1, bw, 12, TFT_ORANGE);
    tft.fillRect(bx + 1, y + 2, (bw - 2) * prog / 100, 10, TFT_ORANGE);
    return;
  }
  String s;
  uint16_t col;
  if (msgCode && millis() - msgTime < 2500) { s = MSG_TEXT[msgCode];        col = TFT_RED; }
  else if (!joyOk)       { s = "Joysticks non calibres (KEY 3 s)";        col = TFT_ORANGE; }
  else if (!carLinked()) { s = "Vehicule : absent";                       col = TFT_DARKGREY; }
  else if (armed)        { s = "MOTEURS ARMES";                           col = TFT_RED; }
  else                   { s = "Moteurs desarmes (joystick G 2 s)";       col = TFT_GREEN; }
  tft.setTextColor(col, TFT_BLACK);
  tft.setTextPadding(tft.width());
  tft.drawString(s, 2, y, 2);
  tft.setTextPadding(0);
}

// ============================================================
//  Vidéo multi-sources
// ============================================================
struct Source {
  WiFiClient client;
  String     name;
  bool       used    = false;
  bool       pending = false;
  uint32_t   lastReq = 0;
};
Source src[MAX_SOURCES];
int    active = -1;

void selectNext(int from);

bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return false;
  tft.pushImage(x, y, w, h, bitmap);
  return true;
}

void showWaiting() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawCentreString("En attente d'une source video", tft.width() / 2, 36, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString(String("Wi-Fi : ") + AP_SSID, tft.width() / 2, 66, 2);
  tft.drawCentreString(String("Mot de passe : ") + AP_PASS, tft.width() / 2, 86, 2);
  tft.drawCentreString(String("Serveur : 192.168.4.1:") + String(VIDEO_PORT), tft.width() / 2, 106, 2);
}

void drawFrame(uint32_t len) {
  uint16_t w = 0, h = 0;
  if (TJpgDec.getJpgSize(&w, &h, jpgBuf, len) != JDR_OK) return;
  uint8_t scale = 1;
  while (scale < 8 && w / scale > tft.width()) scale *= 2;
  TJpgDec.setJpgScale(scale);
  int16_t x = ((int16_t)tft.width()  - (int16_t)(w / scale)) / 2;
  int16_t y = ((int16_t)tft.height() - (int16_t)(h / scale)) / 2;
  tft.startWrite();
  TJpgDec.drawJpg(x, y, jpgBuf, len);
  tft.endWrite();
}

bool readExact(WiFiClient& c, uint8_t* dst, size_t n, uint32_t timeoutMs = 2000) {
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < n) {
    int avail = c.available();
    if (avail > 0) {
      int r = c.read(dst + got, min((size_t)avail, n - got));
      if (r > 0) { got += r; t0 = millis(); }
    } else {
      if (!c.connected()) return false;
      if (millis() - t0 > timeoutMs) return false;
      delay(1);
    }
  }
  return true;
}

String readLine(WiFiClient& c, uint32_t timeoutMs) {
  String s;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs && s.length() < 32) {
    if (c.available()) {
      char ch = c.read();
      if (ch == '\n') break;
      s += ch;
    } else {
      delay(1);
    }
  }
  s.trim();
  return s;
}

void requestFrame(int i) {
  src[i].client.write((uint8_t)'F');
  src[i].pending = true;
  src[i].lastReq = millis();
}

uint32_t readFrame(int i) {
  uint32_t len = 0;
  if (!readExact(src[i].client, (uint8_t*)&len, 4)) return 0;
  if (len == 0 || len > JPG_MAX) {
    Serial.printf("[%s] taille invalide : %u\n", src[i].name.c_str(), len);
    return 0;
  }
  if (!readExact(src[i].client, jpgBuf, len)) return 0;
  return len;
}

int countSources() {
  int n = 0;
  for (auto& s : src) if (s.used) n++;
  return n;
}

void activate(int i) {
  active = i;
  if (i < 0) { showWaiting(); return; }
  Serial.printf("Source affichee : %s\n", src[i].name.c_str());
  tft.fillScreen(TFT_BLACK);
  requestFrame(i);
}

void selectNext(int from) {
  for (int k = 1; k <= MAX_SOURCES; k++) {
    int j = (from + k + MAX_SOURCES) % MAX_SOURCES;
    if (src[j].used) { activate(j); return; }
  }
  activate(-1);
}

void removeSource(int i) {
  Serial.printf("Source deconnectee : %s\n", src[i].name.c_str());
  src[i].client.stop();
  src[i].used = false;
  src[i].pending = false;
  if (i == active) { active = -1; selectNext(i); }
}

void acceptNewSources() {
  WiFiClient nc = server.available();
  if (!nc) return;
  int slot = -1;
  for (int i = 0; i < MAX_SOURCES; i++) if (!src[i].used) { slot = i; break; }
  if (slot < 0) { Serial.println("Trop de sources, connexion refusee"); nc.stop(); return; }

  nc.setNoDelay(true);
  String name = readLine(nc, 1000);
  if (name.isEmpty()) name = "Source " + String(slot + 1);
  src[slot].client  = nc;
  src[slot].name    = name;
  src[slot].used    = true;
  src[slot].pending = false;
  Serial.printf("Nouvelle source : %s (%s)\n", name.c_str(), nc.remoteIP().toString().c_str());
  if (active < 0) activate(slot);
}

void checkIdleSources() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  for (int i = 0; i < MAX_SOURCES; i++)
    if (src[i].used && i != active && !src[i].client.connected()) removeSource(i);
}

void checkBootButton() {
  static bool last = HIGH;
  static uint32_t t = 0;
  bool now = digitalRead(PIN_BTN_BOOT);
  if (now == LOW && last == HIGH && millis() - t > 250 && active >= 0) {
    t = millis();
    if (src[active].pending) {
      if (!readFrame(active)) { removeSource(active); last = now; return; }
      src[active].pending = false;
    }
    selectNext(active);
  }
  last = now;
}

// ============================================================
//  Programme
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);       // indispensable sur batterie

  pinMode(PIN_LED_ARM, OUTPUT); digitalWrite(PIN_LED_ARM, LOW);
  pinMode(PIN_LED_AUX, OUTPUT); digitalWrite(PIN_LED_AUX, LOW);
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
  btn1.begin(); btn2.begin(); btnJoyL.begin(); btnJoyR.begin(); btnKey.begin();

  tft.init();
  tft.setRotation(1);                     // paysage 320x170
  tft.setSwapBytes(true);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  jpgBuf = (uint8_t*)ps_malloc(JPG_MAX);
  if (!jpgBuf) jpgBuf = (uint8_t*)malloc(JPG_MAX);
  if (!jpgBuf) { tft.drawString("Erreur memoire", 10, 10, 4); while (true) delay(1000); }
  TJpgDec.setCallback(tftOutput);

  // Joysticks (KEY maintenu à la mise sous tension = recalibration)
  setupJoysticks(digitalRead(PIN_BTN_KEY) == LOW);
  batVolt = readBatteryVolt();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, MAX_SOURCES + 2);
  WiFi.setSleep(false);
  server.begin();
  server.setNoDelay(true);
  Serial.printf("Point d'acces \"%s\", IP %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  showWaiting();
  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
  static uint32_t frames = 0, t0 = millis();
  static float fps = 0;

  // Calibration demandée (KEY 3 s)
  if (calRequest) {
    calRequest = false;
    calibrationLoop();
    if (active >= 0) tft.fillScreen(TFT_BLACK);
    else             showWaiting();
  }

  acceptNewSources();
  checkBootButton();
  checkIdleSources();

  if (active < 0) {
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 100) {
      drawBattery();
      drawBottomStatus();
      lastStatus = millis();
    }
    delay(5);
    return;
  }

  Source& s = src[active];
  if (!s.client.connected()) { removeSource(active); return; }
  if (!s.pending) requestFrame(active);

  if (s.client.available() < 4) {
    if (millis() - s.lastReq > FRAME_TIMEOUT_MS) {
      Serial.printf("[%s] ne repond plus\n", s.name.c_str());
      removeSource(active);
    } else {
      delay(1);
    }
    return;
  }

  uint32_t len = readFrame(active);
  s.pending = false;
  if (!len) { removeSource(active); return; }

  requestFrame(active);                   // la source prépare la suivante pendant le décodage
  drawFrame(len);

  frames++;
  if (millis() - t0 >= 1000) {
    fps = frames * 1000.0f / (millis() - t0);
    frames = 0;
    t0 = millis();
  }

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(s.name + " " + String(active + 1) + "/" + String(countSources()) +
                 "  " + String(fps, 1) + " fps", 2, 2, 2);
  drawBattery();
  drawBottomStatus();
}
// Sigma Controls Logger - ESP32
// SoftAP QR: Sigma_Controls / SC15SC15 -> http://192.168.4.1
// SoftAP + Station both always available
// Firmware version + OTA via GitHub Releases

#include <WiFi.h>
#include <WebServer.h>
#include <ModbusMaster.h>
#include <SD.h>
#include <SPI.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <ArduinoJson.h>   // ArduinoJson v7
#include <time.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>
#include "secrets.h"

// =============================================================================
// FIRMWARE VERSION (increase BUILD on every OTA release)
// =============================================================================
#define FIRMWARE_VERSION  "1.1.1"
#define FIRMWARE_BUILD    3

// GitHub version manifest (always on main branch)
static const char* OTA_VERSION_URL =
  "https://raw.githubusercontent.com/snehshah-tech-2004/Sigma-Controls-IntelliConnect/main/version.json";

static const unsigned long OTA_CHECK_INTERVAL_MS = 12UL * 60UL * 60UL * 1000UL;  // 12 hours
static const unsigned long OTA_AFTER_STA_MS      = 45UL * 1000UL;                 // 45s after Station up

bool otaDeferred = false;
bool otaInProgress = false;
String otaLastMessage = "—";
unsigned long lastOtaCheckMs = 0;
unsigned long stationUpSinceMs = 0;
bool otaCheckedAfterSta = false;
String otaPendingUrl = "";  // set when update should run after HTTP response


// =============================================================================
// HARDWARE PIN DEFINITIONS
// =============================================================================
#define RXD2            16
#define TXD2            17
#define SD_CS            4     // Was 5 – GPIO 5 is boot strap; can break WiFi with SD attached
#define RED_LED         25     // ON = COMM / Modbus error
#define BLUE_LED        26     // ON = SD card error  (was YELLOW)
#define BUZZER          27

// LEDC buzzer
#define BUZZER_CHANNEL  0
#define BUZZER_FREQ     2000
#define BUZZER_RES      8

// =============================================================================
// WIFI CREDENTIALS (SoftAP always on as fallback)
// =============================================================================
const char* ap_ssid     = AP_SSID_VALUE;
const char* ap_password = AP_PASSWORD_VALUE;

// =============================================================================
// STATION WIFI – DEFAULTS (used only if user has not saved WiFi via SoftAP form)
// =============================================================================
const char* STA_SSID_DEFAULT = STA_SSID_DEFAULT_VALUE;
const char* STA_PASS_DEFAULT = STA_PASS_DEFAULT_VALUE;
String sta_ssid     = STA_SSID_DEFAULT;
String sta_password = STA_PASS_DEFAULT;
bool   staConnected = false;
bool   softApActive = false;
String lastStaIP    = "";   // last known Station IP (for SoftAP redirect UI)
bool   staCredsFromUser = false;  // true if saved from SoftAP form

bool     useStaticIP = false;       // true only after auto-learn or user save
bool     staticIPLearned = false;   // saved in NVS after first successful join
IPAddress staStaticIP(192, 168, 1, 200);
IPAddress staGateway(192, 168, 1, 1);
IPAddress staSubnet(255, 255, 255, 0);
IPAddress staDns1(8, 8, 8, 8);
IPAddress staDns2(1, 1, 1, 1);

WebServer server(80);
DNSServer dnsServer;
ModbusMaster node;
Preferences prefs;

#define CONFIG_FILE "/config.json"

String deviceId = "";   // auto from MAC

// =============================================================================
// LOGGER TYPES: 0=TempScanner, 1=PID
// =============================================================================
#define MAX_SLAVES 16
const char* loggerTypeName[2] = { "TempScanner", "PID" };

struct SlaveConfig {
  bool     enabled;
  uint8_t  slaveID;
  uint16_t startReg;
  uint8_t  modbusChannels;
  uint8_t  saveChannels;
  uint16_t pvReg;
  uint16_t svReg;
};
SlaveConfig slaves[MAX_SLAVES];

uint8_t  globalLoggerType  = 0;
bool     loggerTypeLocked  = false;
uint8_t  numSlavesConfigured = 4;   // How many slaves the user configured

bool loggingEnabled    = false;
bool sdCardOK          = false;
unsigned long logIntervalMs = 5000;
float divFactor        = 10.0f;
String unit            = "";
unsigned long lastLogMs     = 0;
unsigned long baseMillis    = 0;
String currentLogDate       = "";
bool   loggingWanted        = false;  // true = user started logging (auto-resume after power loss)

String errorMessage         = "";
uint32_t baudRate           = 9600;

// Time system
bool timeSyncedFromNTP = false;
bool timeValid         = false;   // true once we have any reliable time
unsigned long lastNtpAttempt = 0;
#define NTP_RETRY_MS 20000   // retry NTP every 20s until success

// =============================================================================
// NEW: GLOBAL SERIAL COMMUNICATION SETTINGS
// =============================================================================
uint8_t  serialParity    = 0;   // 0=None, 1=Even, 2=Odd
uint8_t  serialStopBits  = 1;   // 1 or 2
uint8_t  serialByteSize  = 8;   // 7 or 8
uint16_t modbusTimeout   = 1000; // milliseconds (stored, used to set node timeout)

String batchBuffer[MAX_SLAVES];
int    batchCount[MAX_SLAVES];

// =============================================================================
// LIVE MONITOR RING BUFFER
// =============================================================================
#define LIVE_BUF_SIZE 10
struct LiveRow {
  char ts[22];
  float ch[24];
};
LiveRow liveBuf[MAX_SLAVES][LIVE_BUF_SIZE];
int     liveHead[MAX_SLAVES];
int     liveCount[MAX_SLAVES];

// =============================================================================
// SOFTWARE RTC — DD-MM-YYYY everywhere
// =============================================================================
struct DateTime { int y, mo, d, h, mi, s; } nowTime;

bool isLeap(int y) {
  return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
}

int daysInMonth(int m, int y) {
  if (m == 2) return isLeap(y) ? 29 : 28;
  if (m == 4 || m == 6 || m == 9 || m == 11) return 30;
  return 31;
}

String getDateFull() {
  char buf[11];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d", nowTime.d, nowTime.mo, nowTime.y);
  return String(buf);
}

String getDateYY() {
  char buf[11];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d", nowTime.d, nowTime.mo, nowTime.y);
  return String(buf);
}

String getTimeSS() {
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", nowTime.h, nowTime.mi, nowTime.s);
  return String(buf);
}

void advanceTime() {
  unsigned long elapsed = (millis() - baseMillis) / 1000;
  baseMillis = millis();
  while (elapsed--) {
    nowTime.s++;
    if (nowTime.s >= 60)  { nowTime.s = 0;  nowTime.mi++; }
    if (nowTime.mi >= 60) { nowTime.mi = 0; nowTime.h++; }
    if (nowTime.h >= 24)  { nowTime.h = 0;  nowTime.d++; }
    if (nowTime.d > daysInMonth(nowTime.mo, nowTime.y)) { nowTime.d = 1; nowTime.mo++; }
    if (nowTime.mo > 12) { nowTime.mo = 1; nowTime.y++; }
  }
}

// =============================================================================
// FILENAMES
// =============================================================================
String getSlaveFilename(int s) {
  String datePart = (currentLogDate.length() > 0) ? currentLogDate : getDateYY();
  return "/" + datePart + "_" +
         String(loggerTypeName[globalLoggerType]) +
         "_ID" + String(slaves[s].slaveID) + ".csv";
}

String getUserLogFilename() {
  String datePart = (currentLogDate.length() > 0) ? currentLogDate : getDateYY();
  return "/" + datePart + "_userlog.txt";
}

// =============================================================================
// SERIAL CONFIG HELPERS
// =============================================================================
// Convert our stored parity value to Arduino SerialConfig
uint32_t getSerialConfig() {
  // byteSize x stopBits x parity
  // We support 8N1, 8E1, 8O1, 8N2, 8E2, 8O2, 7N1, 7E1, 7O1, 7N2, 7E2, 7O2
  if (serialByteSize == 8 && serialStopBits == 1 && serialParity == 0) return SERIAL_8N1;
  if (serialByteSize == 8 && serialStopBits == 1 && serialParity == 1) return SERIAL_8E1;
  if (serialByteSize == 8 && serialStopBits == 1 && serialParity == 2) return SERIAL_8O1;
  if (serialByteSize == 8 && serialStopBits == 2 && serialParity == 0) return SERIAL_8N2;
  if (serialByteSize == 8 && serialStopBits == 2 && serialParity == 1) return SERIAL_8E2;
  if (serialByteSize == 8 && serialStopBits == 2 && serialParity == 2) return SERIAL_8O2;
  if (serialByteSize == 7 && serialStopBits == 1 && serialParity == 0) return SERIAL_7N1;
  if (serialByteSize == 7 && serialStopBits == 1 && serialParity == 1) return SERIAL_7E1;
  if (serialByteSize == 7 && serialStopBits == 1 && serialParity == 2) return SERIAL_7O1;
  if (serialByteSize == 7 && serialStopBits == 2 && serialParity == 0) return SERIAL_7N2;
  if (serialByteSize == 7 && serialStopBits == 2 && serialParity == 1) return SERIAL_7E2;
  if (serialByteSize == 7 && serialStopBits == 2 && serialParity == 2) return SERIAL_7O2;
  // Default fallback
  return SERIAL_8N1;
}

void reinitSerial2() {
  Serial2.end();
  delay(50);
  Serial2.begin(baudRate, getSerialConfig(), RXD2, TXD2);
  delay(100);
}

// =============================================================================
// BUZZER (LEDC – full volume @ 3.3 V)
// =============================================================================
void buzzerInit() {
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZER, BUZZER_CHANNEL);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

void beepTone(int freq, int durationMs) {
  ledcWriteTone(BUZZER_CHANNEL, freq);
  delay(durationMs);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

// Severity-based patterns (harshness increases with level)
// level 1 = soft confirm, 2 = warning, 3 = error, 4 = critical
void playBeepPattern(int level) {
  switch (level) {
    case 1: // soft confirmation (start/stop success)
      beepTone(1800, 80);
      delay(60);
      beepTone(2200, 80);
      break;
    case 2: // warning (single COMM fail)
      beepTone(1400, 120);
      delay(80);
      beepTone(1400, 120);
      break;
    case 3: // error (SD / multi COMM)
      for (int i = 0; i < 3; i++) {
        beepTone(900, 180);
        delay(100);
      }
      break;
    case 4: // critical (reboot imminent)
      for (int i = 0; i < 4; i++) {
        beepTone(700, 250);
        delay(80);
        beepTone(1100, 250);
        delay(80);
      }
      break;
    default:
      beepTone(1500, 100);
      break;
  }
}

// =============================================================================
// TIME SYSTEM – NTP preferred + robust software fallback
// =============================================================================
void saveLastGoodTime() {
  prefs.begin("time", false);
  prefs.putInt("y",  nowTime.y);
  prefs.putInt("mo", nowTime.mo);
  prefs.putInt("d",  nowTime.d);
  prefs.putInt("h",  nowTime.h);
  prefs.putInt("mi", nowTime.mi);
  prefs.putInt("s",  nowTime.s);
  prefs.end();
}

void loadLastGoodTime() {
  prefs.begin("time", true);
  int y = prefs.getInt("y", 0);
  if (y >= 2024 && y <= 2099) {
    nowTime.y  = y;
    nowTime.mo = prefs.getInt("mo", 1);
    nowTime.d  = prefs.getInt("d", 1);
    nowTime.h  = prefs.getInt("h", 0);
    nowTime.mi = prefs.getInt("mi", 0);
    nowTime.s  = prefs.getInt("s", 0);
    timeValid  = true;
    baseMillis = millis();
    Serial.printf("[TIME] Restored last good: %02d-%02d-%04d %02d:%02d:%02d\n",
                  nowTime.d, nowTime.mo, nowTime.y, nowTime.h, nowTime.mi, nowTime.s);
  }
  prefs.end();
}

bool tryNTPSync() {
  if (WiFi.status() != WL_CONNECTED) return false;
  {
    IPAddress testIP;
    if (!WiFi.hostByName("pool.ntp.org", testIP)) {
      Serial.println("[TIME] No internet / DNS failed on Station WiFi");
      Serial.println("[TIME] NTP needs internet. Use dashboard (browser sets time) or a WiFi with internet.");
      return false;
    }
    Serial.printf("[TIME] DNS OK – pool.ntp.org = %s\n", testIP.toString().c_str());
  }

  // IST = UTC + 5:30 = 19800 seconds
  const long GMT_OFFSET_SEC  = 19800;
  const int  DAYLIGHT_OFFSET = 0;

  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET,
             "pool.ntp.org",
             "time.google.com",
             "time.cloudflare.com");

  struct tm timeinfo;
  bool ok = false;
  for (int i = 0; i < 12; i++) {
    if (getLocalTime(&timeinfo, 1000)) {
      if (timeinfo.tm_year + 1900 >= 2024) {
        ok = true;
        break;
      }
    }
    delay(400);
  }

  if (!ok) {
    // IP fallback if hostnames are filtered
    Serial.println("[TIME] Hostname NTP failed – trying public NTP IPs...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, "216.239.35.0", "216.239.35.4", "162.159.200.1");
    for (int i = 0; i < 10; i++) {
      if (getLocalTime(&timeinfo, 1000)) {
        if (timeinfo.tm_year + 1900 >= 2024) {
          ok = true;
          break;
        }
      }
      delay(400);
    }
  }

  if (!ok) {
    Serial.println("[TIME] NTP sync failed (will retry later)");
    return false;
  }

  nowTime.y  = timeinfo.tm_year + 1900;
  nowTime.mo = timeinfo.tm_mon + 1;
  nowTime.d  = timeinfo.tm_mday;
  nowTime.h  = timeinfo.tm_hour;
  nowTime.mi = timeinfo.tm_min;
  nowTime.s  = timeinfo.tm_sec;
  baseMillis = millis();
  timeSyncedFromNTP = true;
  timeValid = true;
  saveLastGoodTime();
  Serial.printf("[TIME] NTP OK (IST): %02d-%02d-%04d %02d:%02d:%02d\n",
                nowTime.d, nowTime.mo, nowTime.y, nowTime.h, nowTime.mi, nowTime.s);
  return true;
}

void maintainTime() {
  // When NTP has synced, read time from ESP32 system clock (keeps accurate IST)
  // Otherwise fall back to software millis clock
  if (timeSyncedFromNTP) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
      nowTime.y  = timeinfo.tm_year + 1900;
      nowTime.mo = timeinfo.tm_mon + 1;
      nowTime.d  = timeinfo.tm_mday;
      nowTime.h  = timeinfo.tm_hour;
      nowTime.mi = timeinfo.tm_min;
      nowTime.s  = timeinfo.tm_sec;
      baseMillis = millis();
      timeValid = true;
    } else {
      // brief failure – keep advancing in software
      advanceTime();
    }
  } else {
    advanceTime();
  }

  // Periodic NTP retry / re-sync when station is connected
  if (staConnected && (millis() - lastNtpAttempt > NTP_RETRY_MS)) {
    lastNtpAttempt = millis();
    if (!timeSyncedFromNTP) {
      tryNTPSync();
    } else {
      // Re-sync every hour to correct drift
      static unsigned long lastHourly = 0;
      if (millis() - lastHourly > 3600000UL) {
        lastHourly = millis();
        tryNTPSync();
      }
    }
  }

  // Persist every ~5 minutes if valid
  static unsigned long lastSave = 0;
  if (timeValid && millis() - lastSave > 300000UL) {
    lastSave = millis();
    saveLastGoodTime();
  }
}

// =============================================================================
// ERROR HANDLERS
// =============================================================================
void errorSD(const char* msg) {
  Serial.printf("[SD ERROR] %s\n", msg);
  errorMessage = String(msg) + " - SD error, rebooting in 30s...";
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  playBeepPattern(4);          // critical harsh pattern once
  unsigned long start = millis();
  while (millis() - start < 30000) {
    server.handleClient();
    // occasional reminder beep
    if ((millis() - start) % 5000 < 50) playBeepPattern(3);
    delay(20);
  }
  delay(200);
  ESP.restart();
}

void errorCOMM(const char* msg) {
  Serial.printf("[COMM ERROR] %s\n", msg);
  errorMessage = String(msg) + " - COMM error, rebooting in 30s...";
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);
  playBeepPattern(4);
  unsigned long start = millis();
  while (millis() - start < 30000) {
    server.handleClient();
    if ((millis() - start) % 5000 < 50) playBeepPattern(3);
    delay(20);
  }
  delay(200);
  ESP.restart();
}

#define MAX_COMM_FAILS 3
int commFailCounts[MAX_SLAVES];

void warnCOMM(int slaveIdx, const char* msg) {
  // One bad slave must NOT stop logging or reboot the device.
  // Working slaves keep writing CSV; UI shows which ID failed.
  Serial.print("[COMM WARN] Slave ");
  Serial.print(slaveIdx);
  Serial.print(": ");
  Serial.print(msg);
  Serial.print("  fail#");
  Serial.println(commFailCounts[slaveIdx] + 1);
  commFailCounts[slaveIdx]++;
  char buf[96];
  snprintf(buf, sizeof(buf), "Slave ID %d offline / Modbus fail (others still logging)",
           slaves[slaveIdx].slaveID);
  errorMessage = String(buf);
  digitalWrite(RED_LED, HIGH);
  // Beep only on first fail of a streak (avoid noise every interval)
  if (commFailCounts[slaveIdx] == 1) playBeepPattern(2);
  // No errorCOMM() / reboot — recover when slave responds again
}

// Forward declarations
void logUserAction(const String& action);

// =============================================================================
// STATION WIFI HELPERS
// =============================================================================
void indicateSoftAP() {
  // SoftAP only / not on station WiFi – longer blue indication
  digitalWrite(RED_LED, LOW);
  for (int i = 0; i < 4; i++) {
    digitalWrite(BLUE_LED, HIGH);
    beepTone(1200, 120);
    delay(80);
    digitalWrite(BLUE_LED, LOW);
    delay(200);
  }
  digitalWrite(BLUE_LED, HIGH);  // leave blue on while SoftAP-only
}

void indicateStationOK() {
  // Station WiFi connected – longer green-style success (both LEDs + rising beeps)
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  for (int i = 0; i < 3; i++) {
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
    beepTone(1600 + i * 300, 150);
    delay(100);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);
    delay(150);
  }
  // Final long confirmation
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  beepTone(2400, 400);
  delay(200);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}

void indicateStationFail() {
  // Station failed – harsh long pattern, then SoftAP indication
  digitalWrite(BLUE_LED, LOW);
  for (int i = 0; i < 5; i++) {
    digitalWrite(RED_LED, HIGH);
    beepTone(700, 200);
    delay(100);
    digitalWrite(RED_LED, LOW);
    delay(150);
  }
}

void saveLastStaIP() {
  if (WiFi.status() != WL_CONNECTED) return;
  lastStaIP = WiFi.localIP().toString();
  prefs.begin("wifi", false);
  prefs.putString("lastStaIP", lastStaIP);
  prefs.end();
}

void loadLastStaIP() {
  prefs.begin("wifi", true);
  lastStaIP = prefs.getString("lastStaIP", "");
  prefs.end();
}
void loadStationCreds() {
  prefs.begin("wifi", true);
  String s = prefs.getString("ssid", "");
  String p = prefs.getString("pass", "");
  useStaticIP = prefs.getBool("static", false);
  staticIPLearned = prefs.getBool("learned", false);
  String ip = prefs.getString("ip", "192.168.1.200");
  String gw = prefs.getString("gw", "192.168.1.1");
  String sn = prefs.getString("sn", "255.255.255.0");
  prefs.end();

  if (s.length() > 0) {
    sta_ssid = s;
    sta_password = p;
    staCredsFromUser = true;
    Serial.printf("[WIFI] Using saved Station SSID: \"%s\"\n", sta_ssid.c_str());
  } else {
    sta_ssid = STA_SSID_DEFAULT;
    sta_password = STA_PASS_DEFAULT;
    staCredsFromUser = false;
    Serial.printf("[WIFI] Using default Station SSID: \"%s\"\n", sta_ssid.c_str());
  }
  staStaticIP.fromString(ip);
  staGateway.fromString(gw);
  staSubnet.fromString(sn);
  Serial.printf("[WIFI] IP mode: %s  %s  gw %s\n",
                useStaticIP ? "STATIC" : "DHCP",
                staStaticIP.toString().c_str(),
                staGateway.toString().c_str());
}

void saveStationCreds(const String& ssid, const String& pass) {
  sta_ssid = ssid;
  sta_password = pass;
  staCredsFromUser = true;
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putBool("static", useStaticIP);
  prefs.putBool("learned", staticIPLearned);
  prefs.putString("ip", staStaticIP.toString());
  prefs.putString("gw", staGateway.toString());
  prefs.putString("sn", staSubnet.toString());
  prefs.end();
  Serial.printf("[WIFI] Saved SSID=\"%s\" static=%d ip=%s\n",
                ssid.c_str(), useStaticIP, staStaticIP.toString().c_str());
}

void applyStationIPConfig() {
  if (useStaticIP && staticIPLearned) {
    if (!WiFi.config(staStaticIP, staGateway, staSubnet, staDns1, staDns2)) {
      Serial.println("[WIFI] Static IP config failed");
    } else {
      Serial.printf("[WIFI] Static IP applied: %s\n", staStaticIP.toString().c_str());
    }
  } else {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    Serial.println("[WIFI] DHCP mode (auto-lock to x.x.x.200 after join)");
  }
}

void adoptStaticFromLease() {
  IPAddress gw = WiFi.gatewayIP();
  IPAddress sn = WiFi.subnetMask();
  IPAddress cur = WiFi.localIP();
  if (gw[0] == 0 && gw[1] == 0) {
    Serial.println("[WIFI] No gateway - skip static adopt");
    return;
  }
  IPAddress fixed(gw[0], gw[1], gw[2], 200);
  if (fixed == gw) fixed[3] = 201;

  if (staticIPLearned && useStaticIP && cur == staStaticIP) {
    Serial.print("[WIFI] Fixed IP active now: ");
    Serial.println(cur);
    return;
  }

  if (staticIPLearned && useStaticIP && staStaticIP == fixed && cur != fixed) {
    Serial.print("[WIFI] Locked IP is ");
    Serial.print(staStaticIP);
    Serial.print(" but current is ");
    Serial.println(cur);
    return;
  }

  staStaticIP = fixed;
  staGateway = gw;
  staSubnet = sn;
  useStaticIP = true;
  staticIPLearned = true;

  prefs.begin("wifi", false);
  prefs.putBool("static", true);
  prefs.putBool("learned", true);
  prefs.putString("ip", staStaticIP.toString());
  prefs.putString("gw", staGateway.toString());
  prefs.putString("sn", staSubnet.toString());
  prefs.putString("ssid", sta_ssid);
  prefs.putString("pass", sta_password);
  prefs.end();

  Serial.print("[WIFI] AUTO-LOCKED fixed IP: http://");
  Serial.print(staStaticIP);
  Serial.print(" (gw ");
  Serial.print(staGateway);
  Serial.println(")");
  Serial.println("[WIFI] Rebooting in 2s so static IP applies cleanly...");
  delay(2000);
  ESP.restart();
}

void startSoftAP() {
  // Always-on SoftAP at fixed QR address — recovery backdoor for any location
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  // Stay on WIFI_AP_STA so Station can join home WiFi without killing SoftAP
  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
    delay(50);
  }
  WiFi.softAPConfig(apIP, gateway, subnet);
  // channel 1 is often clearer when Station is also scanning; max 8 clients
  bool ok = WiFi.softAP(ap_ssid, ap_password, 1, 0, 8);
  delay(100);
  softApActive = ok || (WiFi.softAPIP().toString() == "192.168.4.1");
  // Captive-portal DNS: every domain resolves to SoftAP IP so phones stay connected
  dnsServer.stop();
  dnsServer.start(53, "*", apIP);
  Serial.printf("[WIFI] SoftAP %s - %s IP: %s\n", softApActive ? "ON" : "FAIL", ap_ssid, WiFi.softAPIP().toString().c_str());
  Serial.println("[WIFI] QR recovery page (always): http://192.168.4.1");
}

void stopSoftAP() {
  // SoftAP stays always on for QR – do not turn off
}

void fixStationDNS() {
  if (WiFi.status() != WL_CONNECTED) return;
  // Re-assert IP + public DNS (important for static mode)
  if (useStaticIP && staticIPLearned) {
    WiFi.config(staStaticIP, staGateway, staSubnet, staDns1, staDns2);
  } else {
    WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), staDns1, staDns2);
  }
  Serial.println("[WIFI] DNS set to 8.8.8.8 / 1.1.1.1");
}

bool stationCredsConfigured() {
  return (sta_ssid.length() > 0);
}

void connectStationWiFi() {
  loadLastStaIP();
  loadStationCreds();

  // Dual mode: SoftAP always + Station when credentials exist
  WiFi.mode(WIFI_AP_STA);
  delay(100);
  startSoftAP();
  indicateSoftAP();

  if (!stationCredsConfigured()) {
    Serial.println("[WIFI] No Station SSID – SoftAP only at http://192.168.4.1");
    staConnected = false;
    return;
  }

  Serial.printf("[WIFI] Connecting Station to \"%s\" ...\n", sta_ssid.c_str());
  WiFi.persistent(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());

  unsigned long t0 = millis();
  // Cap wait so SoftAP recovery UI stays responsive; retries continue in maintainWiFi
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  // SoftAP must stay up after every Station attempt
  startSoftAP();

  if (WiFi.status() == WL_CONNECTED) {
    staConnected = true;
    Serial.println("[WIFI] Station ON – IP: " + WiFi.localIP().toString());
    WiFi.setSleep(false);
    // Auto-lock to x.x.x.200 on this network (no manual IP ever)
    adoptStaticFromLease();
    saveLastStaIP();
    Serial.println("[WIFI] SoftAP still ON – http://192.168.4.1");
    fixStationDNS();
    delay(200);
    tryNTPSync();
    indicateStationOK();
    if (MDNS.begin("sigma")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("[WIFI] mDNS: http://sigma.local");
    }
  } else {
    // Static IP may be rejected — fall back to DHCP so device still works
    if (useStaticIP && staticIPLearned) {
      Serial.println("[WIFI] Static IP failed — retrying DHCP...");
      WiFi.disconnect(false);  // keep SoftAP up
      delay(200);
      startSoftAP();
      WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
      WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
      unsigned long t1 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t1 < 12000) {
        delay(250);
        Serial.print(".");
      }
      Serial.println();
      startSoftAP();  // always re-assert after STA attempt
      if (WiFi.status() == WL_CONNECTED) {
        staConnected = true;
        Serial.println("[WIFI] Station ON via DHCP – " + WiFi.localIP().toString());
        Serial.println("[WIFI] Target fixed IP remains: " + staStaticIP.toString());
        WiFi.setSleep(false);
        saveLastStaIP();
        fixStationDNS();
        tryNTPSync();
        indicateStationOK();
        return;
      }
    }
    staConnected = false;
    startSoftAP();  // SoftAP is the always-on backdoor
    Serial.println("[WIFI] Station failed – SoftAP still available at http://192.168.4.1");
    indicateStationFail();
  }
}

void maintainWiFi() {
  static unsigned long lastCheck = 0;
  const unsigned long BOOT_RETRY_WINDOW_MS = 5UL * 60UL * 1000UL;
  const unsigned long BOOT_RETRY_INTERVAL  = 30UL * 1000UL;
  const unsigned long NORMAL_CHECK_MS      = 15UL * 1000UL;

  unsigned long now = millis();
  bool inBootWindow = (now < BOOT_RETRY_WINDOW_MS);
  unsigned long interval = inBootWindow ? BOOT_RETRY_INTERVAL : NORMAL_CHECK_MS;
  if (now - lastCheck < interval) return;
  lastCheck = now;

  // SoftAP is the recovery backdoor — always re-assert (STA retry must not kill it)
  if (!softApActive || WiFi.softAPIP().toString() != "192.168.4.1") {
    startSoftAP();
  }

  if (!stationCredsConfigured()) return;

  if (WiFi.status() == WL_CONNECTED) {
    if (!staConnected) {
      staConnected = true;
      saveLastStaIP();
      Serial.println("[WIFI] Station reconnected – " + WiFi.localIP().toString());
      adoptStaticFromLease();
      fixStationDNS();
      indicateStationOK();
      tryNTPSync();
      // STA may move RF channel — SoftAP must stay advertised
      startSoftAP();
    }
    return;
  }

  // Station down – keep SoftAP, retry Station without wiping SoftAP
  if (staConnected) {
    staConnected = false;
    Serial.println("[WIFI] Station lost – SoftAP still at http://192.168.4.1");
  }

  Serial.printf("[WIFI] Retry Station \"%s\" ...\n", sta_ssid.c_str());
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.disconnect(false);   // false = do NOT shut down SoftAP
  delay(80);
  startSoftAP();            // re-assert SoftAP before STA begin
  applyStationIPConfig();
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
}

// =============================================================================
// CONFIG FILE — now includes parity, stopBits, byteSize, timeout, numSlaves,
// =============================================================================
void writeConfig() {
  JsonDocument doc;
  doc["interval"]   = (uint32_t)(logIntervalMs / 1000);
  doc["baud"]       = baudRate;
  doc["loggerType"] = globalLoggerType;
  doc["typeLocked"] = true;
  doc["divFactor"]  = divFactor;
  doc["unit"]       = unit;
  doc["numSlaves"]  = numSlavesConfigured;
  // Serial settings
  doc["parity"]     = serialParity;
  doc["stopBits"]   = serialStopBits;
  doc["byteSize"]   = serialByteSize;
  doc["timeout"]    = modbusTimeout;
  // Station WiFi
  doc["sta_ssid"]   = sta_ssid;
  doc["sta_pass"]   = sta_password;
  doc["deviceId"]    = deviceId;

  for (int i = 0; i < MAX_SLAVES; i++) {
    JsonObject sl = doc["slaves"][String(i)].to<JsonObject>();
    sl["enabled"]  = slaves[i].enabled;
    sl["id"]       = slaves[i].slaveID;
    sl["startReg"] = slaves[i].startReg;
    sl["mch"]      = slaves[i].modbusChannels;
    sl["sch"]      = slaves[i].saveChannels;
    sl["pvReg"]    = slaves[i].pvReg;
    sl["svReg"]    = slaves[i].svReg;
  }

  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!f) {
    Serial.println("[CFG] ERROR: cannot open config.json");
    return;
  }
  serializeJsonPretty(doc, f);
  f.close();
  loggerTypeLocked = true;
  Serial.println("[CFG] Written OK");
}

void readOrCreateConfig() {
  if (!SD.exists(CONFIG_FILE)) {
    JsonDocument doc;
    doc["interval"]   = (uint32_t)(logIntervalMs / 1000);
    doc["baud"]       = baudRate;
    doc["loggerType"] = globalLoggerType;
    doc["typeLocked"] = false;
    doc["divFactor"]  = divFactor;
    doc["unit"]       = unit;
    doc["numSlaves"]  = numSlavesConfigured;
    doc["parity"]     = serialParity;
    doc["stopBits"]   = serialStopBits;
    doc["byteSize"]   = serialByteSize;
    doc["timeout"]    = modbusTimeout;
    doc["sta_ssid"]   = "";
    doc["sta_pass"]   = "";
    doc["deviceId"]   = deviceId;

    for (int i = 0; i < MAX_SLAVES; i++) {
      JsonObject sl = doc["slaves"][String(i)].to<JsonObject>();
      sl["enabled"]  = false;
      sl["id"]       = (uint8_t)(i + 1);
      sl["startReg"] = (uint16_t)0;
      sl["mch"]      = (uint8_t)8;
      sl["sch"]      = (uint8_t)8;
      sl["pvReg"]    = (uint16_t)0;
      sl["svReg"]    = (uint16_t)1;
    }
    File f = SD.open(CONFIG_FILE, FILE_WRITE);
    if (f) {
      serializeJsonPretty(doc, f);
      f.close();
    }
    loggerTypeLocked = false;
    return;
  }

  File f = SD.open(CONFIG_FILE, FILE_READ);
  if (!f) {
    loggerTypeLocked = false;
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    loggerTypeLocked = false;
    return;
  }

  if (doc["interval"].is<uint32_t>())   logIntervalMs         = (unsigned long)doc["interval"].as<uint32_t>() * 1000UL;
  if (doc["baud"].is<uint32_t>())       baudRate              = doc["baud"].as<uint32_t>();
  if (doc["loggerType"].is<uint8_t>())  globalLoggerType      = doc["loggerType"].as<uint8_t>();
  if (globalLoggerType > 1)             globalLoggerType      = 0;
  if (doc["typeLocked"].is<bool>())     loggerTypeLocked      = doc["typeLocked"].as<bool>();
  if (doc["divFactor"].is<float>())     divFactor             = doc["divFactor"].as<float>();
  if (divFactor <= 0)                   divFactor             = 1.0f;
  if (doc["unit"].is<const char*>())    unit                  = doc["unit"].as<String>();
  if (doc["numSlaves"].is<uint8_t>())   numSlavesConfigured   = doc["numSlaves"].as<uint8_t>();
  if (numSlavesConfigured < 1)          numSlavesConfigured   = 1;
  if (numSlavesConfigured > MAX_SLAVES) numSlavesConfigured   = MAX_SLAVES;

  // Load new serial settings with safe defaults
  if (doc["parity"].is<uint8_t>())      serialParity          = doc["parity"].as<uint8_t>();
  if (serialParity > 2)                 serialParity          = 0;
  if (doc["stopBits"].is<uint8_t>())    serialStopBits        = doc["stopBits"].as<uint8_t>();
  if (serialStopBits < 1 || serialStopBits > 2) serialStopBits = 1;
  if (doc["byteSize"].is<uint8_t>())    serialByteSize        = doc["byteSize"].as<uint8_t>();
  if (serialByteSize != 7 && serialByteSize != 8) serialByteSize = 8;
  if (doc["timeout"].is<uint16_t>())    modbusTimeout         = doc["timeout"].as<uint16_t>();
  if (modbusTimeout < 100)              modbusTimeout         = 100;
  if (modbusTimeout > 10000)            modbusTimeout         = 10000;

  // Station WiFi credentials are HARDCODED in main.cpp – never override from config
  if (doc["deviceId"].is<const char*>() && doc["deviceId"].as<String>().length() > 0)
    deviceId = doc["deviceId"].as<String>();

  for (int i = 0; i < MAX_SLAVES; i++) {
    String key = String(i);
    if (!doc["slaves"][key].is<JsonObject>()) continue;
    JsonObject sl = doc["slaves"][key].as<JsonObject>();
    slaves[i].enabled        = sl["enabled"]  | false;
    slaves[i].slaveID        = sl["id"]        | (uint8_t)(i + 1);
    slaves[i].startReg       = sl["startReg"]  | (uint16_t)0;
    slaves[i].modbusChannels = sl["mch"]       | (uint8_t)8;
    slaves[i].saveChannels   = sl["sch"]       | (uint8_t)8;
    slaves[i].pvReg          = sl["pvReg"]     | (uint16_t)0;
    slaves[i].svReg          = sl["svReg"]     | (uint16_t)1;
  }

  Serial.printf("[CFG] Loaded OK loggerType=%d locked=%d numSlaves=%d parity=%d stop=%d byte=%d timeout=%d\n",
    globalLoggerType, loggerTypeLocked, numSlavesConfigured,
    serialParity, serialStopBits, serialByteSize, modbusTimeout);
}

// =============================================================================
// USER ACTION LOG
// =============================================================================
void logUserAction(const String& action) {
  if (currentLogDate == "") return;
  File logFile = SD.open(getUserLogFilename(), FILE_APPEND);
  if (logFile) {
    logFile.println(getDateFull() + " " + getTimeSS() + " | " + action);
    logFile.close();
  }
}

void handleGetAvailableDates() {

  String json = "[";
  bool first = true;

  File root = SD.open("/");

  if (!root) {
    server.send(500, "application/json", "[]");
    return;
  }

  String seen[200];
  int seenCount = 0;

  File file = root.openNextFile();

  while (file) {

    String name = file.name();

    if (name.startsWith("/"))
      name = name.substring(1);

    if (name.endsWith(".csv")) {

      String date = "";

      // NEW FORMAT: DD-MM-YYYY
      if (name.length() >= 10 &&
          isDigit(name[0]) &&
          isDigit(name[1]) &&
          name[2] == '-' &&
          isDigit(name[3]) &&
          isDigit(name[4]) &&
          name[5] == '-' &&
          isDigit(name[6]) &&
          isDigit(name[7]) &&
          isDigit(name[8]) &&
          isDigit(name[9]))
      {
        date = name.substring(0, 10);
      }

      if (date != "") {

        bool found = false;

        for (int i = 0; i < seenCount; i++) {
          if (seen[i] == date) {
            found = true;
            break;
          }
        }

        if (!found && seenCount < 200) {

          seen[seenCount++] = date;

          if (!first)
            json += ",";

          json += "\"" + date + "\"";
          first = false;
        }
      }
    }

    file = root.openNextFile();
  }

  json += "]";
  server.send(200, "application/json", json);
}

void handleGetFilesForDate() {
  if (!server.hasArg("date")) {
    server.send(400, "application/json", "[]");
    return;
  }
  String date = server.arg("date");
  String json = "[";
  bool first = true;
  File root = SD.open("/");

  if (!root) {
    server.send(500, "application/json", "[]");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.startsWith("/"))
      name = name.substring(1);
    if (name.endsWith(".csv") && name.startsWith(date)) {
      if (!first)
        json += ",";
      json += "\"" + name + "\"";
      first = false;
    }
    file = root.openNextFile();
  }

  json += "]";
  server.send(200, "application/json", json);
}

void handleSetWifi() {
  if (!server.hasArg("ssid")) {
    server.send(400, "text/plain", "Missing ssid");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  ssid.trim();
  if (ssid.length() == 0) {
    server.send(400, "text/plain", "SSID empty");
    return;
  }

  // Static IP options (company default 192.168.1.200)
  if (server.hasArg("static")) {
    useStaticIP = (server.arg("static") == "1" || server.arg("static") == "true");
  }
  if (server.hasArg("ip") && server.arg("ip").length() > 0) {
    staStaticIP.fromString(server.arg("ip"));
  }
  if (server.hasArg("gw") && server.arg("gw").length() > 0) {
    staGateway.fromString(server.arg("gw"));
  }
  if (server.hasArg("sn") && server.arg("sn").length() > 0) {
    staSubnet.fromString(server.arg("sn"));
  }

  saveStationCreds(ssid, pass);

  // Reconnect Station with new credentials — SoftAP must stay on for recovery
  staConnected = false;
  WiFi.disconnect(false);  // do not tear down SoftAP
  delay(200);
  startSoftAP();
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  applyStationIPConfig();
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  startSoftAP();  // always re-assert SoftAP after STA connect attempt

  if (WiFi.status() == WL_CONNECTED) {
    staConnected = true;
    saveLastStaIP();
    fixStationDNS();
    tryNTPSync();
    indicateStationOK();
    String msg = "WiFi saved and connected. Station IP: " + WiFi.localIP().toString();
    Serial.println("[WIFI] " + msg);
    server.send(200, "text/plain", msg);
  } else {
    staConnected = false;
    String msg = "WiFi saved but could not connect to \"" + ssid + "\". SoftAP still at http://192.168.4.1 — check password / 2.4GHz.";
    Serial.println("[WIFI] " + msg);
    server.send(200, "text/plain", msg);
  }
}


// =============================================================================
// OTA UPDATE (GitHub Releases + version.json)
// Policy: never while logging; Station+internet required; SoftAP stays on
// =============================================================================
String runOtaCheck(bool manual) {
  if (otaInProgress) {
    otaLastMessage = "Update already in progress";
    return otaLastMessage;
  }
  if (loggingEnabled) {
    otaDeferred = true;
    otaLastMessage = "Stop logging before updating";
    Serial.println("[OTA] Deferred – logging active");
    return otaLastMessage;
  }
  if (WiFi.status() != WL_CONNECTED) {
    otaLastMessage = "Connect to home WiFi (Station) first";
    Serial.println("[OTA] No Station WiFi");
    return otaLastMessage;
  }
  // HTTPS needs DNS + preferably valid clock
  fixStationDNS();
  if (!timeSyncedFromNTP) {
    Serial.println("[OTA] Trying NTP before HTTPS...");
    tryNTPSync();
  }
  delay(200);

  if (ESP.getFreeHeap() < 35000) {
    otaLastMessage = "Not enough free memory for OTA - try again later";
    Serial.printf("[OTA] Low heap %u\n", ESP.getFreeHeap());
    return otaLastMessage;
  }

  Serial.printf("[OTA] Checking version.json (heap=%u)...\n", ESP.getFreeHeap());
  otaLastMessage = "Checking for update...";

  {
    IPAddress tip;
    if (!WiFi.hostByName("raw.githubusercontent.com", tip)) {
      otaLastMessage = "DNS failed - Station WiFi has no internet?";
      Serial.println("[OTA] DNS failed for raw.githubusercontent.com");
      return otaLastMessage;
    }
    Serial.print("[OTA] DNS OK -> ");
    Serial.println(tip);
  }

  String body;
  int code = -1;
  for (int attempt = 1; attempt <= 2; attempt++) {
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(20);

    HTTPClient http;
    http.setTimeout(20000);
    http.setConnectTimeout(15000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setReuse(false);
    http.setUserAgent("SigmaControls-ESP32/1.1");

    if (!http.begin(client, OTA_VERSION_URL)) {
      Serial.printf("[OTA] begin() failed attempt %d\n", attempt);
      otaLastMessage = "Cannot start update check. Need home WiFi with internet.";
      http.end();
      delay(500);
      continue;
    }

    code = http.GET();
    if (code == HTTP_CODE_OK) {
      body = http.getString();
      http.end();
      break;
    }
    Serial.printf("[OTA] GET version attempt %d failed: %d\n", attempt, code);
    http.end();
    delay(800);
  }

  if (code != HTTP_CODE_OK) {
    // User-facing messages (avoid raw SSL codes like HTTP -1)
    if (code < 0) {
      otaLastMessage = "Cannot reach update server. Connect the device to home WiFi with internet, then try again.";
    } else if (code == 404) {
      otaLastMessage = "Update info not found on server (404). Contact support.";
    } else if (code == 403) {
      otaLastMessage = "Update server denied access (403). Try again later.";
    } else {
      otaLastMessage = "Update check failed (server code " + String(code) + "). Check internet and try again.";
    }
    Serial.printf("[OTA] GET version failed final: %d\n", code);
    return otaLastMessage;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    otaLastMessage = "Update info is invalid. Try again or contact support.";
    Serial.println("[OTA] JSON parse error");
    return otaLastMessage;
  }

  int remoteBuild = doc["build"] | 0;
  String remoteVer = doc["version"] | "unknown";
  String binUrl = doc["url"] | "";

  Serial.printf("[OTA] Local build %d | Remote %s build %d\n",
                FIRMWARE_BUILD, remoteVer.c_str(), remoteBuild);

  if (remoteBuild <= 0) {
    otaLastMessage = "Invalid remote build";
    return otaLastMessage;
  }
  if (remoteBuild <= FIRMWARE_BUILD) {
    otaLastMessage = "Already on latest (build " + String(FIRMWARE_BUILD) + ")";
    Serial.println("[OTA] Up to date");
    lastOtaCheckMs = millis();
    otaDeferred = false;
    return otaLastMessage;
  }
  if (binUrl.length() < 8) {
    otaLastMessage = "version.json missing firmware url";
    return otaLastMessage;
  }
  if (binUrl.indexOf("https://") != 0) {
    otaLastMessage = "Firmware URL must be HTTPS";
    return otaLastMessage;
  }
  if (binUrl.indexOf("github.com") < 0 && binUrl.indexOf("githubusercontent.com") < 0) {
    otaLastMessage = "Firmware URL must be on GitHub";
    return otaLastMessage;
  }

    Serial.print("[OTA] Updating to build ");
  Serial.print(remoteBuild);
  Serial.print(" from ");
  Serial.println(binUrl);
  otaLastMessage = "Downloading update - device will reboot (wait 1-2 min)";
  otaPendingUrl = binUrl;
  lastOtaCheckMs = millis();
  // Actual flash runs in maintainOta() so HTTP response can return first
  return otaLastMessage;
}

void performOtaDownload(const String& binUrl) {
  if (otaInProgress) return;
  if (loggingEnabled) {
    otaLastMessage = "Stop logging before updating";
    otaPendingUrl = "";
    return;
  }
  otaInProgress = true;
  otaLastMessage = "Downloading update - please wait, device will reboot";
  Serial.print("[OTA] Starting download: ");
  Serial.println(binUrl);
  Serial.print("[OTA] Free heap before update: ");
  Serial.println(ESP.getFreeHeap());

  WiFiClientSecure upClient;
  upClient.setInsecure();
  upClient.setTimeout(60);
  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  t_httpUpdate_return ret = httpUpdate.update(upClient, binUrl.c_str());

  otaInProgress = false;
  otaPendingUrl = "";
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      otaLastMessage = String("Update failed: ") + httpUpdate.getLastErrorString();
      Serial.print("[OTA] FAIL ");
      Serial.print(httpUpdate.getLastError());
      Serial.print(" ");
      Serial.println(httpUpdate.getLastErrorString());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      otaLastMessage = "No update payload";
      Serial.println("[OTA] No update payload");
      break;
    case HTTP_UPDATE_OK:
      otaLastMessage = "OK - rebooting";
      Serial.println("[OTA] OK - rebooting");
      break;
  }
}


void maintainOta() {
  // Mark current firmware valid after 45s uptime (enables rollback if new OTA crashes early)
  static bool appMarkedValid = false;
  if (!appMarkedValid && millis() > 45000UL) {
    appMarkedValid = true;
    esp_ota_mark_app_valid_cancel_rollback();
    Serial.println("[OTA] App marked valid after 45s (rollback window closed)");
  }

  // Run pending OTA after web handler returned
  static unsigned long pendingSince = 0;
  if (otaPendingUrl.length() > 0 && !otaInProgress) {
    if (pendingSince == 0) pendingSince = millis();
    if (millis() - pendingSince > 1000) {
      String url = otaPendingUrl;
      pendingSince = 0;
      performOtaDownload(url);
    }
    return;
  } else {
    pendingSince = 0;
  }

  if (otaInProgress) return;
  if (WiFi.status() != WL_CONNECTED) {
    stationUpSinceMs = 0;
    otaCheckedAfterSta = false;
    return;
  }
  if (stationUpSinceMs == 0) stationUpSinceMs = millis();

  if (!otaCheckedAfterSta && (millis() - stationUpSinceMs >= OTA_AFTER_STA_MS)) {
    otaCheckedAfterSta = true;
    if (!loggingEnabled) {
      Serial.println("[OTA] Post-connect check");
      runOtaCheck(false);
    } else {
      otaDeferred = true;
    }
  }

  if (millis() - lastOtaCheckMs >= OTA_CHECK_INTERVAL_MS) {
    lastOtaCheckMs = millis();
    if (!loggingEnabled) {
      Serial.println("[OTA] Periodic 12h check");
      runOtaCheck(false);
    } else {
      otaDeferred = true;
      Serial.println("[OTA] Periodic check deferred (logging)");
    }
  }
}

void handleOtaCheck() {
  String msg = runOtaCheck(true);
  server.send(200, "text/plain", msg);
  // Allow TCP to flush before possible long OTA in maintainOta
  delay(50);
}


void handleStatus() {
  JsonDocument doc;
  doc["logging"]      = loggingEnabled;
  doc["sdOK"]         = sdCardOK;
  doc["staConnected"] = staConnected;
  doc["timeValid"]    = timeValid;
  doc["ntp"]          = timeSyncedFromNTP;
  doc["deviceId"]     = deviceId;
  doc["deviceDate"]   = getDateFull();
  doc["fwVersion"]    = FIRMWARE_VERSION;
  doc["fwBuild"]      = FIRMWARE_BUILD;
  doc["otaMessage"]   = otaLastMessage;
  doc["otaDeferred"]  = otaDeferred;
  doc["otaBusy"]      = otaInProgress;
  doc["deviceTime"]   = getTimeSS();
  doc["apIP"]         = WiFi.softAPIP().toString();
  doc["softAp"]       = softApActive;
  doc["lastStaIP"]    = lastStaIP;
  doc["staSsid"]      = sta_ssid;
  doc["staCredsUser"] = staCredsFromUser;
  doc["useStatic"]    = useStaticIP;
  doc["staticIP"]     = staStaticIP.toString();
  doc["gateway"]      = staGateway.toString();
  doc["subnet"]       = staSubnet.toString();
  doc["staticLearned"]= staticIPLearned;
  // staIP = address that works RIGHT NOW; lockedIP = permanent target
  if (staConnected) doc["staIP"] = WiFi.localIP().toString();
  else if (lastStaIP.length()) doc["staIP"] = lastStaIP;
  if (staticIPLearned) doc["lockedIP"] = staStaticIP.toString();
  if (loggingEnabled) {
    JsonArray af = doc["activeFiles"].to<JsonArray>();
    for (int i = 0; i < MAX_SLAVES; i++) {
      if (!slaves[i].enabled) continue;
      String fn = getSlaveFilename(i);
      if (fn.startsWith("/")) fn = fn.substring(1);
      af.add(fn);
    }
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleErrorStatus() {
  String safe = errorMessage;
  safe.replace("\"", "'");
  server.send(200, "application/json", "{\"error\":\"" + safe + "\"}");
}

void handleGetConfig() {
  JsonDocument doc;
  doc["interval"]   = (uint32_t)(logIntervalMs / 1000);
  doc["baud"]       = baudRate;
  doc["loggerType"] = globalLoggerType;
  doc["typeLocked"] = loggerTypeLocked;
  doc["divFactor"]  = divFactor;
  doc["unit"]       = unit;
  doc["numSlaves"]  = numSlavesConfigured;
  doc["parity"]     = serialParity;
  doc["stopBits"]   = serialStopBits;
  doc["byteSize"]   = serialByteSize;
  doc["timeout"]    = modbusTimeout;
  // Network / IoT
  doc["sta_ssid"]    = sta_ssid;
  doc["sta_pass"]    = sta_password;
  doc["staConnected"]= staConnected;
  doc["deviceId"]    = deviceId;
  doc["timeValid"]   = timeValid;
  doc["ntp"]         = timeSyncedFromNTP;
  doc["apIP"]        = WiFi.softAPIP().toString();
  doc["staIP"]       = staConnected ? WiFi.localIP().toString() : "";

  for (int i = 0; i < MAX_SLAVES; i++) {
    JsonObject sl = doc["slaves"][String(i)].to<JsonObject>();
    sl["enabled"]  = slaves[i].enabled;
    sl["id"]       = slaves[i].slaveID;
    sl["startReg"] = slaves[i].startReg;
    sl["mch"]      = slaves[i].modbusChannels;
    sl["sch"]      = slaves[i].saveChannels;
    sl["pvReg"]    = slaves[i].pvReg;
    sl["svReg"]    = slaves[i].svReg;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleSetTime() {
  // SINGLE CLOCK RULE:
  // All CSV / graph timestamps always come from ESP32 nowTime (never phone clocks).
  // Browser may set time ONLY if device has no valid time yet (no NTP, no NVS last-good).
  // Once timeValid, further phones cannot shift the clock (avoids 10:00 vs 10:29 collisions).
  if (loggingEnabled) {
    server.send(403, "text/plain", "Cannot set time while logging");
    return;
  }
  if (timeSyncedFromNTP) {
    server.send(200, "text/plain", "NTP time locked – phone time ignored");
    return;
  }
  if (timeValid) {
    // Already have last-good / earlier set — do not let a second phone overwrite
    server.send(200, "text/plain", "Device time already set – phone time ignored");
    return;
  }
  if (!server.hasArg("ts") || !server.hasArg("tz")) {
    server.send(400, "text/plain", "Missing ts or tz");
    return;
  }
  long epoch = server.arg("ts").toInt();
  int tzMin  = server.arg("tz").toInt();
  epoch -= tzMin * 60;
  unsigned long seconds = (unsigned long)epoch;
  nowTime.s  = seconds % 60; seconds /= 60;
  nowTime.mi = seconds % 60; seconds /= 60;
  nowTime.h  = seconds % 24; seconds /= 24;
  if (server.hasArg("y") && server.hasArg("m") && server.hasArg("d")) {
    nowTime.y  = server.arg("y").toInt();
    nowTime.mo = server.arg("m").toInt();
    nowTime.d  = server.arg("d").toInt();
  }
  baseMillis = millis();
  timeValid = true;
  saveLastGoodTime();
  Serial.print("[TIME] First manual set from browser: ");
  char _tb[32];
  snprintf(_tb, sizeof(_tb), "%02d-%02d-%04d %02d:%02d:%02d",
           nowTime.d, nowTime.mo, nowTime.y, nowTime.h, nowTime.mi, nowTime.s);
  Serial.println(_tb);
  server.send(200, "text/plain", "Device time set once – later phones ignored until reboot without NVS");
}


void saveLoggingState(bool wanted) {
  loggingWanted = wanted;
  prefs.begin("log", false);
  prefs.putBool("wanted", wanted);
  prefs.putString("logDate", currentLogDate);
  prefs.end();
  Serial.printf("[LOG] Saved resume flag wanted=%d date=%s\n", wanted, currentLogDate.c_str());
}

bool loadLoggingState() {
  prefs.begin("log", true);
  loggingWanted = prefs.getBool("wanted", false);
  String d = prefs.getString("logDate", "");
  prefs.end();
  if (loggingWanted && d.length() > 0) currentLogDate = d;
  Serial.printf("[LOG] Loaded resume flag wanted=%d date=%s\n", loggingWanted, currentLogDate.c_str());
  return loggingWanted;
}

bool isActiveSessionFile(const String& fname) {
  if (!loggingEnabled) return false;
  String f = fname;
  if (!f.startsWith("/")) f = "/" + f;
  for (int i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].enabled) continue;
    if (f == getSlaveFilename(i)) return true;
  }
  if (currentLogDate.length() > 0 && f == getUserLogFilename()) return true;
  return false;
}

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file");
    return;
  }
  String fname = server.arg("file");
  if (!fname.startsWith("/")) fname = "/" + fname;

  // While logging: block only the active session file(s); other dates/files OK
  if (isActiveSessionFile(fname)) {
    server.send(409, "text/plain", "Cannot download the active log file while logging. Stop logging first, or download older files.");
    return;
  }

  File file = SD.open(fname, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "Not found: " + fname);
    return;
  }
  String cleanName = fname.substring(1);
  // Direct attachment download — avoid inline/preview prompts where possible
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + cleanName + "\"");
  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("Content-Transfer-Encoding", "binary");
  if (file.size() > 0) {
    server.sendHeader("Content-Length", String(file.size()));
  }
  server.streamFile(file, "application/octet-stream");
  file.close();
}

void handleStart() {
  if (!sdCardOK) {
    errorMessage = "SD card not found - SD error";
    digitalWrite(BLUE_LED, HIGH);
    server.send(503, "text/plain", "SD card not inserted.");
    return;
  }

  if (server.hasArg("int"))      logIntervalMs       = server.arg("int").toInt() * 1000UL;
  if (server.hasArg("baud"))     baudRate            = server.arg("baud").toInt();
  if (server.hasArg("div"))      { divFactor = server.arg("div").toFloat(); if (divFactor <= 0) divFactor = 1.0f; }
  if (server.hasArg("unit"))     unit                = server.arg("unit");
  if (server.hasArg("ns"))       {
    numSlavesConfigured = (uint8_t)server.arg("ns").toInt();
    if (numSlavesConfigured < 1)          numSlavesConfigured = 1;
    if (numSlavesConfigured > MAX_SLAVES) numSlavesConfigured = MAX_SLAVES;
  }

  // New serial settings
  if (server.hasArg("parity"))   {
    serialParity = (uint8_t)server.arg("parity").toInt();
    if (serialParity > 2) serialParity = 0;
  }
  if (server.hasArg("stopBits")) {
    serialStopBits = (uint8_t)server.arg("stopBits").toInt();
    if (serialStopBits < 1 || serialStopBits > 2) serialStopBits = 1;
  }
  if (server.hasArg("byteSize")) {
    serialByteSize = (uint8_t)server.arg("byteSize").toInt();
    if (serialByteSize != 7 && serialByteSize != 8) serialByteSize = 8;
  }
  if (server.hasArg("timeout"))  {
    modbusTimeout = (uint16_t)server.arg("timeout").toInt();
    if (modbusTimeout < 100)   modbusTimeout = 100;
    if (modbusTimeout > 10000) modbusTimeout = 10000;
  }

  if (!loggerTypeLocked && server.hasArg("lt")) globalLoggerType = server.arg("lt").toInt();
  if (globalLoggerType > 1) globalLoggerType = 0;

  bool anyEnabled = false;
  for (int i = 0; i < MAX_SLAVES; i++) {
    String enKey = "en" + String(i);
    slaves[i].enabled = (server.hasArg(enKey) && server.arg(enKey) == "1");
    if (slaves[i].enabled) {
      anyEnabled = true;
      slaves[i].slaveID = server.hasArg("sid" + String(i))
        ? (uint8_t)server.arg("sid" + String(i)).toInt()
        : (i + 1);

      if (globalLoggerType == 1) {
        // PID
        slaves[i].pvReg          = server.hasArg("pvr" + String(i)) ? (uint16_t)server.arg("pvr" + String(i)).toInt() : 0;
        slaves[i].svReg          = server.hasArg("svr" + String(i)) ? (uint16_t)server.arg("svr" + String(i)).toInt() : 1;
        slaves[i].modbusChannels = 2;
        slaves[i].saveChannels   = 2;
        slaves[i].startReg       = min(slaves[i].pvReg, slaves[i].svReg);
      } else {
        // TempScanner
        slaves[i].startReg       = server.hasArg("sreg" + String(i)) ? (uint16_t)server.arg("sreg" + String(i)).toInt() : 0;
        slaves[i].modbusChannels = server.hasArg("mch" + String(i))  ? (uint8_t)server.arg("mch" + String(i)).toInt()  : 8;
        slaves[i].saveChannels   = server.hasArg("sch" + String(i))  ? (uint8_t)server.arg("sch" + String(i)).toInt()  : 8;
        if (slaves[i].modbusChannels != 8 && slaves[i].modbusChannels != 16 && slaves[i].modbusChannels != 24) {
          slaves[i].modbusChannels = 8;
        }
        if (slaves[i].saveChannels < 1 || slaves[i].saveChannels > 24) {
          slaves[i].saveChannels = 8;
        }
      }

      batchBuffer[i]  = "";
      batchCount[i]   = 0;
      commFailCounts[i] = 0;
      liveHead[i]     = 0;
      liveCount[i]    = 0;
    }
  }

  if (!anyEnabled) {
    server.send(400, "text/plain", "No slaves enabled");
    return;
  }

  // Re-init Serial2 with the new settings before pre-start check
  reinitSerial2();

  // Pre-start COMM check — start if AT LEAST one slave answers.
  // Offline IDs are reported; logging continues for working slaves only.
  int okCount = 0;
  int failCount = 0;
  String failIds = "";
  for (int i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].enabled) continue;
    node.begin(slaves[i].slaveID, Serial2);
    uint16_t testReg = (globalLoggerType == 1) ? slaves[i].pvReg : slaves[i].startReg;
    uint8_t result = node.readHoldingRegisters(testReg, 1);
    delay(50);
    if (result != node.ku8MBSuccess) {
      failCount++;
      if (failIds.length()) failIds += ",";
      failIds += String(slaves[i].slaveID);
      Serial.printf("[LOG] Pre-start: Slave ID %d no response (err 0x%02X)",slaves[i].slaveID, result);
    } else {
      okCount++;
    }
  }
  if (okCount == 0) {
    errorMessage = "Pre-start: no enabled slave responded";
    digitalWrite(RED_LED, HIGH);
    playBeepPattern(3);
    server.send(503, "text/plain", errorMessage);
    return;
  }
  if (failCount > 0) {
    errorMessage = "Slave ID(s) offline: " + failIds + " — logging others";
    digitalWrite(RED_LED, HIGH);
    playBeepPattern(2);
  } else {
    errorMessage = "";
    digitalWrite(RED_LED, LOW);
  }
  for (int i = 0; i < MAX_SLAVES; i++) commFailCounts[i] = 0;

  writeConfig();
  loggingEnabled  = true;
  currentLogDate  = getDateYY();
  baseMillis      = millis();
  lastLogMs       = millis() - logIntervalMs;
  errorMessage    = "";
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);

  Serial.printf("[LOG] Started session file date=%s  divFactor=%.4f  interval=%lus\n",
                currentLogDate.c_str(), divFactor, logIntervalMs / 1000UL);

  logUserAction("Logging started interval=" + String(logIntervalMs / 1000) +
                "s baud=" + String(baudRate) +
                " div=" + String(divFactor, 4) +
                " parity=" + String(serialParity) +
                " stop=" + String(serialStopBits) +
                " byte=" + String(serialByteSize) +
                " timeout=" + String(modbusTimeout) +
                " type=" + String(loggerTypeName[globalLoggerType]) +
                " slaves=" + String(numSlavesConfigured));

  saveLoggingState(true);
  playBeepPattern(1);
  server.send(200, "text/plain", "Logging started");
}

void handleStop() {
  for (int i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].enabled || batchBuffer[i].length() == 0) continue;
    File f = SD.open(getSlaveFilename(i), FILE_APPEND);
    if (f) {
      f.print(batchBuffer[i]);
      f.close();
      batchBuffer[i]  = "";
      batchCount[i]   = 0;
    } else {
      char buf[64];
      snprintf(buf, sizeof(buf), "Flush failed slave %d", i);
      errorSD(buf);
    }
  }
  logUserAction("Logging stopped");
  loggingEnabled = false;
  currentLogDate = "";   // next start will use the new day's filename
  saveLoggingState(false);
  if (otaDeferred) {
    Serial.println("[OTA] Deferred OTA after stop");
    otaDeferred = false;
    // run after response path — schedule via lastOtaCheckMs
    lastOtaCheckMs = millis() - OTA_CHECK_INTERVAL_MS + 3000;  // fire in ~3s via maintainOta
  }  // explicit stop — do NOT auto-resume after power loss
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  writeConfig();
  playBeepPattern(1);
  server.send(200, "text/plain", "Logging stopped");
}

void handleLiveData() {
  if (!loggingEnabled) {
    server.send(200, "application/json", "{\"logging\":false}");
    return;
  }
  String json = "{\"logging\":true,\"interval\":";
  json += String(logIntervalMs / 1000);
  json += ",\"unit\":\"";
  json += unit;
  json += "\"";
  json += ",\"loggerType\":";
  json += String(globalLoggerType);
  json += ",\"slaves\":[";
  bool firstSlave = true;
  for (int s = 0; s < MAX_SLAVES; s++) {
    if (!slaves[s].enabled) continue;
    if (!firstSlave) json += ",";
    firstSlave = false;
    json += "{\"idx\":";
    json += String(s);
    json += ",\"id\":";
    json += String(slaves[s].slaveID);
    json += ",\"ch\":";
    json += String(slaves[s].saveChannels);
    json += ",\"rows\":[";
    int count = liveCount[s];
    int start = (count < LIVE_BUF_SIZE) ? 0 : liveHead[s];
    for (int r = 0; r < count; r++) {
      int idx = (start + r) % LIVE_BUF_SIZE;
      if (r > 0) json += ",";
      json += "{\"ts\":\"";
      json += String(liveBuf[s][idx].ts);
      json += "\",\"v\":[";
      for (int c = 0; c < slaves[s].saveChannels && c < 24; c++) {
        if (c > 0) json += ",";
        json += String(liveBuf[s][idx].ch[c], 1);
      }
      json += "]}";
      if (json.length() > 3000) break;
    }
    json += "]}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleHistoryData() {
  if (!loggingEnabled && currentLogDate == "") {
    server.send(200, "application/json", "{\"error\":\"Not logging\"}");
    return;
  }
  int slaveIdx = server.hasArg("slave") ? server.arg("slave").toInt() : 0;
  int page     = server.hasArg("page")  ? server.arg("page").toInt()  : 0;
  if (slaveIdx < 0 || slaveIdx >= MAX_SLAVES || !slaves[slaveIdx].enabled) {
    server.send(200, "application/json", "{\"error\":\"Invalid slave\"}");
    return;
  }
  String fname = getSlaveFilename(slaveIdx);
  File f = SD.open(fname, FILE_READ);
  if (!f) {
    server.send(200, "application/json", "{\"error\":\"File not found\"}");
    return;
  }
  int totalLines = 0;
  while (f.available()) {
    if (f.read() == '\n') totalLines++;
  }
  f.seek(0);
  const int PAGE_SIZE   = 200;
  int totalPages        = max(1, (totalLines + PAGE_SIZE - 1) / PAGE_SIZE);
  if (page < 0 || page >= totalPages) page = max(0, totalPages - 1);
  int skipped = 0;
  while (f.available() && skipped < page * PAGE_SIZE) {
    f.readStringUntil('\n');
    skipped++;
  }
  String json = "{\"slave\":";
  json += String(slaveIdx);
  json += ",\"page\":";
  json += String(page);
  json += ",\"total\":";
  json += String(totalPages);
  json += ",\"lines\":";
  json += String(totalLines);
  json += ",\"ch\":";
  json += String(slaves[slaveIdx].saveChannels);
  json += ",\"loggerType\":";
  json += String(globalLoggerType);
  json += ",\"rows\":[";
  int emitted = 0;
  while (f.available() && emitted < PAGE_SIZE) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (emitted > 0) json += ",";
    json += "[";
    int pos = 0, field = 0;
    while (pos <= (int)line.length()) {
      int comma = line.indexOf(',', pos);
      if (comma < 0) comma = line.length();
      String tok = line.substring(pos, comma);
      if (field > 0) json += ",";
      json += "\"";
      json += tok;
      json += "\"";
      pos = comma + 1;
      field++;
    }
    json += "]";
    emitted++;
    if (json.length() > 8000) break;
  }
  f.close();
  json += "]}";
  server.send(200, "application/json", json);
}

// =============================================================================
// MAIN WEB UI
// =============================================================================
void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  // ---- CHUNK 0: DOCTYPE, head, CSS variables, base styles ----
  server.sendContent(F(R"CHUNK0(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<title>Sigma Controls Logger</title>
<script>
// Chart.js — Station/internet only. SoftAP (192.168.4.1) skips CDN so page loads instantly.
window._chartReady = false;
(function(){
  function markReady(){ window._chartReady = (typeof Chart !== 'undefined'); }
  var h = location.hostname || '';
  var isSoftAP = (h === '192.168.4.1' || h.indexOf('192.168.4.') === 0);
  if (isSoftAP || !navigator.onLine) { markReady(); return; }
  var urls = [
    'https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js',
    'https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js'
  ];
  var i = 0;
  function tryLoad() {
    if (i >= urls.length) { markReady(); return; }
    var s = document.createElement('script');
    s.src = urls[i++];
    s.onload = markReady;
    s.onerror = function(){ setTimeout(tryLoad, 200); };
    document.head.appendChild(s);
  }
  tryLoad();
})();
</script>
<style>
*,*::before,*::after{margin:0;padding:0;box-sizing:border-box}
:root{
  --p1:#4f46e5;--p2:#7c3aed;
  --bg:#f4f6ff;--bg2:#eaecfb;
  --card:#fff;--card2:#f7f8ff;
  --border:#dde1f9;--border2:#c7ccf5;
  --txt:#1e1b4b;--txt2:#4b5280;--txt3:#8b93c4;
  --success:#059669;--danger:#dc2626;--warn:#d97706;
  --grad:linear-gradient(135deg,#4f46e5 0%,#7c3aed 55%,#a855f7 100%);
  --grad-btn:linear-gradient(135deg,#4f46e5,#7c3aed);
  --grad-stop:linear-gradient(135deg,#dc2626,#b91c1c);
  --grad-dl:linear-gradient(135deg,#059669,#047857);
  --shadow-sm:0 2px 12px rgba(79,70,229,.09),0 1px 3px rgba(0,0,0,.05);
  --r:14px;--rs:10px;--rxs:7px;
}
[data-theme=dark]{
  --bg:#0f0e24;--bg2:#181630;
  --card:#1a1838;--card2:#201e40;
  --border:#2a2750;--border2:#3a3768;
  --txt:#ffffff;--txt2:#e0dcff;--txt3:#b0aad8;
  --shadow-sm:0 2px 12px rgba(0,0,0,.3);
}
html{font-size:15px;-webkit-text-size-adjust:100%}
body{
  font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
  background:var(--bg);min-height:100vh;
  display:flex;justify-content:center;padding-bottom:60px;
  color:var(--txt);transition:background .3s,color .3s;
}
.app{width:100%;max-width:820px;display:flex;flex-direction:column}
.hdr{background:var(--grad);padding:26px 20px 22px;position:relative;overflow:hidden}
.hdr::before{content:'';position:absolute;inset:0;pointer-events:none;
  background:radial-gradient(ellipse at 20% 70%,rgba(255,255,255,.13) 0%,transparent 60%),
             radial-gradient(ellipse at 80% 15%,rgba(255,255,255,.08) 0%,transparent 50%);}
.hdr-row{position:relative;z-index:1;display:flex;flex-direction:column;align-items:center;gap:7px;padding-top:4px}
.hdr-logo {
    display: flex;
    justify-content: center;
    align-items: center;
    margin: 0 auto 18px;
}
.hdr-logo img {
    width: 520px;
    max-width: 85%;
    height: auto;
    border-radius: 12px;
    display: block;
}
.hdr-title{color:#fff;font-size:1.45rem;font-weight:800;letter-spacing:-.5px;text-align:center;text-shadow:0 2px 10px rgba(0,0,0,.25);line-height:1.2}
.hdr-sub{color:rgba(255,255,255,.72);font-size:.74rem;text-align:center;letter-spacing:.04em}
.theme-btn{
  position:absolute;top:12px;right:12px;z-index:20;
  display:inline-flex;align-items:center;gap:4px;padding:6px 11px;
  border-radius:999px;border:1.5px solid rgba(255,255,255,.35);
  background:rgba(0,0,0,.18);color:#fff;
  font-size:.72rem;font-weight:600;cursor:pointer;
  transition:background .2s;font-family:inherit;white-space:nowrap;
}
.theme-btn:hover{background:rgba(0,0,0,.3)}
.hdr-badge{
  display:inline-flex;align-items:center;gap:5px;padding:4px 12px;
  border-radius:999px;border:1.5px solid rgba(255,255,255,.22);
  background:rgba(255,255,255,.1);color:rgba(255,255,255,.92);
  font-size:.72rem;font-weight:700;pointer-events:none;white-space:nowrap;margin-top:2px;
}
.led-strip{background:var(--card);border-bottom:1px solid var(--border);display:flex;flex-wrap:wrap}
.led-cell{flex:1;min-width:80px;display:flex;align-items:center;justify-content:center;gap:8px;
  padding:11px 8px;border-right:1px solid var(--border)}
.led-cell:last-child{border-right:none}
.led{width:22px;height:22px;border-radius:50%;flex-shrink:0;position:relative;transition:background .25s,box-shadow .25s}
.led-ring{position:absolute;inset:0;border-radius:50%;pointer-events:none;opacity:0;animation:led-pulse 1.4s ease-out infinite}
@keyframes led-pulse{0%{transform:scale(1);opacity:.75}100%{transform:scale(2.6);opacity:0}}
.led-pwr{background:#22c55e;box-shadow:0 0 0 4px rgba(34,197,94,.2),0 0 16px 5px rgba(34,197,94,.6)}
.led-comm-off{background:#fecaca;box-shadow:none}
.led-comm-on{background:#ef4444;box-shadow:0 0 0 5px rgba(239,68,68,.25),0 0 20px 6px rgba(239,68,68,.75)}
.led-comm-on .led-ring{background:rgba(239,68,68,.5);opacity:1}
.led-sd-off{background:#93c5fd;box-shadow:none}
.led-sd-on{background:#3b82f6;box-shadow:0 0 0 5px rgba(59,130,246,.25),0 0 20px 6px rgba(59,130,246,.8)}
.led-sd-on .led-ring{background:rgba(59,130,246,.5);opacity:1}
[data-theme=dark] .led-comm-off{background:#3a1515}
[data-theme=dark] .led-sd-off{background:#1e3a5f}
.led-label{font-size:.72rem;font-weight:600;color:var(--txt2);line-height:1.3;white-space:nowrap}
.led-label b{display:block;font-size:.65rem;font-weight:500;color:var(--txt3)}
.main{padding:16px 14px;display:flex;flex-direction:column;gap:14px}
.st{border-radius:var(--r);padding:14px 18px;display:flex;align-items:center;gap:14px;
  border:2px solid transparent;transition:all .3s}
.st-idle{background:var(--card);border-color:var(--border);color:var(--txt2)}
.st-live{background:rgba(5,150,105,.07);border-color:rgba(5,150,105,.35);color:var(--success)}
.st-ico{width:42px;height:42px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:1.3rem;flex-shrink:0}
.st-idle .st-ico{background:var(--bg2)}
.st-live .st-ico{background:rgba(5,150,105,.12)}
.st-tt{font-size:.96rem;font-weight:700}
.st-ts{font-size:.77rem;opacity:.72;margin-top:2px}
@keyframes pulse-ring{0%{box-shadow:0 0 0 0 rgba(5,150,105,.55)}70%{box-shadow:0 0 0 10px rgba(5,150,105,0)}100%{box-shadow:0 0 0 0 rgba(5,150,105,0)}}
.pa{animation:pulse-ring 1.8s infinite}
.err-box{display:none;background:rgba(220,38,38,.08);border:2px solid rgba(220,38,38,.35);
  border-radius:var(--rs);padding:11px 15px;color:var(--danger);font-size:.86rem;
  font-weight:600;align-items:center;gap:10px}
.sc{background:var(--card);border-radius:var(--r);box-shadow:var(--shadow-sm);border:1px solid var(--border);overflow:hidden}
.sc-hdr{padding:13px 17px;display:flex;align-items:center;gap:12px;border-bottom:1px solid var(--border);
  cursor:pointer;user-select:none;-webkit-user-select:none;transition:background .15s}
.sc-hdr:hover{background:var(--bg2)}
.sc-ico{width:36px;height:36px;border-radius:9px;flex-shrink:0;display:flex;align-items:center;
  justify-content:center;font-size:1rem;color:#fff}
.ico-v{background:var(--grad-btn)}
.ico-b{background:linear-gradient(135deg,#0284c7,#0369a1)}
.ico-t{background:linear-gradient(135deg,#0d9488,#0f766e)}
.ico-g{background:var(--grad-dl)}
.ico-o{background:linear-gradient(135deg,#ea580c,#c2410c)}
.ico-p{background:linear-gradient(135deg,#7c3aed,#6d28d9)}
.ico-s{background:linear-gradient(135deg,#0891b2,#0e7490)}
.sc-meta{flex:1}
.sc-meta h3{font-size:.94rem;font-weight:700;color:var(--txt)}
.sc-meta p{font-size:.73rem;color:var(--txt3);margin-top:1px}
.sc-arr{font-size:.8rem;color:var(--txt3);transition:transform .25s;flex-shrink:0}
.sc-arr.open{transform:rotate(180deg)}
.sc-body{padding:18px 17px}
.sc-body.hide{display:none}

details.sc > summary{list-style:none}
details.sc > summary::-webkit-details-marker{display:none}
details.sc[open] > summary .sc-arr{transform:rotate(180deg)}
details.sc > summary.sc-hdr{border-bottom:1px solid var(--border)}

.field{margin-bottom:13px}
.field:last-child{margin-bottom:0}
.field label{display:block;font-size:.71rem;font-weight:700;color:var(--txt3);
  text-transform:uppercase;letter-spacing:.07em;margin-bottom:5px}
)CHUNK0"));

  // ---- CHUNK 1: More CSS ----
  server.sendContent(F(R"CHUNK1(.field input,.field select{
  width:100%;padding:10px 12px;font-size:.9rem;
  border:1.5px solid var(--border);border-radius:8px;
  background:var(--bg2);color:var(--txt);
  transition:border-color .2s,box-shadow .2s;
  appearance:none;-webkit-appearance:none;font-family:inherit;
}
[data-theme=dark] .field input,[data-theme=dark] .field select{
  color:#ffffff;background:#1a1838;border-color:#3a3768;
}
[data-theme=dark] .mon-tbl td{color:#e0dcff;}
[data-theme=dark] .mon-tbl td:first-child{color:#b0aad8;}
[data-theme=dark] .mon-tbl tr.latest td{color:#ffffff;}
[data-theme=dark] .sc-meta h3{color:#ffffff;}
[data-theme=dark] .led-label{color:#e0dcff;}
[data-theme=dark] .hist-meta{color:#e0dcff;}
[data-theme=dark] .chart-title{color:#e0dcff;}
[data-theme=dark] .chart-legend-item{color:#e0dcff;}
[data-theme=dark] .f-name{color:#ffffff;}
[data-theme=dark] .f-date{color:#b0aad8;}
[data-theme=dark] .copy{color:#b0aad8;}
[data-theme=dark] .tgl-lbl{color:#b0aad8;}
[data-theme=dark] .slv-num{color:#e0dcff;}
[data-theme=dark] .lt-note{color:#b0aad8;}
[data-theme=dark] .field-hint{color:#b0aad8;}
[data-theme=dark] .tb-lbl{color:rgba(255,255,255,.85);}
[data-theme=dark] .serial-summary{color:#e0dcff;}
.field input:focus,.field select:focus{
  border-color:var(--p1);box-shadow:0 0 0 3px rgba(79,70,229,.15);
  outline:none;background:var(--card)}
.field input:disabled,.field select:disabled{opacity:.45;cursor:not-allowed}
.field-hint{font-size:.69rem;color:var(--txt3);margin-top:4px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:11px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:11px}
.g4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:11px}
.tbox{background:var(--grad);border-radius:var(--rs);padding:15px 18px;margin-bottom:14px;
  display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px}
.tb-lbl{font-size:.66rem;color:rgba(255,255,255,.72);text-transform:uppercase;letter-spacing:.08em;margin-bottom:3px}
.tb-val{font-size:1.15rem;font-weight:800;color:#fff;letter-spacing:.05em;font-variant-numeric:tabular-nums}
.lt-locked{display:flex;align-items:center;gap:10px;padding:10px 13px;
  background:var(--bg2);border:1.5px solid var(--border);border-radius:8px}
.lt-badge{padding:4px 12px;background:var(--grad-btn);color:#fff;border-radius:999px;font-size:.78rem;font-weight:700}
.lt-note{font-size:.72rem;color:var(--txt3)}
.slv-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.slv-tile{border:1.5px solid var(--border);border-radius:var(--rs);overflow:hidden;
  background:var(--bg2);transition:border-color .2s,box-shadow .2s}
.slv-tile.on{border-color:var(--p1);background:var(--card);box-shadow:0 0 0 3px rgba(79,70,229,.1)}
.slv-hdr{padding:9px 12px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid var(--border)}
.slv-num{display:flex;align-items:center;gap:7px;font-size:.85rem;font-weight:700;color:var(--txt2)}
.slv-badge{width:22px;height:22px;border-radius:6px;background:var(--grad-btn);color:#fff;
  font-size:.7rem;font-weight:800;display:flex;align-items:center;justify-content:center}
.tgl{position:relative;display:inline-flex;align-items:center;gap:6px;cursor:pointer}
.tgl input{opacity:0;width:0;height:0;position:absolute}
.tgl-track{width:36px;height:20px;border-radius:999px;background:var(--border2);position:relative;transition:background .25s;flex-shrink:0}
.tgl-track::after{content:'';position:absolute;top:3px;left:3px;width:14px;height:14px;
  border-radius:50%;background:#fff;transition:transform .25s;box-shadow:0 1px 3px rgba(0,0,0,.25)}
.tgl input:checked+.tgl-track{background:var(--p1)}
.tgl input:checked+.tgl-track::after{transform:translateX(16px)}
.tgl-lbl{font-size:.72rem;font-weight:600;color:var(--txt3)}
.slv-fields{padding:10px 12px;display:none}
.slv-fields.show{display:block}
.slv-fields .field{margin-bottom:8px}
.slv-fields .field label{font-size:.67rem}
.slv-fields .field input,.slv-fields .field select{padding:7px 9px;font-size:.83rem}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;
  width:100%;padding:13px 18px;font-size:.93rem;font-weight:700;
  border:none;border-radius:var(--rs);cursor:pointer;font-family:inherit;
  letter-spacing:.02em;transition:transform .15s,box-shadow .2s,filter .15s}
.btn:active{transform:scale(.97)}
.btn:disabled{opacity:.45;cursor:not-allowed;transform:none!important}
.btn svg{width:17px;height:17px;flex-shrink:0}
.btn-start{background:var(--grad-btn);color:#fff;box-shadow:0 6px 20px rgba(79,70,229,.4)}
.btn-start:hover:not(:disabled){box-shadow:0 10px 28px rgba(79,70,229,.55);transform:translateY(-1px);filter:brightness(1.08)}
.btn-stop{background:var(--grad-stop);color:#fff;box-shadow:0 6px 20px rgba(220,38,38,.4)}
.btn-stop:hover:not(:disabled){box-shadow:0 10px 28px rgba(220,38,38,.55);transform:translateY(-1px);filter:brightness(1.08)}
.btn-dl{background:var(--grad-dl);color:#fff;box-shadow:0 6px 20px rgba(5,150,105,.4)}
.btn-dl:hover:not(:disabled){box-shadow:0 10px 28px rgba(5,150,105,.55);transform:translateY(-1px);filter:brightness(1.08)}
.btn-zip{background:linear-gradient(135deg,#0369a1,#075985);color:#fff;box-shadow:0 6px 20px rgba(3,105,161,.4);margin-top:4px;border-top:1px dashed rgba(255,255,255,.18)}
.btn-zip:hover:not(:disabled){box-shadow:0 10px 28px rgba(3,105,161,.55);transform:translateY(-1px);filter:brightness(1.08)}
.btn-zip:disabled{opacity:.45;cursor:not-allowed;transform:none!important;filter:none!important}
.btn-grp{display:flex;flex-direction:column;gap:10px}
.btn-zip-note{font-size:.7rem;color:var(--txt3);text-align:center;margin-top:5px;padding:0 4px}
.log-info{display:none;margin-top:12px;padding:12px 15px;
  background:rgba(5,150,105,.08);border:1.5px solid rgba(5,150,105,.3);
  border-radius:var(--rxs);font-size:.84rem;color:var(--success);font-weight:600}
.log-info span{display:block;font-weight:400;opacity:.8;margin-top:2px}
.serial-summary{background:var(--bg2);border:1px solid var(--border);border-radius:8px;
  padding:10px 14px;margin-top:14px;margin-bottom:14px;font-size:.78rem;color:var(--txt2);
  display:flex;flex-wrap:wrap;gap:8px 20px}
.serial-summary b{color:var(--txt);font-weight:700}
.ns-display{display:inline-flex;align-items:center;gap:8px;padding:7px 13px;
  background:var(--grad-btn);color:#fff;border-radius:999px;font-size:.82rem;
  font-weight:700;margin-bottom:35px}
)CHUNK1"));

  // ---- CHUNK 2: Calendar + file panel + footer CSS + live monitor CSS ----
  server.sendContent(F(R"CHUNK2(.cal-legend{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:12px;font-size:.76rem;font-weight:600;color:var(--txt3)}
.cal-legend i{display:flex;align-items:center;gap:5px}
.leg{width:10px;height:10px;border-radius:3px;flex-shrink:0}
.leg-h{background:rgba(79,70,229,.4);border:1px solid var(--p1)}
.leg-s{background:var(--p1)}
.leg-e{background:var(--bg2);border:1px solid var(--border)}
.cal-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:13px;max-height:520px;overflow-y:auto;padding:2px}
.cal-grid::-webkit-scrollbar{width:5px}
.cal-grid::-webkit-scrollbar-thumb{background:var(--border2);border-radius:3px}
.mo-card{border:1px solid var(--border);border-radius:var(--rs);overflow:hidden;background:var(--card);box-shadow:var(--shadow-sm)}
.mo-hdr{padding:8px 13px;display:flex;align-items:center;justify-content:space-between;background:var(--grad);color:#fff}
.mo-name{font-weight:700;font-size:.84rem;letter-spacing:.03em}
.mo-cnt{background:rgba(255,255,255,.22);padding:2px 8px;border-radius:999px;font-size:.68rem;font-weight:700}
.mo-tbl{width:100%;border-collapse:collapse}
.mo-tbl th{padding:5px 0;font-size:.65rem;font-weight:700;color:var(--txt3);text-align:center;
  text-transform:uppercase;background:var(--card2);border-bottom:1px solid var(--border)}
.mo-tbl td{text-align:center;padding:1px}
.day{width:100%;padding:5px 0;font-size:.8rem;color:var(--border2);background:transparent;
  border:none;cursor:default;border-radius:5px;font-weight:500;line-height:1;transition:background .15s,color .15s}
.day.hd{color:var(--p1);background:rgba(79,70,229,.09);cursor:pointer;font-weight:700}
.day.hd:hover{background:rgba(79,70,229,.22)}
.day.sel{background:var(--p1)!important;color:#fff!important;box-shadow:0 2px 6px rgba(79,70,229,.4)}
.day.tdy{outline:2px solid var(--warn);outline-offset:-2px}
.fp{margin-top:14px;border:1px solid var(--border);border-radius:var(--rs);overflow:hidden;display:none}
.fp-hdr{padding:10px 15px;background:var(--card2);border-bottom:1px solid var(--border);
  display:flex;align-items:center;justify-content:space-between;gap:10px}
.fp-hdr h4{font-size:.86rem;font-weight:700;color:var(--txt)}
.f-cnt{background:var(--p1);color:#fff;padding:2px 9px;border-radius:999px;font-size:.7rem;font-weight:700}
.f-list{padding:8px 10px;display:flex;flex-direction:column;gap:6px;max-height:240px;overflow-y:auto}
.f-list::-webkit-scrollbar{width:4px}
.f-list::-webkit-scrollbar-thumb{background:var(--border2);border-radius:3px}
.f-row{display:flex;align-items:center;gap:10px;padding:9px 11px;border-radius:7px;
  border:1.5px solid var(--border);background:var(--card);cursor:pointer;transition:border-color .15s,background .15s}
.f-row:hover{border-color:var(--p1);background:var(--bg2)}
.f-row.fck{border-color:var(--p1);background:rgba(79,70,229,.06)}
.f-row input[type=checkbox]{width:15px;height:15px;accent-color:var(--p1);flex-shrink:0;pointer-events:none}
.f-badge{font-size:.62rem;font-weight:800;color:var(--p1);background:rgba(79,70,229,.1);
  border:1px solid rgba(79,70,229,.25);padding:2px 5px;border-radius:4px;flex-shrink:0}
.f-name{flex:1;font-size:.83rem;font-weight:600;color:var(--txt);word-break:break-all}
.f-date{font-size:.7rem;color:var(--txt3);flex-shrink:0}
.f-empty{text-align:center;color:var(--txt3);font-size:.86rem;padding:18px}
.fp-foot{padding:11px 13px;border-top:1px solid var(--border);background:var(--card2)}
.sel-all{display:flex;align-items:center;gap:8px;margin-bottom:9px;font-size:.8rem;font-weight:600;color:var(--txt2);cursor:pointer}
.sel-all input{width:14px;height:14px;accent-color:var(--p1)}
.foot{padding:14px 13px 6px;text-align:center}
.copy{font-size:.78rem;color:var(--txt3)}
.copy strong{color:var(--p1)}
.mon-slave-block{margin-bottom:22px}
.mon-slave-block:last-child{margin-bottom:0}
.mon-slave-title{display:flex;align-items:center;gap:10px;margin-bottom:10px;
  padding-bottom:8px;border-bottom:2px solid var(--border)}
.mon-slave-tag{padding:3px 11px;border-radius:999px;font-size:.76rem;font-weight:800;
  color:#fff;background:var(--grad-btn)}
.mon-slave-name{font-size:.9rem;font-weight:700;color:var(--txt)}
.mon-tbl-wrap{overflow-x:auto;overflow-y:auto;max-height:260px;margin-bottom:14px;
  border-radius:8px;border:1px solid var(--border)}
.mon-tbl{width:100%;border-collapse:collapse;font-size:.78rem;min-width:320px}
.mon-tbl th{padding:7px 10px;background:var(--grad-btn);color:#fff;font-weight:700;
  text-align:center;white-space:nowrap;font-size:.72rem;letter-spacing:.04em}
.mon-tbl th:first-child{text-align:left;border-radius:7px 0 0 0}
.mon-tbl th:last-child{border-radius:0 7px 0 0}
.mon-tbl td{padding:6px 10px;border-bottom:1px solid var(--border);
  text-align:center;color:var(--txt2);font-variant-numeric:tabular-nums}
.mon-tbl td:first-child{text-align:left;color:var(--txt3);font-size:.71rem;white-space:nowrap}
.mon-tbl tr:last-child td{border-bottom:none}
.mon-tbl tr:nth-child(even) td{background:var(--bg2)}
.mon-tbl tr.latest td{background:rgba(79,70,229,.07);font-weight:600;color:var(--txt)}
.mon-tbl tr.latest td:first-child{color:var(--p1)}
.chart-wrap{position:relative;width:100%;background:var(--card2);border-radius:10px;
  border:1px solid var(--border);padding:14px 14px 8px;margin-bottom:6px;overflow:hidden}
.chart-title{font-size:.78rem;font-weight:700;color:var(--txt2);margin-bottom:10px;
  display:flex;align-items:center;justify-content:space-between}
.chart-legend{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px;padding-top:8px;border-top:1px solid var(--border)}
.chart-legend-item{display:flex;align-items:center;gap:5px;font-size:.7rem;font-weight:600;color:var(--txt2)}
.legend-dot{width:10px;height:10px;border-radius:50%;flex-shrink:0}
canvas.mon-canvas{width:100%!important;display:block}
.hist-meta{font-size:.75rem;color:var(--txt3);margin-bottom:10px;padding:6px 10px;
  background:var(--bg2);border-radius:6px;border:1px solid var(--border)}
@media(max-width:600px){
  .hdr{padding:22px 14px 20px}
  .hdr-title{font-size:1.3rem}
  .main{padding:12px 10px;gap:11px}
  .slv-grid{grid-template-columns:1fr}
  .g2{grid-template-columns:1fr}
  .g3{grid-template-columns:1fr 1fr}
  .g4{grid-template-columns:1fr 1fr}
  .led-cell{padding:10px;min-width:80px}
  .tbox{flex-direction:column;align-items:flex-start}
  .cal-grid{grid-template-columns:1fr}
  .sc-body{padding:13px 12px}
  .mon-tbl th,.mon-tbl td{padding:5px 7px;font-size:.71rem}
}
@media(max-width:400px){
  .g3{grid-template-columns:1fr}
  .g4{grid-template-columns:1fr 1fr}
  .hdr-title{font-size:1.15rem}
}
@media(min-width:900px){
  .slv-grid{grid-template-columns:1fr 1fr}
}
</style>
</head>
<body>
<div class="app">
)CHUNK2"));

  // ---- CHUNK 3: Header + LED strip + Status + Error box ----
  server.sendContent(F(R"CHUNK3(<!-- HEADER -->
<div class="hdr">
  <button class="theme-btn" onclick="toggleTheme()" id="themeBtn">
    <span id="themeIco">&#x1F319;</span>
    <span id="themeTxt">Dark Mode</span>
  </button>
  <div class="hdr-row">
    <div class="hdr-logo">
      <img src="data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAUDBAQEAwUEBAQFBQUGBwwIBwcHBw8LCwkMEQ8SEhEPERETFhwXExQaFRERGCEYGh0dHx8fExciJCIeJBweHx7/2wBDAQUFBQcGBw4ICA4eFBEUHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh7/wAARCAC6A+gDASIAAhEBAxEB/8QAHQABAAEEAwEAAAAAAAAAAAAAAAcBBQYIAgQJA//EAF8QAAEDAwEEBAoDCwkEBgYLAAEAAgMEBREGBxIhMRNBUdEIFxgiVmFxgZGVFFWhFRYyMzZCUmJ0sbIjN1NykpOUs8Ekc8LSNEN1goTDJSY1OEVjJyhGVGWio6S04fD/xAAbAQEAAgMBAQAAAAAAAAAAAAAAAQIEBQYDB//EADwRAAEDAgQDBAcHAwUBAQAAAAEAAgMEEQUSITEGQVETFGFxIoGRobHB0RUWMlJT4fA0NUIjMzZy8UNE/9oADAMBAAIRAxEAPwDcpERWVUREREREUIiIilERERERERERERERERERERERERERERERERERERERERERERERERERERERERERUzxVVCIiIpRERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERWjVV+pNO2p1zroaqSnY4CR0EW+WA9ZHZ61h42y6N65K8f+FPepCqoYqinkhmjbJG9pa9jhkOB5gha0bW9DS6UugqqNj3Wipcehdz6F39G4/uPWPYtrhdPTVL+zlJB5LmuIa3EKFgmpwC3npqPFSuNsei//vFd/hXLm3bDoonjVVg9tK5a15Rb/wC71L1P89S44ca4h0b7D9Vsy3a7od3O5TN9tM/uXMbWNCnneHD208n/ACrXfT+n7vf5pYLPRmrljaHPYJGtcB24cRw9ivXi01z6PT/3sf8AzLFkwigjdldJY+YWdDxPi8zc0cNx1AKnHxraE+vP/wBtL/yr6M2o6FeOF+iHtikH72qCTs11z6PT/wB7H/zKh2b64B/J6o/vI/8AmXn9l4f+t7wvYcRYzzp/cVPjdpWiHctQ0g9u8P8ARcxtE0Sf/tHQe9+P9FAHi41xn8nqn+8j/wCZPFxrn0eqf7xn/Mn2VQfre8K44kxcf/m9xWwTdoOizy1Lbv70L6DXmjTy1LbP78LWq7aP1Tao+lr7FXRR4zviPfaPaW5wrFnsIXozAaaQXZIT7Fjy8ZV8JtLAB53C23p9Z6UqJmQQahtsksjg1jGztJcTyAV9HatO9NP6LUlrkz+DWwn/APUatxBz9S1OKYeKJzQ03uul4exp+Kse57QMpGy5IiLVrokRERERERERERERERERERERERERUK+VVOymp5J5ntjjjaXPc44DQBkkohIAuV9C4BdO5Xe121m/X3ClpRjP8rKG/vULam2i6i1be/uDomOSCF5LWzM4Sygc3ZP4DfXz/crtZdi1PKRVamvFVWVTvOe2F2Bn+u7Lj7eC2PcGQgOqX5b8hqf2XPjGpap5ZQxZwP8AImzf3Weff5o7e3fvktmf9+Fc7dfrLcgPufdaKqzyEUzXH4ZWGSbHdFujLW09aw4xvCqdn7eCwbW+x2rtdLJcdPVUlayMFzqeQAS7o/RI/C9nAq7KehlOVshB8R9FSeuxemaZHwtcB+Um/vWeadvt1qtsd+ss9Y59vpqVr4YS0YafM45xnrKkELX/AMHOWafW9wlqJZJZDQYL5HFzuD2gcTxWwAXjiUIhnyDkB8FkcP1TqukMzubnb8tVVERYK3aIiIiIiIiIqZVURERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERUK6F9tVFebXPbbhC2amnbuvaf3jsI5grvphASDcKr2Ne0tcLgrUvX+la3Sd9fQVAdJTvy6mnxgSs7xyI71j3WttddaXt+qbFLbq0brvwoJgPOif1OH+o6wtWtQ2eusN3ntlxiLJ4Tg9jh1Ob2g9S7jCcSFUzI78Q96+RcR4E7Dpc8f+27bw8PouFiutbZLrBc7fMYqiB2WnqI62kdYPWFtNoLVVDqyxR3Cl8yVvmVEBOXRP6x7Ow9YWpmFftDanrtKX2O5UhL4z5tRDnAlZ2e3rB6ipxXDRVszN/GP5ZRw7jjsNmyPN43b+Hitt0Vt05eaG+2iC526YS08zcjtB6wR1EHqVzXDuBabHdfXo5GyND2m4Koiqiqrri5ocOKwLaFs2s+o6SWekgioboGksnjbgPPY8DmD28ws/XFwXrDNJA4PYbFYtVRw1cZjmbcFaZuZPQXAMlYYp6aYBzDza5ruI+IW5UB3oWO7WgrWTbXaHWraFWubEWw1pbUxHqcXYDsf97PxWy1tLjb6cvBa7om5B6jgLd43KJo4ZRzC5HhKndS1FTAf8SPmuyioqrQLt0REREREREREREREREREREREREUbeELdJqDQwpYXFprqhsLyP0MFxHvxhSQVHXhAWia5aGNTA0ufQTNqHNAySzi13wBz7llUGXvTM+11q8bznD5RHvYq0+DbbqVmm666BgdVTVRic7rDGgYHxJKlsLXHYhrWn03c5rZcpBHb61wcJTyikAwCf1SOBPVgLYqCVksTZI3texwyHNOQR2hZWMQyR1TnO2OywOFqqGXD2Mj3boR4/uvquLhkKqHktUujUR7P6KK3bdNT00DWtj6AvDQMAbzmOP2kqXFFulOG37U3DP+xs/wCBSkFm15JkaT+UfBafBGhsLwPzu+KqiIsNbhERERERERYZtL13SaMhphJSSVlTUk9HE1waMNxkl3HHMdS6uzXaNR6wqp6IUMlFVws6TcLw9r2ZxkHA4gnlhd3aLoWg1nHTGpqpqWel3ujliAPB2Mgg8+QXw2dbO7Zo+eoqoaqesq5mBhlkAaGtznAA7T+5bAGj7rY37RaFwxX7RuLdj6v/AHdZsqqgVVrgt+iIilQiIiIiIiIiIihERFQoiqiplVREREUoiIiIiIiIiIiIiIiIiIiIiIiIiIiIiIiIiKmVVQiIiKUREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREREVFhO1fREGrbRvwBkV0pmk00p/O7WO9R+w8Vm6oRlekMr4Xh7DYhY9VSx1UToZRcFaX1dNPSVUtLVRPhnicWSMeMFrhzBXyPILYHbZoH7tUz79aIR90oGfy0bR/wBIYP8AiHV2jh2LX4jBxyXfYfXMrIsw3G4XxnGMJkwyoMbvwnY9Qs02Va3n0ldtycuktVS4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Po3D3twvBpowbEO89Ph+6zHDFiMzSwHpr8f2VyseorJe49+1XOlqxjJbG8Fw9o5hXPeB4LXbadoGfRUsV+sFbUikEgaXb+Jadx5ecObSeH71n2xjX8upIX2i7Ob91Kdm8JBw6dnLOP0hwz8V6z4eBD3iB2ZvvCxaLG3Gp7nVsyScuh8lJiqqBVWsXRIiIpRERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERcXDIwoL25aA+iSTans8J6B53q2Fg/AP8ASD1Hr+Pap1K4TRMmidHI1r2OBa5rhkEHmFk0dW+klD2evxWtxXDIsRgMUm/I9CtLVRSBtf0I/S9xNfQMcbRUv8zh+IefzD6uw+7q44AvoNNUMqYxIw6FfF62ilopzDKLEKcdhuv/AKQ2PTF6n/l2jdop3n8YB/1ZPaBy7R61MgIPFaw7JtFVeqb0ypeZYLbSSB00zSWlzhxDGnt7T1D3LZ1gw0DOVxWMxQx1B7M+Y6FfVOFamqnogagaDY9QuSIi1S6ZERERERERERERERERERERERERERERU5K06n1BatO2x1fdapsMQ4NHNzz+i0cyV27xX09rtlTcat4ZBTxukkd2ADKgrStNV7VdezXO8h/3JoeIpwfNa0nzI/fjLj14wsykpRKHSSGzG7/RanE8RdTOZBCLyP2HTxPkr83VWv8AXEhGkqBtotm9uisnI3nDtBII9zQfavs3ZHdLkBLqHWdfVS891mS0f2j/AKBSvTQQ08LIYI2RxsADGsGA0eoL64VjXuZpC0NHv9q8mYKyUZqt5kd52HqAURv2H23cJhv9xY/qO61WO76c2kaFhNfaL7PcaGEEva0l+40dsbs8PYp4XF4BBBHBSzFJgf8AUs4dCFWXh6kLbw3Y7kQSoq2c7Wqe8VcNrv0MdHVyENinYf5KR3UCD+CT8FIGodPWbUNJ9HutBDUs/NLm4c31hw4j3LW3a3a6W0a/uNJQhrIXOZM1rfzC9uSB2cePwWw2zS5VF30LabhVHM8tOA8n84t83PvxlZeI0rIWMqYNA73Fa7A6+WqkloKyzizn1CiPXex+4W5sldpyR9fTDiaZ/wCOaPV1PHq5+1YXo3Vt80hcHPoZXdFv4npJc7j8cwR+a718/atsiM81Fm2/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qx6Q2b6J0lVvrNP6fpaOpcC0z5dJIAeYDnkkD1BerKpgAJGov5KDGSdFFFyyPDRtm8d4i2DJ9fQPVs0Jqay6J8IvaAzVFdHbI615MM0+Qw+eHgZ9bXZHsWwEmmbA/UjNSOtFEbyxnRsrjEOma3BGA7njBIVs1js80bq+eOfUVhpa6eNu62bzmSBvZvMIJHqJVRUNPouGlrIYyNR1UP7Aa+n1F4QmvdS2wumtk0e7FNukB2ZGBp9/Rk+zC6HgyD/1c2n5/pH4/sTLYHS2mLDpe3fc7T9sprdTb28WQtxvO/ScTxcfWSVwsuktN2WGthtFkoKGOvOatsEIaJ+Y87HPmfiUdUtOYAb29yCMi3rWqul6K4Vngg31lBHI8w3ps07Wc+iaIi446wOBPqCya5652ZSeDUywxyUZugtrIW0IixM2qGMyZxw84F2/niPatitPabsWnrc+22S00duo5Hl74KeIMY5xABJHrAAWNR7Idm0d5+6zNIW4VQdvjzXGIO7ejzufYr96Y4nMDvdR2ZAWuu0Ogr6TwbNn1Nc2Pa59wkeyN4wWxP3y0Ef1Tn2FSH4YdDSUuyqxRUsDImU1fHHC1owGN6FwwPVwCmrUWmbBqGmhpr7aKK5QwP6SFlTEHtY7GMgHkcLlqPTli1HQx0V9tNHcqaN4kZFUxB7WuAIBAPXglUFUMzSRsSfap7LQhQltNsNjr9V2G5WfaBS6X1vT2uMxNnkxG+MM4FzjwYSHOHHO8M8DhZL4NmvL3rK2Xijv0dJJVWmqbEa2laGx1O9vZPDgXZaTkYBDgcBZnq7Z5ovVjIG6g0/R1pp2hkTyCx7Gjk0OaQcerOFdNL6dsmmLY22WC209vo2nPRQtwCe0nmT6zxVHTNdFl5/BSGkOuruiIsZeiIiKURERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERUVUREREUIiIilERERERERERERERERERERERERERERERERERERERERERERFRVRERURVRERERERERERERERERERERERERERERERERQpRERSoREREREREREREVFVERERERF//2Q==" alt="Sigma Controls" width="1000" height="186" />
    </div>
    <div class="hdr-title">Sigma Controls Data Logger</div>
    <div class="hdr-sub">Industrial Multi-Channel Data Acquisition System</div>
    <div class="hdr-badge" id="ltBadgeHdr">&#128268; TempScanner</div>
  </div>
</div>

<!-- LED STRIP -->
<div class="led-strip">
  <div class="led-cell">
    <div class="led led-pwr"></div>
    <div class="led-label">Power<b>ALWAYS ON</b></div>
  </div>
  <div class="led-cell">
    <div class="led led-comm-off" id="ledComm"><div class="led-ring"></div></div>
    <div class="led-label">COMM / Modbus<b id="ledCommLbl">OK</b></div>
  </div>
  <div class="led-cell">
    <div class="led led-sd-off" id="ledSD"><div class="led-ring"></div></div>
    <div class="led-label">SD Card<b id="ledSDlbl">OK</b></div>
  </div>
</div>

<!-- MAIN -->
<div class="main">

  <!-- STATUS -->
  <div class="st st-idle" id="stCard">
    <div class="st-ico" id="stIco">&#x26AA;</div>
    <div>
      <div class="st-tt" id="stTt">System Ready</div>
      <div class="st-ts" id="stTs">Configure settings below, then tap Start Logging</div>
    </div>
  </div>

  <!-- ERROR BOX -->
  <div class="err-box" id="errBox">
    <span style="font-size:1.1rem;flex-shrink:0">&#9888;</span>
    <span id="errTxt"></span>
  </div>

  <!-- FIRMWARE / OTA (native details = reliable open/close on all phones) -->
  <details class="sc" id="fwCard" style="margin-bottom:12px">
    <summary class="sc-hdr" style="list-style:none;cursor:pointer;-webkit-user-select:none;user-select:none">
      <div class="sc-ico ico-b">&#8679;</div>
      <div class="sc-meta">
        <h3>Firmware &amp; Updates</h3>
        <p id="fwHdrSub">v— · Tap to expand</p>
      </div>
      <div class="sc-arr" id="arr-fw">&#9660;</div>
    </summary>
    <div class="sc-body" id="body-fw">
      <div class="field" style="margin-bottom:10px">
        <label>Installed version</label>
        <div id="fwLabel" style="font-weight:800;font-size:1.05rem;color:var(--txt);padding:6px 0">—</div>
      </div>
      <div class="field" style="margin-bottom:10px">
        <label>Last check result</label>
        <div id="otaMsg" style="font-size:.85rem;font-weight:600;color:var(--txt2);padding:6px 0;line-height:1.4">Not checked yet</div>
      </div>
      <div class="field-hint" style="margin-bottom:12px">
        Updates only on <b>home WiFi</b> with internet. <b>Stop logging</b> first.
        SoftAP recovery: <b>http://192.168.4.1</b>. Bad update can roll back to previous firmware.
      </div>
      <div class="btn-grp">
        <button type="button" class="btn btn-start" id="otaBtn" onclick="checkOta()">Check for update</button>
      </div>
    </div>
  </details>

  <!-- NETWORK / STATION IP BANNER (SoftAP only) -->
  <div id="netBanner" style="display:none;background:linear-gradient(135deg,#0369a1,#0ea5e9);border-radius:12px;padding:14px 16px;color:#fff;margin-bottom:4px">
    <div style="font-weight:800;font-size:.95rem;margin-bottom:6px">&#127760; Daily access on your WiFi</div>
    <div id="staIpTxt" style="font-size:.82rem;opacity:.95;line-height:1.5;margin-bottom:12px;word-break:break-all"></div>
    <div style="display:flex;flex-wrap:wrap;gap:8px;align-items:center">
      <button type="button" id="staGuideBtn"
              style="background:#fff;color:#0369a1;font-weight:800;padding:10px 16px;border-radius:8px;border:none;font-size:.88rem;cursor:pointer;font-family:inherit">
        How to open daily link
      </button>
      <button type="button" id="staCopyBtn"
              style="background:rgba(255,255,255,.2);border:1.5px solid rgba(255,255,255,.5);color:#fff;font-weight:700;padding:10px 14px;border-radius:8px;font-size:.82rem;cursor:pointer;font-family:inherit">
        Copy daily link
      </button>
    </div>
    <div style="font-size:.72rem;margin-top:10px;opacity:.9">You are on setup WiFi <b>Sigma_Controls</b>. Daily use needs your home/office WiFi first.</div>
  </div>

  <!-- WiFi switch guide modal (all devices) -->
  <div id="wifiGuideModal" style="display:none;position:fixed;inset:0;z-index:10000;background:rgba(15,12,40,.55);align-items:center;justify-content:center;padding:16px">
    <div style="background:var(--card,#fff);color:var(--txt,#1e1b4b);border-radius:16px;max-width:420px;width:100%;padding:20px 18px;box-shadow:0 20px 50px rgba(0,0,0,.35);font-family:inherit">
      <div style="font-size:1.05rem;font-weight:800;margin-bottom:8px">Switch WiFi, then open the link</div>
      <div style="font-size:.85rem;line-height:1.55;color:var(--txt2,#4b5280);margin-bottom:14px">
        The daily address only works on <b>your</b> network — not on SoftAP.
      </div>
      <ol style="margin:0 0 14px 18px;padding:0;font-size:.88rem;line-height:1.65;color:var(--txt,#1e1b4b)">
        <li style="margin-bottom:8px">Open phone <b>WiFi settings</b></li>
        <li style="margin-bottom:8px">Leave <b>Sigma_Controls</b></li>
        <li style="margin-bottom:8px">Join <b id="guideSsid">your WiFi</b></li>
        <li style="margin-bottom:8px">Return here or open the daily link below</li>
      </ol>
      <div style="background:var(--bg2,#eaecfb);border-radius:10px;padding:12px;margin-bottom:14px;word-break:break-all">
        <div style="font-size:.72rem;font-weight:700;opacity:.7;margin-bottom:4px">DAILY LINK</div>
        <div id="guideUrl" style="font-weight:800;font-size:.95rem;color:#4f46e5">http://192.168.1.200</div>
      </div>
      <div style="display:flex;flex-wrap:wrap;gap:8px">
        <button type="button" id="guideCopyBtn" class="btn btn-start" style="flex:1;min-width:120px">Copy link</button>
        <button type="button" id="guideOpenBtn" class="btn btn-start" style="flex:1;min-width:120px">I switched — Open</button>
        <button type="button" id="guideCloseBtn" class="btn" style="flex:1;min-width:120px;background:var(--bg2,#eaecfb);color:var(--txt,#1e1b4b)">Close</button>
      </div>
    </div>
  </div>


  <!-- WIFI SETUP (SoftAP only — hidden on Station IP) -->
  <div class="sc" id="wifiCard" style="display:none">
    <div class="sc-hdr" onclick="tog('wifi')">
      <div class="sc-ico ico-b">&#128246;</div>
      <div class="sc-meta"><h3>WiFi Setup</h3><p>Home / office network for Station mode — SoftAP stays at 192.168.4.1</p></div>
      <div class="sc-arr open" id="arr-wifi">&#9660;</div>
    </div>
    <div class="sc-body" id="body-wifi">
      <div class="tbox" style="margin-bottom:14px">
        <div>
          <div class="tb-lbl">Current Station SSID</div>
          <div class="tb-val" id="curStaSsid" style="font-size:1rem">—</div>
        </div>
        <div>
          <div class="tb-lbl">SoftAP (QR)</div>
          <div class="tb-val" style="font-size:.9rem">192.168.4.1</div>
        </div>
      </div>
      <div class="g2">
        <div class="field">
          <label for="newSsid">WiFi Name (SSID)</label>
          <input type="text" id="newSsid" maxlength="32" placeholder="e.g. MyHome_2.4G" autocomplete="off">
          <div class="field-hint">Must be a 2.4 GHz network (ESP32 does not support 5 GHz)</div>
        </div>
        <div class="field">
          <label for="newPass">WiFi Password</label>
          <input type="password" id="newPass" maxlength="64" placeholder="Network password" autocomplete="off">
          <div class="field-hint">Leave blank only if the network is open</div>
        </div>
      </div>
      <div class="field">
        <label style="display:flex;align-items:center;gap:8px;cursor:pointer">
          <input type="checkbox" id="useStatic" checked style="width:auto;margin:0">
          Use fixed Station IP (auto: same subnet as router, host .200)
        </label>
        <div class="field-hint">Device learns network once, then stays on <b>x.x.x.200</b> every day — no manual change</div>
      </div>
      <div class="g2" id="staticFields">
        <div class="field">
          <label for="staticIP">Station IP</label>
          <input type="text" id="staticIP" value="192.168.1.200" placeholder="192.168.1.200">
          <div class="field-hint">Auto-filled after first connection — usually ends with .200</div>
        </div>
        <div class="field">
          <label for="gatewayIP">Gateway (router)</label>
          <input type="text" id="gatewayIP" value="192.168.1.1" placeholder="192.168.1.1">
          <div class="field-hint">Usually your router IP</div>
        </div>
      </div>
      <div class="field">
        <label for="subnetMask">Subnet mask</label>
        <input type="text" id="subnetMask" value="255.255.255.0" placeholder="255.255.255.0">
      </div>
      <div class="btn-grp">
        <button type="button" class="btn btn-start" id="saveWifiBtn" onclick="saveWifi()">
          <svg viewBox="0 0 24 24" fill="currentColor" width="18" height="18"><path d="M12 3C7.46 3 3.34 4.67.29 7.29l1.42 1.42C4.33 6.23 7.89 5 12 5s7.67 1.23 10.29 3.71l1.42-1.42C20.66 4.67 16.54 3 12 3zm0 4c-3.12 0-5.95 1.17-8.12 3.09l1.42 1.42C7.17 9.76 9.44 9 12 9s4.83.76 6.7 2.51l1.42-1.42C17.95 8.17 15.12 7 12 7zm0 4c-1.85 0-3.55.73-4.8 1.9l1.41 1.41C9.47 13.45 10.68 13 12 13s2.53.45 3.39 1.31l1.41-1.41C15.55 11.73 13.85 11 12 11zm0 4c-.83 0-1.5.67-1.5 1.5S11.17 18 12 18s1.5-.67 1.5-1.5S12.83 15 12 15z"/></svg>
          Save WiFi &amp; Connect
        </button>
      </div>
      <div class="log-info" id="wifiMsg" style="display:none;margin-top:12px"></div>
    </div>
  </div>

  <!-- TIMING & DISPLAY -->
  <div class="sc" id="card-timing">
    <div class="sc-hdr" onclick="tog('timing')">
      <div class="sc-ico ico-v">&#128336;</div>
      <div class="sc-meta"><h3>Timing &amp; Display</h3><p>Interval, division factor, unit</p></div>
      <div class="sc-arr open" id="arr-timing">&#9660;</div>
    </div>
    <div class="sc-body" id="body-timing">
      <div class="tbox">
        <div><div class="tb-lbl">Date</div><div class="tb-val" id="dispDate">--</div></div>
        <div><div class="tb-lbl">Time</div><div class="tb-val" id="dispTime">--</div></div>
      </div>
      <div class="g2">
        <div class="field">
          <label>Logging Interval (seconds)</label>
          <input type="number" id="interval" min="1" max="3600" value="5">
          <div class="field-hint">Range: 1 &ndash; 3600 s</div>
        </div>
        <div class="field">
          <label>Division Factor</label>
          <input type="number" id="divFactor" min="0.001" max="100000" step="any" value="10">
          <div class="field-hint">Raw &divide; factor = displayed value. Values &gt;2000 set to 0.</div>
        </div>
      </div>
      <div class="field">
        <label>Display Unit (graphs &amp; tables only &mdash; not saved in CSV)</label>
        <input type="text" id="unitField" maxlength="20" placeholder="e.g. &#176;C, bar, kg/cm&#178;, RPM">
      </div>
    </div>
  </div>
)CHUNK3"));

  // ---- CHUNK 4: Device + Communication card (baud + new serial settings) ----
  server.sendContent(F(R"CHUNK4(  <!-- DEVICE & COMMUNICATION -->
  <div class="sc" id="card-device">
    <div class="sc-hdr" onclick="tog('device')">
      <div class="sc-ico ico-b">&#9881;</div>
      <div class="sc-meta"><h3>Device &amp; Communication</h3><p>RS-485 Modbus RTU — baud rate, parity, stop bits, byte size, timeout</p></div>
      <div class="sc-arr" id="arr-device">&#9660;</div>
    </div>
    <div class="sc-body hide" id="body-device" style="display:none">
      <div class="g2">
        <div class="field">
          <label>RS-485 Baud Rate</label>
          <select id="baudrate">
            <option value="1200">1200 bps</option>
            <option value="2400">2400 bps</option>
            <option value="4800">4800 bps</option>
            <option value="9600" selected>9600 bps</option>
            <option value="19200">19200 bps</option>
            <option value="38400">38400 bps</option>
            <option value="57600">57600 bps</option>
            <option value="115200">115200 bps</option>
          </select>
        </div>
        <div class="field">
          <label>Modbus Timeout (ms)</label>
          <input type="number" id="modbusTimeout" min="100" max="10000" step="100" value="1000">
          <div class="field-hint">Range: 100 &ndash; 10000 ms</div>
        </div>
      </div>
      <div class="g3">
        <div class="field">
          <label>Parity</label>
          <select id="parity">
            <option value="0" selected>None (N)</option>
            <option value="1">Even (E)</option>
            <option value="2">Odd (O)</option>
          </select>
        </div>
        <div class="field">
          <label>Stop Bits</label>
          <select id="stopBits">
            <option value="1" selected>1 Stop Bit</option>
            <option value="2">2 Stop Bits</option>
          </select>
        </div>
        <div class="field">
          <label>Byte Size (Data Bits)</label>
          <select id="byteSize">
            <option value="8" selected>8 Bits</option>
            <option value="7">7 Bits</option>
          </select>
        </div>
      </div>
      <!-- Live summary of selected serial config -->
      <div class="serial-summary" id="serialSummary">
        <span>&#128268; <b>Config preview:</b></span>
        <span id="serialPreview">9600 8N1 &mdash; Timeout: 1000 ms</span>
      </div>
    </div>
  </div>

  <!-- LOGGER TYPE -->
  <div class="sc" id="card-ltype">
    <div class="sc-hdr" onclick="tog('ltype')">
      <div class="sc-ico ico-p">&#127919;</div>
      <div class="sc-meta"><h3>Logger Type</h3><p>Locked after first start &mdash; edit config.json to reset</p></div>
      <div class="sc-arr open" id="arr-ltype">&#9660;</div>
    </div>
    <div class="sc-body" id="body-ltype">
      <div id="ltUnlocked">
        <div class="field">
          <label>Select Logger Type</label>
          <select id="ltSelect" onchange="onLtChange()">
            <option value="0">Temp Scanner (multi-channel temperature)</option>
            <option value="1">PID Controller (PV &amp; SV per slave)</option>
          </select>
        </div>
        <div class="field-hint" style="margin-top:4px">&#128274; Once logging starts this type locks permanently.</div>
      </div>
      <div id="ltLocked" style="display:none">
        <div class="lt-locked">
          <span class="lt-badge" id="ltBadgeTxt">--</span>
          <span class="lt-note">&#128274; Locked &mdash; delete config.json from SD card to reset</span>
        </div>
      </div>
    </div>
  </div>
)CHUNK4"));

  // ---- CHUNK 5: Slave configuration card (with dynamic count input) ----
  server.sendContent(F(R"CHUNK5(  <!-- SLAVE DEVICE CONFIGURATION -->
  <div class="sc" id="card-slaves">
    <div class="sc-hdr" onclick="tog('slaves')">
      <div class="sc-ico ico-t">&#128268;</div>
      <div class="sc-meta"><h3>Slave Device Configuration</h3><p>Enter the number of slaves &mdash; each gets its own toggle</p></div>
      <div class="sc-arr" id="arr-slaves">&#9660;</div>
    </div>
    <div class="sc-body hide" id="body-slaves" style="display:none">

      <!-- Number of slaves input — user types any number -->
      <div class="g2" style="margin-bottom:16px">
        <div class="field">
          <label>Number of Slave Devices</label>
          <input type="number" id="numSlaves" min="1" max="16" value="4"
                 oninput="onNumSlavesInput()"
                 onchange="onNumSlavesChange()">
          <div class="field-hint">Enter 1 &ndash; 16. Slave tiles appear instantly below.</div>
        </div>
        <div style="display:flex;align-items:flex-end;padding-bottom:2px">
          <div class="ns-display" id="nsDisplay">&#128268; 4 Slaves configured</div>
        </div>
      </div>

      <!-- Slave tiles rendered by JS -->
      <div class="slv-grid" id="slvGrid"></div>
    </div>
  </div>

  <!-- LOGGING CONTROLS -->
  <div class="sc" id="card-ctrl">
    <div class="sc-hdr" onclick="tog('ctrl')">
      <div class="sc-ico ico-g">&#9654;</div>
      <div class="sc-meta"><h3>Logging Controls</h3><p>Start or stop data recording</p></div>
      <div class="sc-arr open" id="arr-ctrl">&#9660;</div>
    </div>
    <div class="sc-body" id="body-ctrl">
      <div class="btn-grp">
        <button class="btn btn-start" id="startBtn" onclick="startLog()">
          <svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg>
          Start Logging
        </button>
        <button class="btn btn-stop" id="stopBtn" onclick="stopLog()" style="display:none">
          <svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg>
          Stop Logging
        </button>
      </div>
      <div class="log-info" id="logInfo">
        Logging active &mdash; WiFi can be safely disconnected.
        <span>Data is being saved automatically to the SD card.</span>
      </div>
    </div>
  </div>

)CHUNK5"));

  // ---- CHUNK 6: Live monitor section ----
  server.sendContent(F(R"CHUNK6(  <!-- LIVE MONITOR -->
  <div id="monitorSection" style="display:none">
    <div class="sc" id="card-mon10">
      <div class="sc-hdr" onclick="tog('mon10')">
        <div class="sc-ico ico-t">&#128200;</div>
        <div class="sc-meta"><h3>Live Monitor &mdash; Last 10 Readings</h3><p>Most recent values from all active slaves</p></div>
        <div class="sc-arr open" id="arr-mon10">&#9660;</div>
      </div>
      <div class="sc-body" id="body-mon10">
        <div id="mon10Content"><div class="f-empty">Waiting for data&hellip;</div></div>
      </div>
    </div>
    <div class="sc" id="card-monHist">
      <div class="sc-hdr" onclick="tog('monHist')">
        <div class="sc-ico ico-o">&#128202;</div>
        <div class="sc-meta"><h3>Live Monitor &mdash; Full History</h3><p>Paginated view &mdash; 200 rows/page</p></div>
        <div class="sc-arr" id="arr-monHist">&#9660;</div>
      </div>
      <div class="sc-body hide" id="body-monHist" style="display:none">
        <div class="field" style="margin-bottom:14px">
          <label>Select Slave to View</label>
          <select id="histSlave" onchange="histPage=0;loadHistory()">
            <option value="0">Slave 1</option>
          </select>
        </div>
        <div id="monHistContent"><div class="f-empty">Waiting for data&hellip;</div></div>
        <div id="histPager" style="display:none;margin-top:12px;display:flex;gap:8px;align-items:center;flex-wrap:wrap">
          <button class="btn btn-start" style="width:auto;padding:8px 16px;font-size:.82rem" onclick="histPrev()">&#8592; Prev</button>
          <span id="histPageInfo" style="font-size:.82rem;color:var(--txt2);flex:1;text-align:center"></span>
          <button class="btn btn-start" style="width:auto;padding:8px 16px;font-size:.82rem" onclick="histNext()">Next &#8594;</button>
        </div>
      </div>
    </div>
  </div>

  <!-- DOWNLOAD -->
  <div class="sc" id="card-dl">
    <div class="sc-hdr" onclick="tog('dl')">
      <div class="sc-ico ico-o">&#8659;</div>
      <div class="sc-meta"><h3>Download Data</h3><p>Browse SD card files by date &amp; download CSV</p></div>
      <div class="sc-arr" id="arr-dl">&#9660;</div>
    </div>
    <div class="sc-body hide" id="body-dl" style="display:none">
      <div class="cal-legend">
        <i><span class="leg leg-h"></span>Has data</i>
        <i><span class="leg leg-s"></span>Selected</i>
        <i><span class="leg leg-e"></span>No data</i>
      </div>
      <div class="cal-grid" id="calGrid"></div>
      <div class="fp" id="fp">
        <div class="fp-hdr">
          <h4>Files for selected dates</h4>
          <span class="f-cnt" id="fCnt">0</span>
        </div>
        <div class="f-list" id="fList"></div>
        <div class="fp-foot">
          <label class="sel-all" onclick="toggleAll()">
            <input type="checkbox" id="selAll" checked> Select / Deselect All
          </label>
          <button class="btn btn-dl" onclick="dlAll()">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
              <line x1="12" y1="3" x2="12" y2="15"/><polyline points="7 10 12 15 17 10"/><line x1="4" y1="20" x2="20" y2="20"/>
            </svg>
            Download Selected Files
          </button>
          <div class="btn-zip-note">&#9432; ZIP bundles all selected CSV files &mdash; works offline</div>
          <button class="btn btn-zip" id="zipBtn" onclick="dlZip()" disabled
                  title="Select 2 or more files to enable ZIP download">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
              <line x1="12" y1="3" x2="12" y2="15"/><polyline points="7 10 12 15 17 10"/><line x1="4" y1="20" x2="20" y2="20"/>
            </svg>
            Download as ZIP
            <span id="zipStatusDot" style="
              display:inline-block;width:8px;height:8px;border-radius:50%;
              background:#ef4444;margin-left:6px;flex-shrink:0;
              transition:background .3s;" title="No internet"></span>
          </button>
        </div>
      </div>
    </div>
  </div>

</div><!-- /main -->

<div class="foot">
  <div class="copy">Powered by <strong>Sigma Controls</strong></div>
</div>

</div><!-- /app -->
)CHUNK6"));

  // ---- CHUNK 7: JavaScript — state, init, clock, slave grid builder ----
  server.sendContent(F(R"CHUNK7(<script>
// ============================================================
// STATE
// ============================================================
var isLog        = false;
var selDts       = new Set();
var avDts        = new Set();
var fmap         = {};
// Defaults: open = Logger Type, Timing, Logging Controls, Live last-10
  //           closed = Device, Slaves, Live full, Download, Firmware
  var secOpen      = {fw:false, timing:true, device:false, ltype:true, slaves:false, ctrl:true,
                    dl:false, mon10:true, monHist:false, wifi:true};
var isDark       = false;
var ltLocked     = false;
var ltType       = 0;
var monInterval  = null;
var histPage     = 0;
var histTotalPages = 1;
var charts10     = {};
var chartsHist   = {};
var currentUnit  = '';
// numSlaves is the live count of slave tiles currently rendered
var numSlaves    = 4;

// ============================================================
// INIT
// ============================================================
window.addEventListener('load', function() {
  // Menu order + open/close defaults (names unchanged)
  // 1 Logger Type open | 2 Timing open | 3 Device closed | 4 Slaves closed
  // 5 Logging open | 6 Live last-10 open | 7 Live full closed | 8 Download closed | 9 Firmware closed
  (function layoutSections() {
    var main = document.querySelector('.main');
    if (!main) return;
    var order = ['card-ltype', 'card-timing', 'card-device', 'card-slaves', 'card-ctrl',
                 'monitorSection', 'card-dl', 'fwCard'];
    // Insert after status/error/wifi/net banner blocks — append in order at end of known anchors
    var anchor = document.getElementById('wifiCard') || document.getElementById('errBox') || document.getElementById('stCard');
    order.forEach(function(id) {
      var el = document.getElementById(id);
      if (!el) return;
      if (anchor && anchor.parentNode === main) {
        if (anchor.nextSibling) main.insertBefore(el, anchor.nextSibling);
        else main.appendChild(el);
        anchor = el;
      } else {
        main.appendChild(el);
      }
    });
    // Apply open/closed from secOpen
    ['timing','device','ltype','slaves','ctrl','dl','mon10','monHist'].forEach(function(k) {
      var b = document.getElementById('body-' + k);
      var a = document.getElementById('arr-' + k);
      if (!b) return;
      if (secOpen[k]) {
        b.classList.remove('hide');
        b.style.display = 'block';
        if (a) a.classList.add('open');
      } else {
        b.classList.add('hide');
        b.style.display = 'none';
        if (a) a.classList.remove('open');
      }
    });
    // Firmware closed by default (details element)
    var fw = document.getElementById('fwCard');
    if (fw) fw.open = false;
    // SoftAP (192.168.4.1): hide Firmware & Updates entirely
    var host = location.hostname || '';
    if (host === '192.168.4.1' || host.indexOf('192.168.4.') === 0) {
      if (fw) fw.style.display = 'none';
    }
  })();

  buildSlaveGrid(numSlaves);
  updateSerialPreview();
  tick();
  setInterval(tick, 250);   // smooth clock; advances locally between /status samples
  syncTime();
  setInterval(pollSt,  2000);  // status can be slower — clock does not depend on it
  setInterval(pollErr, 2500);
  pollSt();
  pollErr();
  // Defer heavier SD reads so first paint is fast
  setTimeout(function(){ try { loadConfig(); } catch(e){} }, 50);
  setTimeout(function(){ try { loadDates(); } catch(e){} }, 150);

  // Attach live-update listeners for serial preview
  document.getElementById('baudrate').addEventListener('change',    updateSerialPreview);
  document.getElementById('parity').addEventListener('change',      updateSerialPreview);
  document.getElementById('stopBits').addEventListener('change',    updateSerialPreview);
  document.getElementById('byteSize').addEventListener('change',    updateSerialPreview);
  document.getElementById('modbusTimeout').addEventListener('input', updateSerialPreview);
});

// ============================================================
// SERIAL CONFIG PREVIEW
// ============================================================
function updateSerialPreview() {
  var baud    = document.getElementById('baudrate').value;
  var par     = parseInt(document.getElementById('parity').value);
  var stop    = document.getElementById('stopBits').value;
  var bits    = document.getElementById('byteSize').value;
  var timeout = document.getElementById('modbusTimeout').value;
  var parChar = par === 0 ? 'N' : (par === 1 ? 'E' : 'O');
  var preview = baud + ' ' + bits + parChar + stop + ' \u2014 Timeout: ' + timeout + ' ms';
  var el = document.getElementById('serialPreview');
  if (el) el.textContent = preview;
}

// ============================================================
// THEME
// ============================================================
function toggleTheme() {
  isDark = !isDark;
  document.documentElement.setAttribute('data-theme', isDark ? 'dark' : 'light');
  document.getElementById('themeIco').textContent = isDark ? '\u2600\uFE0F' : '\uD83C\uDF19';
  document.getElementById('themeTxt').textContent = isDark ? 'Light Mode' : 'Dark Mode';
}

// ============================================================
// ACCORDION
// ============================================================
function tog(k) {
  secOpen[k] = !secOpen[k];
  var b = document.getElementById('body-' + k);
  var a = document.getElementById('arr-' + k);
  if (b) {
    if (secOpen[k]) {
      b.classList.remove('hide');
      b.style.display = 'block';
      if (a) a.classList.add('open');
    } else {
      b.classList.add('hide');
      b.style.display = 'none';
      if (a) a.classList.remove('open');
    }
  }
}

// ============================================================
// CLOCK
// ============================================================
function p2(n) {
  return String(n).padStart(2, '0');
}

// Device clock from ESP32. Advanced locally every second so UI never lags on slow /status.
var deviceDateStr = '';
var deviceTimeStr = '';
var deviceTimeSynced = false;
var deviceBase = null;   // {y,mo,d,h,mi,s} last authoritative sample from ESP
var deviceBaseMs = 0;    // Date.now() when deviceBase was taken

function daysInMonthJS(m, y) {
  if (m === 2) return ((y % 4 === 0 && y % 100 !== 0) || (y % 400 === 0)) ? 29 : 28;
  if (m === 4 || m === 6 || m === 9 || m === 11) return 30;
  return 31;
}

function applyDeviceClock(dateStr, timeStr) {
  if (!dateStr || !timeStr) return;
  var dp = dateStr.split('-');
  var tp = timeStr.split(':');
  if (dp.length < 3 || tp.length < 3) return;
  deviceBase = {
    y:  parseInt(dp[2], 10),
    mo: parseInt(dp[1], 10),
    d:  parseInt(dp[0], 10),
    h:  parseInt(tp[0], 10),
    mi: parseInt(tp[1], 10),
    s:  parseInt(tp[2], 10)
  };
  deviceBaseMs = Date.now();
  deviceDateStr = dateStr;
  deviceTimeStr = timeStr;
  deviceTimeSynced = true;
}

function tick() {
  var elD = document.getElementById('dispDate');
  var elT = document.getElementById('dispTime');
  if (deviceTimeSynced && deviceBase) {
    // Advance from last ESP sample — independent of network /status lag
    var elapsed = Math.floor((Date.now() - deviceBaseMs) / 1000);
    if (elapsed < 0) elapsed = 0;
    var y = deviceBase.y, mo = deviceBase.mo, d = deviceBase.d;
    var h = deviceBase.h, mi = deviceBase.mi, s = deviceBase.s + elapsed;
    while (s >= 60) { s -= 60; mi++; }
    while (mi >= 60) { mi -= 60; h++; }
    while (h >= 24) { h -= 24; d++; }
    while (d > daysInMonthJS(mo, y)) { d -= daysInMonthJS(mo, y); mo++; if (mo > 12) { mo = 1; y++; } }
    deviceDateStr = p2(d) + '-' + p2(mo) + '-' + y;
    deviceTimeStr = p2(h) + ':' + p2(mi) + ':' + p2(s);
    if (elD) elD.textContent = deviceDateStr;
    if (elT) elT.textContent = deviceTimeStr;
    return;
  }
  var n = new Date();
  if (elD) elD.textContent = p2(n.getDate()) + '-' + p2(n.getMonth() + 1) + '-' + n.getFullYear();
  if (elT) elT.textContent = p2(n.getHours()) + ':' + p2(n.getMinutes()) + ':' + p2(n.getSeconds());
}

function syncTime() {
  if (isLog) return;
  // Only seed ESP clock if it has NO valid time yet (NTP or last-good).
  // Never overwrite from a second phone — prevents timestamp collisions in logs/graphs.
  if (deviceTimeSynced) return;
  var n = new Date();
  fetch('/setTime?ts=' + Math.floor(n.getTime() / 1000) +
        '&tz=' + n.getTimezoneOffset() +
        '&y='  + n.getFullYear() +
        '&m='  + (n.getMonth() + 1) +
        '&d='  + n.getDate()
  ).then(function(r) { return r.text(); }).then(function(t) {
    if (t && t.indexOf('set once') >= 0) deviceTimeSynced = true;
    if (t && t.indexOf('ignored') >= 0) deviceTimeSynced = true;
  }).catch(function() {});
}

function toast(msg, kind) {
  kind = kind || 'info';
  var t = document.getElementById('toast');
  if (!t) {
    t = document.createElement('div');
    t.id = 'toast';
    t.style.cssText = 'position:fixed;left:50%;bottom:24px;transform:translateX(-50%);z-index:9999;' +
      'padding:12px 18px;border-radius:12px;font-weight:700;font-size:.88rem;color:#fff;' +
      'box-shadow:0 8px 28px rgba(0,0,0,.25);max-width:90%;text-align:center;' +
      'opacity:0;transition:opacity .25s,transform .25s;pointer-events:none';
    document.body.appendChild(t);
  }
  t.style.background = (kind === 'ok') ? '#059669' : (kind === 'err') ? '#dc2626' : '#4f46e5';
  t.textContent = msg;
  t.style.opacity = '1';
  t.style.transform = 'translateX(-50%) translateY(0)';
  clearTimeout(t._tm);
  t._tm = setTimeout(function() {
    t.style.opacity = '0';
    t.style.transform = 'translateX(-50%) translateY(12px)';
  }, 2800);
}
</script>
)CHUNK7"));

  // ---- CHUNK 8: JavaScript — slave grid builder ----
  server.sendContent(F(R"CHUNK8(<script>
// ============================================================
// SLAVE GRID BUILDER
// ============================================================
function buildSlaveGrid(n) {
  numSlaves = n;
  var grid = document.getElementById('slvGrid');
  grid.innerHTML = '';

  for (var i = 0; i < n; i++) {
    var tile = buildSlaveTile(i);
    grid.appendChild(tile);
  }

  // Update the display badge
  var disp = document.getElementById('nsDisplay');
  if (disp) disp.textContent = '\uD83D\uDD0C ' + n + ' Slave' + (n !== 1 ? 's' : '') + ' configured';

  // Rebuild the history slave selector to match
  rebuildHistSlaveSelect(n);
}

// Called live as user types in the number field
function onNumSlavesInput() {
  var el  = document.getElementById('numSlaves');
  var val = parseInt(el.value);
  if (isNaN(val) || val < 1) return;   // wait until valid
  if (val > 16) { el.value = 16; val = 16; }
  buildSlaveGrid(val);
}

// Called on blur / change — snap to valid range
function onNumSlavesChange() {
  var el  = document.getElementById('numSlaves');
  var val = parseInt(el.value);
  if (isNaN(val) || val < 1) { el.value = 1; val = 1; }
  if (val > 16)               { el.value = 16; val = 16; }
  buildSlaveGrid(val);
}

function buildSlaveTile(i) {
  var div = document.createElement('div');
  div.className = 'slv-tile';
  div.id = 'sc' + i;

  var hdr = document.createElement('div');
  hdr.className = 'slv-hdr';
  hdr.innerHTML =
    '<div class="slv-num">' +
      '<div class="slv-badge">' + (i + 1) + '</div>' +
      'Slave ' + (i + 1) +
    '</div>' +
    '<label class="tgl">' +
      '<input type="checkbox" id="en' + i + '" onchange="toggleSlv(' + i + ')">' +
      '<div class="tgl-track"></div>' +
      '<span class="tgl-lbl" id="tl' + i + '">Off</span>' +
    '</label>';

  var fields = document.createElement('div');
  fields.className = 'slv-fields';
  fields.id = 'sf' + i;
  fields.innerHTML =
    '<div class="field">' +
      '<label>Slave ID (1 &ndash; 247)</label>' +
      '<input type="number" id="sid' + i + '" min="1" max="247" value="' + (i + 1) + '">' +
    '</div>' +
    buildSlaveTypeFields(i);

  div.appendChild(hdr);
  div.appendChild(fields);
  return div;
}

function buildSlaveTypeFields(i) {
  if (ltType === 1) {
    // PID mode
    return '<div class="g2">' +
      '<div class="field">' +
        '<label>PV Register Address</label>' +
        '<input type="number" id="pvr' + i + '" min="0" max="65535" value="138">' +
        '<div class="field-hint">Default: 138 (adjust for your PID model)</div>' +
      '</div>' +
      '<div class="field">' +
        '<label>SV Register Address</label>' +
        '<input type="number" id="svr' + i + '" min="0" max="65535" value="0">' +
        '<div class="field-hint">Default: 0 (adjust for your PID model)</div>' +
      '</div>' +
    '</div>';
  } else {
    // TempScanner mode
    return '<div class="g2">' +
      '<div class="field">' +
        '<label>Start Register Address</label>' +
        '<input type="number" id="sreg' + i + '" min="0" max="65535" value="0">' +
        '<div class="field-hint">First register to read from</div>' +
      '</div>' +
      '<div class="field">' +
        '<label>CH to Read (Modbus)</label>' +
        '<select id="mch' + i + '">' +
          '<option value="8" selected>8 CH</option>' +
          '<option value="16">16 CH</option>' +
          '<option value="24">24 CH</option>' +
        '</select>' +
      '</div>' +
    '</div>' +
    '<div class="field">' +
      '<label>CH to Save (1 &ndash; 24)</label>' +
      '<input type="number" id="sch' + i + '" min="1" max="24" value="8">' +
      '<div class="field-hint">Can be less than or equal to CH to Read</div>' +
    '</div>';
  }
}

// Re-build just the type-specific fields inside existing tiles
// (used when logger type changes while slaves are already rendered)
function rebuildSlaveFields() {
  for (var i = 0; i < numSlaves; i++) {
    var sf = document.getElementById('sf' + i);
    if (!sf) continue;

    // Preserve Slave ID and enabled state
    var sidEl      = document.getElementById('sid' + i);
    var sidVal     = sidEl ? sidEl.value : (i + 1);
    var enEl       = document.getElementById('en' + i);
    var wasEnabled = enEl ? enEl.checked : false;

    sf.innerHTML =
      '<div class="field">' +
        '<label>Slave ID (1 &ndash; 247)</label>' +
        '<input type="number" id="sid' + i + '" min="1" max="247" value="' + sidVal + '">' +
      '</div>' +
      buildSlaveTypeFields(i);

    // Restore enabled toggle state
    var enElNew = document.getElementById('en' + i);
    if (enElNew) enElNew.checked = wasEnabled;
    toggleSlv(i);
  }
}

function rebuildHistSlaveSelect(n) {
  var sel = document.getElementById('histSlave');
  if (!sel) return;
  sel.innerHTML = '';
  for (var i = 0; i < n; i++) {
    var opt = document.createElement('option');
    opt.value = i;
    opt.textContent = 'Slave ' + (i + 1);
    sel.appendChild(opt);
  }
}

function onLtChange() {
  ltType = parseInt(document.getElementById('ltSelect').value) || 0;
  rebuildSlaveFields();
  updateLtBadge();
}

function updateLtBadge() {
  var names = ['TempScanner', 'PID'];
  var n = names[ltType] || 'TempScanner';
  var el  = document.getElementById('ltBadgeTxt');
  var el2 = document.getElementById('ltBadgeHdr');
  if (el)  el.textContent  = n;
  if (el2) el2.innerHTML   = '\uD83D\uDD0C ' + n;
}

function toggleSlv(i) {
  var cb  = document.getElementById('en' + i);
  var sf  = document.getElementById('sf' + i);
  var sc  = document.getElementById('sc' + i);
  var tl  = document.getElementById('tl' + i);
  var on  = cb && cb.checked;
  if (sf) sf.classList.toggle('show', on);
  if (sc) sc.classList.toggle('on',   on);
  if (tl) tl.textContent = on ? 'On' : 'Off';
}
</script>
)CHUNK8"));

  // ---- CHUNK 9: JavaScript — config load ----
  server.sendContent(F(R"CHUNK9(<script>
// ============================================================
// CONFIG LOAD FROM DEVICE
// ============================================================
function loadConfig() {
  fetch('/config').then(function(r) {
    return r.json();
  }).then(function(d) {

    // Interval
    var ivEl = document.getElementById('interval');
    if (ivEl && d.interval) ivEl.value = d.interval;

    // Division factor
    var dfEl = document.getElementById('divFactor');
    if (dfEl && d.divFactor !== undefined) dfEl.value = d.divFactor;

    // Unit
    var ufEl = document.getElementById('unitField');
    if (ufEl && d.unit !== undefined) ufEl.value = d.unit;

    // Baud rate
    var bdEl = document.getElementById('baudrate');
    if (bdEl && d.baud) {
      for (var i = 0; i < bdEl.options.length; i++) {
        if (parseInt(bdEl.options[i].value) === d.baud) {
          bdEl.selectedIndex = i;
          break;
        }
      }
    }

    // Parity
    var parEl = document.getElementById('parity');
    if (parEl && d.parity !== undefined) parEl.value = d.parity;

    // Stop bits
    var sbEl = document.getElementById('stopBits');
    if (sbEl && d.stopBits !== undefined) sbEl.value = d.stopBits;

    // Byte size
    var bsEl = document.getElementById('byteSize');
    if (bsEl && d.byteSize !== undefined) bsEl.value = d.byteSize;

    // Modbus timeout
    var toEl = document.getElementById('modbusTimeout');
    if (toEl && d.timeout !== undefined) toEl.value = d.timeout;

    // Update serial preview after loading
    updateSerialPreview();

    // Number of slaves — build grid first then load per-slave values
    var savedSlaves = d.numSlaves || 4;
    if (savedSlaves < 1)   savedSlaves = 1;
    if (savedSlaves > 16)  savedSlaves = 16;
    var nsEl = document.getElementById('numSlaves');
    if (nsEl) nsEl.value = savedSlaves;
    buildSlaveGrid(savedSlaves);

    // Logger type lock
    ltLocked = !!d.typeLocked;
    ltType   = d.loggerType || 0;
    if (ltType > 1) ltType = 0;
    applyLTLock();

    // Per-slave values (loaded after grid is built)
    if (d.slaves) {
      for (var s = 0; s < savedSlaves; s++) {
        var sl = d.slaves[String(s)];
        if (!sl) continue;

        var cbEl = document.getElementById('en' + s);
        if (cbEl) {
          cbEl.checked = !!sl.enabled;
          toggleSlv(s);
        }

        var sidEl2 = document.getElementById('sid' + s);
        if (sidEl2) sidEl2.value = sl.id || (s + 1);

        if (ltType === 1) {
          // PID
          var pvrEl = document.getElementById('pvr' + s);
          if (pvrEl && sl.pvReg !== undefined) pvrEl.value = sl.pvReg;

          var svrEl = document.getElementById('svr' + s);
          if (svrEl && sl.svReg !== undefined) svrEl.value = sl.svReg;
        } else {
          // TempScanner
          var srEl = document.getElementById('sreg' + s);
          if (srEl) srEl.value = sl.startReg || 0;

          var mcEl = document.getElementById('mch' + s);
          if (mcEl) {
            for (var i = 0; i < mcEl.options.length; i++) {
              if (parseInt(mcEl.options[i].value) === (sl.mch || 8)) {
                mcEl.selectedIndex = i;
                break;
              }
            }
          }

          var sc2El = document.getElementById('sch' + s);
          if (sc2El) sc2El.value = sl.sch || 8;
        }
      }
    }

  }).catch(function() {});
}

function applyLTLock() {
  var names = ['TempScanner', 'PID'];
  document.getElementById('ltUnlocked').style.display = ltLocked ? 'none'  : 'block';
  document.getElementById('ltLocked').style.display   = ltLocked ? 'block' : 'none';
  var btEl = document.getElementById('ltBadgeTxt');
  if (btEl) btEl.textContent = names[ltType] || 'TempScanner';
  var bhEl = document.getElementById('ltBadgeHdr');
  if (bhEl) bhEl.innerHTML = '\uD83D\uDD0C ' + (names[ltType] || 'TempScanner');
  if (!ltLocked) {
    var sEl = document.getElementById('ltSelect');
    if (sEl) sEl.selectedIndex = ltType;
  }
  rebuildSlaveFields();
  updateLtBadge();
}
</script>
)CHUNK9"));

  // ---- CHUNK 10: JavaScript — poll, UI state, start/stop ----
  server.sendContent(F(R"CHUNK10(<script>
// ============================================================
// STATUS & ERROR POLL
// ============================================================
var _lastSta = null;

function saveWifi() {
  var ssid = (document.getElementById('newSsid').value || '').trim();
  var pass = document.getElementById('newPass').value || '';
  if (!ssid) { toast('Enter WiFi name', 'err'); return; }
  var btn = document.getElementById('saveWifiBtn');
  var msg = document.getElementById('wifiMsg');
  btn.disabled = true;
  btn.textContent = 'Connecting…';
  if (msg) { msg.style.display = 'block'; msg.textContent = 'Saving and connecting (up to 20s)…'; }
  var useSt = document.getElementById('useStatic');
  var staticOn = useSt ? useSt.checked : true;
  var body = 'ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass) +
    '&static=' + (staticOn ? '1' : '0') +
    '&ip=' + encodeURIComponent((document.getElementById('staticIP') || {}).value || '192.168.1.200') +
    '&gw=' + encodeURIComponent((document.getElementById('gatewayIP') || {}).value || '192.168.1.1') +
    '&sn=' + encodeURIComponent((document.getElementById('subnetMask') || {}).value || '255.255.255.0');
  fetch('/setWifi', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: body })
    .then(function(r) { return r.text().then(function(t) { return { ok: r.ok, text: t }; }); })
    .then(function(res) {
      btn.disabled = false;
      btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor" width="18" height="18"><path d="M12 3C7.46 3 3.34 4.67.29 7.29l1.42 1.42C4.33 6.23 7.89 5 12 5s7.67 1.23 10.29 3.71l1.42-1.42C20.66 4.67 16.54 3 12 3zm0 4c-3.12 0-5.95 1.17-8.12 3.09l1.42 1.42C7.17 9.76 9.44 9 12 9s4.83.76 6.7 2.51l1.42-1.42C17.95 8.17 15.12 7 12 7zm0 4c-1.85 0-3.55.73-4.8 1.9l1.41 1.41C9.47 13.45 10.68 13 12 13s2.53.45 3.39 1.31l1.41-1.41C15.55 11.73 13.85 11 12 11zm0 4c-.83 0-1.5.67-1.5 1.5S11.17 18 12 18s1.5-.67 1.5-1.5S12.83 15 12 15z"/></svg> Save WiFi &amp; Connect';
      if (msg) {
        msg.style.display = 'block';
        msg.textContent = res.text;
      }
      toast(res.text, res.ok ? 'ok' : 'err');
      setTimeout(pollSt, 500);
    })
    .catch(function() {
      btn.disabled = false;
      btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor" width="18" height="18"><path d="M12 3C7.46 3 3.34 4.67.29 7.29l1.42 1.42C4.33 6.23 7.89 5 12 5s7.67 1.23 10.29 3.71l1.42-1.42C20.66 4.67 16.54 3 12 3zm0 4c-3.12 0-5.95 1.17-8.12 3.09l1.42 1.42C7.17 9.76 9.44 9 12 9s4.83.76 6.7 2.51l1.42-1.42C17.95 8.17 15.12 7 12 7zm0 4c-1.85 0-3.55.73-4.8 1.9l1.41 1.41C9.47 13.45 10.68 13 12 13s2.53.45 3.39 1.31l1.41-1.41C15.55 11.73 13.85 11 12 11zm0 4c-.83 0-1.5.67-1.5 1.5S11.17 18 12 18s1.5-.67 1.5-1.5S12.83 15 12 15z"/></svg> Save WiFi &amp; Connect';
      toast('Request failed', 'err');
    });
}

function checkOta() {
  if (!confirm('Check for firmware update now?\n\n• Device must be on home WiFi with internet\n• Logging must be stopped\n• Device will reboot if an update is installed')) return;
  var btn = document.getElementById('otaBtn');
  var msg = document.getElementById('otaMsg');
  if (btn) { btn.disabled = true; btn.textContent = 'Checking…'; }
  if (msg) msg.textContent = 'Checking for update…';
  fetch('/ota-check', { method: 'POST' })
    .then(function(r) { return r.text(); })
    .then(function(t) {
      if (msg) msg.textContent = t;
      var bad = /fail|Stop|Connect|Cannot|Bad|Invalid|missing/i.test(t);
      toast(t, bad ? 'err' : 'ok');
      if (btn) { btn.disabled = false; btn.textContent = 'Check for update'; }
      if (/reboot|Downloading|update/i.test(t) && !bad) {
        toast('Updating – wait for reboot (1–2 min)', 'ok');
      }
    })
    .catch(function() {
      if (msg) msg.textContent = 'Request failed';
      if (btn) { btn.disabled = false; btn.textContent = 'Check for update'; }
      toast('OTA request failed', 'err');
    });
}

function pollSt() {
  fetch('/status').then(function(r) {
    return r.json();
  }).then(function(d) {
    var changed = (isLog !== d.logging);
    isLog = d.logging;
    if (changed) applyUI(isLog);
    var cur = document.getElementById('curStaSsid');
    if (cur && d.staSsid) cur.textContent = d.staSsid + (d.staCredsUser ? ' (saved)' : ' (default)');
    var us = document.getElementById('useStatic');
    if (us && d.useStatic !== undefined) us.checked = !!d.useStatic;
    if (d.staticIP && document.getElementById('staticIP')) document.getElementById('staticIP').value = d.staticIP;
    if (d.gateway && document.getElementById('gatewayIP')) document.getElementById('gatewayIP').value = d.gateway;
    if (d.subnet && document.getElementById('subnetMask')) document.getElementById('subnetMask').value = d.subnet;
    var sf = document.getElementById('staticFields');
    if (sf && us) sf.style.opacity = us.checked ? '1' : '0.45';

    window._activeLogFiles = {};
    if (d.activeFiles && d.activeFiles.length) {
      d.activeFiles.forEach(function(n) { window._activeLogFiles[n] = true; });
    }

    // Prefer ESP32 device clock for the purple DATE/TIME banner (local tick advances it)
    if (d.deviceDate && d.deviceTime) {
      applyDeviceClock(d.deviceDate, d.deviceTime);
      tick();
    }
    var fl = document.getElementById('fwLabel');
    if (fl && d.fwVersion) fl.textContent = 'v' + d.fwVersion + ' (build ' + d.fwBuild + ')';
    var hs = document.getElementById('fwHdrSub');
    if (hs && d.fwVersion) hs.textContent = 'v' + d.fwVersion + ' (build ' + d.fwBuild + ') · Tap to expand';
    var om = document.getElementById('otaMsg');
    if (om && d.otaMessage && d.otaMessage !== '—') om.textContent = d.otaMessage;

    // Banner only on SoftAP setup page (192.168.1.1)
    var nb = document.getElementById('netBanner');
    var host = location.hostname;
    var onSoftAP = (host === '192.168.4.1' || host.indexOf('192.168.4.') === 0);
    // WiFi setup form only on SoftAP (QR page), never on Station IP
    var wc = document.getElementById('wifiCard');
    if (wc) wc.style.display = onSoftAP ? 'block' : 'none';
    // Firmware / OTA only on Station (home WiFi) — hidden on SoftAP recovery page
    var fw = document.getElementById('fwCard');
    if (fw) fw.style.display = onSoftAP ? 'none' : '';
    if (nb) {
      if (onSoftAP && (d.staIP || d.lockedIP || d.staticIP)) {
        var locked = d.lockedIP || (d.staticLearned ? d.staticIP : '') || d.staticIP || '';
        var current = d.staIP || locked;
        var best = locked || current;
        var url = 'http://' + best;
        var homeSsid = d.staSsid || 'your home/office WiFi';
        window._dailyUrl = url;
        window._homeSsid = homeSsid;
        nb.style.display = 'block';
        var txt = document.getElementById('staIpTxt');
        if (txt) {
          var html = 'Daily link: <b>' + url + '</b>';
          html += '<br>Home WiFi name: <b>' + homeSsid + '</b>';
          if (current && locked && current !== locked) html += '<br>Current address now: <b>http://' + current + '</b>';
          if (d.deviceTime) html += '<br>Device time: <b>' + d.deviceDate + ' ' + d.deviceTime + '</b>';
          txt.innerHTML = html;
        }
        function copyDailyLink(btn) {
          var u = window._dailyUrl || url;
          function ok() {
            if (btn) { var t = btn.textContent; btn.textContent = 'Copied!'; setTimeout(function(){ btn.textContent = t; }, 1600); }
            toast('Copied. Switch to ' + (window._homeSsid || 'home WiFi') + ' then open the link', 'ok');
          }
          if (navigator.clipboard && navigator.clipboard.writeText) {
            navigator.clipboard.writeText(u).then(ok).catch(function(){ prompt('Copy this link:', u); });
          } else {
            prompt('Copy this link:', u);
            ok();
          }
        }
        function openWifiGuide() {
          var modal = document.getElementById('wifiGuideModal');
          if (!modal) return;
          var gs = document.getElementById('guideSsid');
          var gu = document.getElementById('guideUrl');
          if (gs) gs.textContent = window._homeSsid || homeSsid;
          if (gu) gu.textContent = window._dailyUrl || url;
          modal.style.display = 'flex';
        }
        function closeWifiGuide() {
          var modal = document.getElementById('wifiGuideModal');
          if (modal) modal.style.display = 'none';
        }
        var guideBtn = document.getElementById('staGuideBtn');
        var copyBtn = document.getElementById('staCopyBtn');
        if (guideBtn) guideBtn.onclick = openWifiGuide;
        if (copyBtn) copyBtn.onclick = function(){ copyDailyLink(copyBtn); };
        var gCopy = document.getElementById('guideCopyBtn');
        var gOpen = document.getElementById('guideOpenBtn');
        var gClose = document.getElementById('guideCloseBtn');
        if (gCopy) gCopy.onclick = function(){ copyDailyLink(gCopy); };
        if (gOpen) gOpen.onclick = function() {
          closeWifiGuide();
          toast('Opening daily link — stay on ' + (window._homeSsid || 'home WiFi'), 'ok');
          setTimeout(function(){ window.location.href = window._dailyUrl || url; }, 400);
        };
        if (gClose) gClose.onclick = closeWifiGuide;
        var modal = document.getElementById('wifiGuideModal');
        if (modal) modal.onclick = function(e){ if (e.target === modal) closeWifiGuide(); };
        if (_lastSta === false) toast('Station ready — tap How to open daily link', 'ok');
        _lastSta = true;
      } else {
        nb.style.display = 'none';
        if (onSoftAP && _lastSta === true && !d.staConnected) {
          toast('WiFi station disconnected — SoftAP still available', 'err');
        }
        _lastSta = d.staConnected ? true : false;
      }
    }
  }).catch(function() {});
}

function pollErr() {
  fetch('/error-status').then(function(r) {
    return r.json();
  }).then(function(d) {
    var box  = document.getElementById('errBox');
    var lc   = document.getElementById('ledComm');
    var lcl  = document.getElementById('ledCommLbl');
    var ls   = document.getElementById('ledSD');
    var lsl  = document.getElementById('ledSDlbl');
    var hasErr = d.error && d.error.trim() !== '';

    if (hasErr) {
      var isRebooting = d.error.toLowerCase().indexOf('rebooting') >= 0;
      if (isRebooting) {
        document.getElementById('stCard').className = 'st st-idle';
        document.getElementById('stIco').textContent = '\u26A0';
        document.getElementById('stTt').textContent  = 'Device Rebooting...';
        document.getElementById('stTs').textContent  = d.error;
      }
      document.getElementById('errTxt').textContent = d.error;
      box.style.display = 'flex';

      var m    = d.error.toLowerCase();
      var isSD = (m.indexOf('sd') >= 0 || m.indexOf('card') >= 0 || m.indexOf('mount') >= 0 ||
                  m.indexOf('flush') >= 0 || m.indexOf('write') >= 0 || m.indexOf('removed') >= 0) &&
                 m.indexOf('comm') < 0 && m.indexOf('modbus') < 0;
      var isC  = m.indexOf('modbus') >= 0 || m.indexOf('comm') >= 0 || m.indexOf('slave') >= 0 ||
                 m.indexOf('pre-start') >= 0 || (m.indexOf('fail') >= 0 && !isSD);

      lc.className = 'led ' + (isC  ? 'led-comm-on' : 'led-comm-off');
      lcl.textContent = isC  ? 'ERROR' : 'OK';
      ls.className = 'led ' + (isSD ? 'led-sd-on'   : 'led-sd-off');
      lsl.textContent = isSD ? 'ERROR' : 'OK';

      var sb = document.getElementById('startBtn');
      if (sb && isSD) { sb.disabled = true; sb.title = 'SD card missing'; }

    } else {
      box.style.display = 'none';
      lc.className = 'led led-comm-off'; lcl.textContent = 'OK';
      ls.className = 'led led-sd-off';   lsl.textContent = 'OK';
      var sb = document.getElementById('startBtn');
      if (sb && !isLog) { sb.disabled = false; sb.title = ''; }
    }
  }).catch(function() {});
}

// ============================================================
// UI STATE APPLY
// ============================================================
function applyUI(logging) {
  var card   = document.getElementById('stCard');
  var ico    = document.getElementById('stIco');
  var tt     = document.getElementById('stTt');
  var ts     = document.getElementById('stTs');
  var sb     = document.getElementById('startBtn');
  var xb     = document.getElementById('stopBtn');
  var li     = document.getElementById('logInfo');

  // All inputs that should be locked while logging
  var inputs = document.querySelectorAll(
    '#body-timing input, #body-timing select,' +
    '#body-device input, #body-device select,' +
    '#body-slaves input, #body-slaves select,' +
    '#ltUnlocked select, #numSlaves'
  );

  if (logging) {
    card.className     = 'st st-live';
    ico.className      = 'st-ico pa';
    ico.textContent    = '\uD83D\uDFE2';
    tt.textContent     = 'Logging Active';
    ts.textContent     = 'Writing data to SD card \u2014 Do not remove SD card';
    sb.style.display   = 'none';
    xb.style.display   = 'flex';
    li.style.display   = 'block';
    inputs.forEach(function(el) { el.disabled = true; });
    if (!secOpen['ctrl']) {
      secOpen['ctrl'] = true;
      var bCtrl = document.getElementById('body-ctrl');
      var aCtrl = document.getElementById('arr-ctrl');
      if (bCtrl) bCtrl.classList.remove('hide');
      if (aCtrl) aCtrl.classList.add('open');
    }
    startMonitor(parseInt(document.getElementById('interval').value) || 5);

  } else {
    card.className     = 'st st-idle';
    ico.className      = 'st-ico';
    ico.textContent    = '\u26AA';
    tt.textContent     = 'System Ready';
    ts.textContent     = 'Configure settings below, then tap Start Logging';
    sb.style.display   = 'flex';
    xb.style.display   = 'none';
    li.style.display   = 'none';
    inputs.forEach(function(el) { el.disabled = false; });
    stopMonitor();
  }
}

// ============================================================
// START LOGGING
// ============================================================
function startLog() {
  var any = false;
  for (var i = 0; i < numSlaves; i++) {
    var el = document.getElementById('en' + i);
    if (el && el.checked) { any = true; break; }
  }
  if (!any) {
    alert('Enable at least one slave device before starting.');
    return;
  }

  var fd = new FormData();
  fd.append('int', document.getElementById('interval').value);
  fd.append('baud', document.getElementById('baudrate').value);

  var dfEl = document.getElementById('divFactor');
  fd.append('div', dfEl ? (parseFloat(dfEl.value) || 10) : 10);

  var ufEl = document.getElementById('unitField');
  fd.append('unit', ufEl ? ufEl.value.trim() : '');

  // New serial settings
  fd.append('parity',   document.getElementById('parity').value);
  fd.append('stopBits', document.getElementById('stopBits').value);
  fd.append('byteSize', document.getElementById('byteSize').value);
  fd.append('timeout',  document.getElementById('modbusTimeout').value);

  // Number of slaves
  fd.append('ns', numSlaves);

  // Logger type (only if not locked)
  if (!ltLocked) {
    var ltEl = document.getElementById('ltSelect');
    fd.append('lt', ltEl ? ltEl.value : '0');
  }

  // Per-slave data
  for (var i = 0; i < numSlaves; i++) {
    var enEl = document.getElementById('en' + i);
    var enV  = enEl && enEl.checked;
    fd.append('en' + i, enV ? '1' : '0');

    if (enV) {
      var sidEl = document.getElementById('sid' + i);
      fd.append('sid' + i, sidEl ? sidEl.value : (i + 1));

      if (ltType === 1) {
        // PID
        var pvrEl = document.getElementById('pvr' + i);
        fd.append('pvr' + i, pvrEl ? pvrEl.value : 138);

        var svrEl = document.getElementById('svr' + i);
        fd.append('svr' + i, svrEl ? svrEl.value : 0);
      } else {
        // TempScanner
        var srEl  = document.getElementById('sreg' + i);
        fd.append('sreg' + i, srEl ? srEl.value : 0);

        var mcEl  = document.getElementById('mch' + i);
        fd.append('mch' + i, mcEl ? mcEl.value : 8);

        var sc2El = document.getElementById('sch' + i);
        fd.append('sch' + i, sc2El ? sc2El.value : 8);
      }
    }
  }

  var btn = document.getElementById('startBtn');
  btn.disabled = true;
  btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg> Starting\u2026';

  fetch('/start', { method: 'POST', body: fd })
    .then(function(r) {
      return r.text().then(function(t) {
        return { ok: r.ok, status: r.status, text: t };
      });
    })
    .then(function(res) {
      btn.disabled = false;
      btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg> Start Logging';
      if (res.status === 503) {
        document.getElementById('errTxt').textContent = res.text;
        document.getElementById('errBox').style.display = 'flex';
        document.getElementById('ledComm').className = 'led led-comm-on';
        document.getElementById('ledCommLbl').textContent = 'ERROR';
        pollErr();
        return;
      }
      if (!res.ok) {
        toast('Failed to start: ' + res.text, 'err');
        return;
      }
      toast('Logging started', 'ok');
      loadConfig();
      pollSt();
      loadDates();
    })
    .catch(function() {
      toast('Failed to start. Check WiFi connection.', 'err');
      btn.disabled = false;
      btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg> Start Logging';
    });
}

// ============================================================
// STOP LOGGING
// ============================================================
function stopLog() {
  if (!confirm('Stop logging?\n\nAll buffered data will be saved to SD card.')) return;
  var btn = document.getElementById('stopBtn');
  btn.disabled = true;
  btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg> Stopping\u2026';

  fetch('/stop', { method: 'POST' })
    .then(function(r) { return r.text(); })
    .then(function() {
      btn.disabled = false;
      btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg> Stop Logging';
      toast('Logging stopped — data saved', 'ok');
      pollSt();
      loadDates();
    })
    .catch(function() {
      toast('Failed to stop.', 'err');
      btn.disabled = false;
      btn.innerHTML = '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg> Stop Logging';
    });
}
</script>
)CHUNK10"));

  // ---- CHUNK 11: JavaScript — calendar + file list ----
  server.sendContent(F(R"CHUNK11(<script>
// ============================================================
// CALENDAR
// ============================================================
function loadDates() {
  fetch('/available-dates').then(function(r) {
    return r.json();
  }).then(function(d) {
    avDts = new Set(d);
    buildCal();
  }).catch(function() {});
}

var MN = ['January','February','March','April','May','June',
          'July','August','September','October','November','December'];

function buildCal() {
  var g  = document.getElementById('calGrid');
  g.innerHTML = '';
  var td = new Date();
  for (var off = 0; off >= -11; off--) {
    g.appendChild(buildMo(new Date(td.getFullYear(), td.getMonth() + off, 1), td));
  }
}

function buildMo(date, today) {
  var y   = date.getFullYear();
  var m   = date.getMonth();
  var dim = new Date(y, m + 1, 0).getDate();
  var cnt = 0;

  for (var d = 1; d <= dim; d++) {
    var ds = p2(d) + '-' + p2(m + 1) + '-' + String(y);
    if (avDts.has(ds)) cnt++;
  }

  var card = document.createElement('div');
  card.className = 'mo-card';
  card.innerHTML =
    '<div class="mo-hdr">' +
      '<span class="mo-name">' + MN[m] + ' ' + y + '</span>' +
      (cnt ? '<span class="mo-cnt">' + cnt + ' day' + (cnt > 1 ? 's' : '') + ' with data</span>' : '') +
    '</div>';

  var tbl = document.createElement('table');
  tbl.className = 'mo-tbl';
  var tr = document.createElement('tr');
  ['Su','Mo','Tu','We','Th','Fr','Sa'].forEach(function(n) {
    var th = document.createElement('th');
    th.textContent = n;
    tr.appendChild(th);
  });
  tbl.appendChild(tr);

  var fd  = new Date(y, m, 1).getDay();
  var row = document.createElement('tr');

  for (var i = 0; i < fd; i++) {
    var td2 = document.createElement('td');
    var b   = document.createElement('button');
    b.className  = 'day';
    b.textContent = '';
    td2.appendChild(b);
    row.appendChild(td2);
  }

  for (var d = 1; d <= dim; d++) {
    var td2   = document.createElement('td');
    var b     = document.createElement('button');
    var ds    = p2(d) + '-' + p2(m + 1) + '-' + String(y);
    var isTdy = (d === today.getDate() && m === today.getMonth() && y === today.getFullYear());

    b.textContent = d;
    b.className   = 'day' +
      (avDts.has(ds) ? ' hd' : '') +
      (selDts.has(ds) ? ' sel' : '') +
      (isTdy ? ' tdy' : '');

    if (avDts.has(ds)) {
      (function(s) {
        b.onclick = function() { toggleDt(s); };
      })(ds);
    }

    td2.appendChild(b);
    row.appendChild(td2);

    if ((fd + d) % 7 === 0) {
      tbl.appendChild(row);
      row = document.createElement('tr');
    }
  }

  if (row.children.length > 0) tbl.appendChild(row);
  card.appendChild(tbl);
  return card;
}

function toggleDt(ds) {
  if (selDts.has(ds)) {
    selDts.delete(ds);
  } else {
    selDts.add(ds);
  }
  buildCal();
  refreshFiles();
  if (!secOpen['dl']) tog('dl');
}

// ============================================================
// FILE LIST
// ============================================================
function refreshFiles() {
  var fp = document.getElementById('fp');
  var fl = document.getElementById('fList');
  fmap = {};
  fl.innerHTML = '';
  if (!selDts.size) {
    fp.style.display = 'none';
    return;
  }
  fp.style.display = 'block';
  fl.innerHTML = '<div class="f-empty">Loading\u2026</div>';

  var arr  = Array.from(selDts).sort();
  var pend = arr.length;
  var all  = [];

  arr.forEach(function(dt) {
    fetch('/files-for-date?date=' + encodeURIComponent(dt))
      .then(function(r) { return r.json(); })
      .then(function(files) {
        files.forEach(function(f) { all.push({ name: f, date: dt }); });
        pend--;
        if (!pend) renderFiles(all);
      })
      .catch(function() {
        pend--;
        if (!pend) renderFiles(all);
      });
  });
}

function renderFiles(files) {
  var fl = document.getElementById('fList');
  fmap = {};
  fl.innerHTML = '';
  document.getElementById('fCnt').textContent = files.length;

  if (!files.length) {
    fl.innerHTML = '<div class="f-empty">No CSV files found for selected dates.</div>';
    return;
  }

  var activeSet = window._activeLogFiles || {};
  files.forEach(function(f) {
    var isActive = !!activeSet[f.name];
    var row = document.createElement('div');
    row.className = 'f-row' + (isActive ? '' : ' fck');
    if (isActive) row.style.opacity = '0.55';

    var cb    = document.createElement('input');
    cb.type   = 'checkbox';
    cb.checked = !isActive;
    cb.disabled = isActive;
    cb.value  = f.name;
    if (isActive) cb.title = 'Active log file — stop logging to download';

    var badge = document.createElement('span');
    badge.className   = 'f-badge';
    badge.textContent = 'CSV';

    var nm = document.createElement('span');
    nm.className   = 'f-name';
    nm.textContent = f.name;

    var dt = document.createElement('span');
    dt.className   = 'f-date';
    dt.textContent = f.date;

    cb.onchange = function() {
      row.classList.toggle('fck', cb.checked);
      updAll();
    };
    row.onclick = function(e) {
      if (e.target !== cb) {
        cb.checked = !cb.checked;
        cb.onchange();
      }
    };

    row.appendChild(cb);
    row.appendChild(badge);
    row.appendChild(nm);
    row.appendChild(dt);
    fl.appendChild(row);
    fmap[f.name] = cb;
  });

  updAll();
}

function toggleAll() {
  var m = document.getElementById('selAll');
  for (var fn in fmap) {
    fmap[fn].checked = m.checked;
    var r = fmap[fn].parentElement;
    if (r) r.classList.toggle('fck', m.checked);
  }
}

function updAll() {
  var t = 0, c = 0;
  for (var fn in fmap) {
    t++;
    if (fmap[fn].checked) c++;
  }
  var cb = document.getElementById('selAll');
  if (cb) {
    cb.checked       = (t > 0 && c === t);
    cb.indeterminate = (c > 0 && c < t);
  }
  document.getElementById('fCnt').textContent = c + '/' + t;
}

function dlAll() {
  var dl = [];
  for (var fn in fmap) {
    if (fmap[fn].checked) dl.push(fn);
  }
  if (!dl.length) { alert('No files selected.'); return; }

  // Blob download: one request per file, forced attachment — reduces extra "Keep" steps
  var idx = 0;
  function nxt() {
    if (idx >= dl.length) {
      toast('Download complete (' + dl.length + ' file(s))', 'ok');
      return;
    }
    var fn = dl[idx++];
    var name = fn.replace(/^\//, '');
    fetch('/download?file=' + encodeURIComponent(fn))
      .then(function(r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.blob();
      })
      .then(function(blob) {
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = name;
        a.style.display = 'none';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        setTimeout(function(){ URL.revokeObjectURL(url); }, 4000);
        setTimeout(nxt, 800);
      })
      .catch(function(e) {
        alert('Download failed for ' + name + ': ' + (e.message || e));
        setTimeout(nxt, 500);
      });
  }
  nxt();
}

// ============================================================
// ZIP CONNECTIVITY CHECK
// Tries to reach JSZip CDN — if successful, enables ZIP button.
// This works when the user has mobile data active alongside the
// ESP32 WiFi AP connection (common on Android/iOS with
// "WiFi + Mobile Data" coexistence enabled in phone settings).
// ============================================================
// ZIP uses built-in browser APIs — no external library needed

function checkZipAvailability() {
  var btn = document.getElementById('zipBtn');
  var dot = document.getElementById('zipStatusDot');
  if (!btn || !dot) return;

  // Count selected files
  var count = 0;
  for (var fn in fmap) { if (fmap[fn].checked) count++; }

  if (count < 2) {
    btn.disabled = true;
    btn.title    = 'Select 2 or more files to enable ZIP';
    dot.style.background = '#f59e0b';
    return;
  }

  // JSZip is served locally from the ESP32 — always available, no internet needed
  btn.disabled         = false;
  btn.title            = 'Download all selected files as a single ZIP archive';
  dot.style.background = '#22c55e';
}

// ============================================================
// ZIP BUILDER — pure browser, zero CDN, works fully offline
// Uses manual ZIP format construction (no library needed)
// Compatible with all modern mobile browsers
// ============================================================

function dlZip() {
  var dl = [];
  for (var fn in fmap) {
    if (fmap[fn].checked) dl.push(fn);
  }
  if (dl.length < 2) { alert('Select at least 2 files to ZIP.'); return; }

  var btn = document.getElementById('zipBtn');
  btn.disabled    = true;
  btn.textContent = 'Fetching files\u2026';

  // Sequential fetch is more reliable on weak SoftAP / Station links than parallel
  var files = [];
  var i = 0;
  function nextFile() {
    if (i >= dl.length) {
      btn.textContent = 'Building ZIP\u2026';
      try {
        var zipBytes = buildZip(files);
        var blob = new Blob([zipBytes], { type: 'application/zip' });
        var url  = URL.createObjectURL(blob);
        var a    = document.createElement('a');
        a.href   = url;
        a.download = 'Sigma_Controls_Zip_' + new Date().toISOString().slice(0,10) + '.zip';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        setTimeout(function() { URL.revokeObjectURL(url); }, 3000);
        toast('ZIP ready', 'ok');
      } catch (e) {
        alert('ZIP failed: ' + (e.message || e));
      }
      btn.disabled = false;
      btn.innerHTML =
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">' +
        '<line x1="12" y1="3" x2="12" y2="15"/><polyline points="7 10 12 15 17 10"/>' +
        '<line x1="4" y1="20" x2="20" y2="20"/></svg> Download as ZIP ' +
        '<span id="zipStatusDot" style="display:inline-block;width:8px;height:8px;border-radius:50%;' +
        'background:#22c55e;margin-left:6px;flex-shrink:0;transition:background .3s;"></span>';
      return;
    }
    var fn = dl[i++];
    btn.textContent = 'Fetching ' + i + '/' + dl.length + '\u2026';
    var ctrl = (typeof AbortController !== 'undefined') ? new AbortController() : null;
    var timer = ctrl ? setTimeout(function(){ try { ctrl.abort(); } catch(e){} }, 60000) : null;
    fetch('/download?file=' + encodeURIComponent(fn), ctrl ? { signal: ctrl.signal } : undefined)
      .then(function(r) {
        if (!r.ok) throw new Error('Failed: ' + fn + ' (HTTP ' + r.status + ')');
        return r.arrayBuffer();
      })
      .then(function(buf) {
        if (timer) clearTimeout(timer);
        files.push({ name: fn, data: new Uint8Array(buf) });
        nextFile();
      })
      .catch(function(e) {
        if (timer) clearTimeout(timer);
        alert('Download failed on weak network: ' + (e.message || e) +
              '\n\nTip: stay connected to Sigma_Controls (192.168.4.1) or home WiFi, then retry.');
        btn.disabled = false;
        btn.textContent = 'Download as ZIP';
      });
  }
  nextFile();
}

// ============================================================
// MANUAL ZIP BUILDER — no library, pure JS
// Implements ZIP store (no compression) — fast, simple, reliable
// ============================================================
function buildZip(files) {
  var localHeaders = [];
  var centralDir   = [];
  var offset       = 0;

  function strToBytes(s) {
    var b = new Uint8Array(s.length);
    for (var i = 0; i < s.length; i++) b[i] = s.charCodeAt(i) & 0xff;
    return b;
  }

  function u16(n) {
    return [n & 0xff, (n >> 8) & 0xff];
  }

  function u32(n) {
    return [n & 0xff, (n >> 8) & 0xff, (n >> 16) & 0xff, (n >> 24) & 0xff];
  }

  function crc32(data) {
    var table = crc32.table;
    if (!table) {
      table = crc32.table = new Uint32Array(256);
      for (var i = 0; i < 256; i++) {
        var c = i;
        for (var j = 0; j < 8; j++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
        table[i] = c;
      }
    }
    var crc = 0xFFFFFFFF;
    for (var i = 0; i < data.length; i++) crc = (crc >>> 8) ^ table[(crc ^ data[i]) & 0xff];
    return (crc ^ 0xFFFFFFFF) >>> 0;
  }

  var now = new Date();
  var dosTime = ((now.getSeconds() >> 1)) |
                ((now.getMinutes()) << 5) |
                ((now.getHours()) << 11);
  var dosDate = ((now.getDate())) |
                ((now.getMonth() + 1) << 5) |
                (((now.getFullYear() - 1980)) << 9);

  files.forEach(function(file) {
    var nameBytes = strToBytes(file.name);
    var crc       = crc32(file.data);
    var size      = file.data.length;

    // Local file header
    var lh = [].concat(
      [0x50,0x4B,0x03,0x04], // signature
      u16(20),               // version needed
      u16(0),                // flags
      u16(0),                // compression (STORE)
      u16(dosTime),
      u16(dosDate),
      u32(crc),
      u32(size),             // compressed size
      u32(size),             // uncompressed size
      u16(nameBytes.length),
      u16(0)                 // extra field length
    );

    localHeaders.push({ lh: lh, nameBytes: nameBytes, data: file.data, offset: offset });

    // Central directory entry
    var cd = [].concat(
      [0x50,0x4B,0x01,0x02], // signature
      u16(20),               // version made by
      u16(20),               // version needed
      u16(0),                // flags
      u16(0),                // compression (STORE)
      u16(dosTime),
      u16(dosDate),
      u32(crc),
      u32(size),
      u32(size),
      u16(nameBytes.length),
      u16(0),                // extra
      u16(0),                // comment
      u16(0),                // disk start
      u16(0),                // internal attr
      u32(0),                // external attr
      u32(offset)            // local header offset
    );

    centralDir.push({ cd: cd, nameBytes: nameBytes });
    offset += lh.length + nameBytes.length + size;
  });

  // Calculate total size
  var cdSize   = 0;
  var cdOffset = offset;
  centralDir.forEach(function(e) { cdSize += e.cd.length + e.nameBytes.length; });

  var eocd = [].concat(
    [0x50,0x4B,0x05,0x06], // end of central dir signature
    u16(0), u16(0),         // disk numbers
    u16(files.length),
    u16(files.length),
    u32(cdSize),
    u32(cdOffset),
    u16(0)                  // comment length
  );

  var totalSize = offset + cdSize + eocd.length;
  var out       = new Uint8Array(totalSize);
  var pos       = 0;

  function write(arr) {
    if (arr instanceof Uint8Array) {
      out.set(arr, pos); pos += arr.length;
    } else {
      for (var i = 0; i < arr.length; i++) out[pos++] = arr[i];
    }
  }

  localHeaders.forEach(function(e) {
    write(e.lh);
    write(e.nameBytes);
    write(e.data);
  });

  centralDir.forEach(function(e) {
    write(e.cd);
    write(e.nameBytes);
  });

  write(eocd);
  return out;
}

// Hook into updAll to update ZIP button state
var _origUpdAll = updAll;
updAll = function() {
  _origUpdAll();
  checkZipAvailability();
};
</script>
)CHUNK11"));

  // ---- CHUNK 12: JavaScript — live monitor + charts ----
  server.sendContent(F(R"CHUNK12(<script>
// ============================================================
// CHANNEL NAMES
// ============================================================
function chName(loggerType, chIdx) {
  if (loggerType === 1) {
    return chIdx === 0 ? 'PV' : 'SV';
  }
  return 'CH' + (chIdx + 1);
}

// ============================================================
// COLORS
// ============================================================
var CH_COLORS = [
  '#e63946','#2196f3','#ff9800','#4caf50','#9c27b0','#00bcd4',
  '#f44336','#03a9f4','#ff5722','#8bc34a','#673ab7','#00e5ff',
  '#ffd600','#e91e63','#1de9b6','#ff6d00','#3d5afe','#76ff03',
  '#d500f9','#00e676','#ff1744','#2979ff','#ffea00','#f50057'
];

// ============================================================
// LIVE MONITOR
// ============================================================
function startMonitor(intervalSec) {
  document.getElementById('monitorSection').style.display = 'block';
  if (monInterval) clearInterval(monInterval);
  var ms = Math.max(intervalSec * 1000, 3000);

  // Tables always load; charts wait for Chart.js (needs internet / Station WiFi)
  var chartTries = 0;
  function tryLoad() {
    loadLive10();
    loadHistory();
    if (typeof Chart === 'undefined' && chartTries < 15) {
      chartTries++;
      setTimeout(tryLoad, 600);
      return;
    }
    if (typeof Chart === 'undefined') {
      console.warn('Chart.js unavailable (offline SoftAP) — tables only');
    }
  }
  tryLoad();
  monInterval = setInterval(function() {
    loadLive10();
    if (histPage >= histTotalPages - 1) loadHistory();
  }, ms);
}

function stopMonitor() {
  document.getElementById('monitorSection').style.display = 'none';
  if (monInterval) {
    clearInterval(monInterval);
    monInterval = null;
  }
  for (var k in charts10)    { try { charts10[k].destroy();    } catch(e) {} }
  for (var k in chartsHist)  { try { chartsHist[k].destroy();  } catch(e) {} }
  charts10   = {};
  chartsHist = {};
  var w = document.getElementById('mon10Content');
  if (w) w.innerHTML = '<div class="f-empty">Waiting for data\u2026</div>';
}

// ============================================================
// LOAD LIVE 10
// ============================================================
function loadLive10() {
  fetch('/live-data').then(function(r) {
    return r.json();
  }).then(function(d) {
    if (!d.logging) return;
    if (d.unit !== undefined) currentUnit = d.unit;
    var lt = d.loggerType || 0;
    if (d.slaves) {
      d.slaves.forEach(function(sl) { sl.loggerType = lt; });
    }
    renderLive10(d);
  }).catch(function() {});
}

// ============================================================
// RENDER LIVE 10
// ============================================================
function renderLive10(d) {
  var wrap = document.getElementById('mon10Content');
  if (!d.slaves || !d.slaves.length) {
    wrap.innerHTML = '<div class="f-empty">No data yet\u2026</div>';
    return;
  }

  var combinedLabels   = [];
  var combinedDatasets = [];

  d.slaves.forEach(function(sl) {
    var nch     = sl.ch;
    var rows    = sl.rows || [];
    var blockId = 'mon10block_s' + sl.idx;
    var lt      = sl.loggerType || 0;

    if (!document.getElementById(blockId)) {
      var block = document.createElement('div');
      block.className = 'mon-slave-block';
      block.id        = blockId;

      var ux = currentUnit ? ' (' + currentUnit + ')' : '';
      var hdrCols = '<th>Date</th><th>Time</th>';
      for (var c = 0; c < nch; c++) {
        hdrCols += '<th>' + (lt === 1 ? (c === 0 ? 'PV' : 'SV') : 'CH' + (c + 1)) + ux + '</th>';
      }

      block.innerHTML =
        '<div class="mon-slave-title">' +
          '<span class="mon-slave-tag">Slave ' + (sl.idx + 1) + '</span>' +
          '<span class="mon-slave-name">ID: ' + sl.id + ' | ' + (lt === 1 ? 'PID' : nch + ' CH') + '</span>' +
        '</div>' +
        '<div class="mon-tbl-wrap" id="mon10wrap_s' + sl.idx + '" style="max-height:220px;overflow-y:auto;margin-bottom:12px">' +
          '<table class="mon-tbl">' +
            '<thead><tr>' + hdrCols + '</tr></thead>' +
            '<tbody id="mon10tbody_s' + sl.idx + '"></tbody>' +
          '</table>' +
        '</div>' +
        '<div class="chart-wrap">' +
          '<div class="chart-title">' +
            '<span id="mon10ctitle_s' + sl.idx + '">Slave ' + (sl.idx + 1) + ' \u2014 waiting\u2026</span>' +
          '</div>' +
          '<canvas class="mon-canvas" id="chart10_s' + sl.idx + '" height="180"></canvas>' +
          '<div class="chart-legend" id="leg10_s' + sl.idx + '"></div>' +
        '</div>';

      wrap.appendChild(block);
    }

    // Update table rows — newest on top, scroll position preserved
    var tbody = document.getElementById('mon10tbody_s' + sl.idx);
    if (tbody) {
      var tbl = '';
      for (var r = rows.length - 1; r >= 0; r--) {
        var isLatest = (r === rows.length - 1);
        tbl += '<tr' + (isLatest ? ' class="latest"' : '') + '>';
        tbl += '<td>' + rows[r].ts.substring(0, 10) + '</td>';
        tbl += '<td>' + rows[r].ts.substring(11)    + '</td>';
        for (var c = 0; c < nch; c++) {
          tbl += '<td>' + (rows[r].v[c] !== undefined ? rows[r].v[c].toFixed(1) : '-') + '</td>';
        }
        tbl += '</tr>';
      }
      var tw = document.getElementById('mon10wrap_s' + sl.idx);
      // Preserve scroll position — only set scrollTop on first render
      var wasEmpty = (tbody.innerHTML === '');
      tbody.innerHTML = tbl;
      if (wasEmpty && tw) tw.scrollTop = 0;
    }

    var labels   = rows.map(function(r) { return r.ts.substring(11); });
    var datasets = [];

    for (var c = 0; c < nch; c++) {
      (function(ch) {
        var cname = (lt === 1 ? (ch === 0 ? 'PV' : 'SV') : 'CH' + (ch + 1));
        datasets.push({
          label: cname,
          data: rows.map(function(r) { return r.v[ch] !== undefined ? r.v[ch] : null; }),
          borderColor:     CH_COLORS[ch % CH_COLORS.length],
          backgroundColor: 'transparent',
          borderWidth:     2,
          pointRadius:     3,
          pointHoverRadius:5,
          tension:         0.4
        });
        combinedDatasets.push({
          label: 'S' + (sl.idx + 1) + '-' + cname,
          data: rows.map(function(r) { return r.v[ch] !== undefined ? r.v[ch] : null; }),
          borderColor:     CH_COLORS[(sl.idx * 8 + ch) % CH_COLORS.length],
          backgroundColor: 'transparent',
          borderWidth:     1.5,
          pointRadius:     2,
          tension:         0.4
        });
      })(c);
    }

    if (labels.length) combinedLabels = labels;

    var ct = document.getElementById('mon10ctitle_s' + sl.idx);
    if (ct) ct.textContent = 'Slave ' + (sl.idx + 1) + ' \u2014 Last ' + rows.length + ' readings';

    var chartId = 'chart10_s' + sl.idx;
    var ctx     = document.getElementById(chartId);
    if (ctx && typeof Chart !== 'undefined') {
      try {
        if (charts10[chartId]) {
          charts10[chartId].data.labels = labels;
          while (charts10[chartId].data.datasets.length < datasets.length) {
            charts10[chartId].data.datasets.push(datasets[charts10[chartId].data.datasets.length]);
          }
          charts10[chartId].data.datasets.forEach(function(ds, i) {
            if (datasets[i]) ds.data = datasets[i].data;
          });
          charts10[chartId].update('none');
        } else {
          charts10[chartId] = new Chart(ctx, {
            type: 'line',
            data: { labels: labels, datasets: datasets },
            options: chartOptions()
          });
        }
        renderLegend('leg10_s' + sl.idx, datasets);
      } catch(e) {
        console.warn('Chart error:', e);
        if (charts10[chartId]) { try { charts10[chartId].destroy(); } catch(e2){} delete charts10[chartId]; }
      }
    }
  });

  // Combined chart for multi-slave
  if (d.slaves.length > 1 && combinedLabels.length) {
    var combId = 'mon10block_combined';
    if (!document.getElementById(combId)) {
      var cb2 = document.createElement('div');
      cb2.className = 'mon-slave-block';
      cb2.id        = combId;
      cb2.innerHTML =
        '<div class="mon-slave-title">' +
          '<span class="mon-slave-tag" style="background:linear-gradient(135deg,#059669,#047857)">Combined</span>' +
          '<span class="mon-slave-name">All Slaves Overview</span>' +
        '</div>' +
        '<div class="chart-wrap">' +
          '<div class="chart-title"><span>All Slaves \u2014 Last readings combined</span></div>' +
          '<canvas class="mon-canvas" id="chart10_combined" height="200"></canvas>' +
          '<div class="chart-legend" id="leg10_combined"></div>' +
        '</div>';
      wrap.appendChild(cb2);
    }
    var combCtx = document.getElementById('chart10_combined');
    if (combCtx) {
      if (charts10['chart10_combined']) {
        charts10['chart10_combined'].data.labels = combinedLabels;
        charts10['chart10_combined'].data.datasets.forEach(function(ds, i) {
          if (combinedDatasets[i]) ds.data = combinedDatasets[i].data;
        });
        charts10['chart10_combined'].update('none');
      } else {
        charts10['chart10_combined'] = new Chart(combCtx, {
          type: 'line',
          data: { labels: combinedLabels, datasets: combinedDatasets },
          options: chartOptions()
        });
      }
      renderLegend('leg10_combined', combinedDatasets);
    }
  }
}

// ============================================================
// HISTORY DATA
// ============================================================
function loadHistory() {
  var sl = document.getElementById('histSlave') ? document.getElementById('histSlave').value : '0';
  fetch('/history-data?slave=' + sl + '&page=' + histPage)
    .then(function(r) { return r.json(); })
    .then(function(d) {
      if (d.error) {
        document.getElementById('monHistContent').innerHTML = '<div class="f-empty">' + d.error + '</div>';
        return;
      }
      histTotalPages = d.total || 1;
      if (histPage === 0 && histTotalPages > 1) {
        histPage = histTotalPages - 1;
        loadHistory();
        return;
      }
      renderHistory(d);
    }).catch(function() {});
}

function renderHistory(d) {
  var wrap = document.getElementById('monHistContent');
  wrap.innerHTML = '';
  var nch  = d.ch || 8;
  var rows = d.rows || [];
  var lt   = d.loggerType || 0;

  var meta = document.createElement('div');
  meta.className = 'hist-meta';
  meta.innerHTML =
    'Page <b>' + (d.page + 1) + '</b> of <b>' + d.total + '</b>' +
    ' &nbsp;|&nbsp; Total rows: <b>' + d.lines + '</b>' +
    ' &nbsp;|&nbsp; Showing ' + rows.length + ' rows';
  wrap.appendChild(meta);

  var tblWrap = document.createElement('div');
  tblWrap.className      = 'mon-tbl-wrap';
  tblWrap.style.maxHeight = '300px';

  var ux  = currentUnit ? ' (' + currentUnit + ')' : '';
  var tbl = '<table class="mon-tbl"><thead><tr><th>Date</th><th>Time</th>';
  for (var c = 0; c < nch; c++) {
    tbl += '<th>' + (lt === 1 ? (c === 0 ? 'PV' : 'SV') : 'CH' + (c + 1)) + ux + '</th>';
  }
  tbl += '</tr></thead><tbody>';

  // Render rows in descending order — newest on top
  for (var r = rows.length - 1; r >= 0; r--) {
    var row      = rows[r];
    var isLatest = (r === rows.length - 1 && d.page === d.total - 1);
    tbl += '<tr' + (isLatest ? ' class="latest"' : '') + '>';
    for (var f = 0; f < row.length; f++) tbl += '<td>' + row[f] + '</td>';
    for (var f = row.length; f < nch + 2; f++) tbl += '<td>-</td>';
    tbl += '</tr>';
  }
  tbl += '</tbody></table>';
  tblWrap.innerHTML = tbl;
  wrap.appendChild(tblWrap);

  if (rows.length > 1) {
    var chartWrap = document.createElement('div');
    chartWrap.className = 'chart-wrap';
    var chartId = 'chartHist_s' + d.slave;
    chartWrap.innerHTML =
      '<div class="chart-title"><span>Slave ' + (d.slave + 1) + ' \u2014 Page ' + (d.page + 1) + ' trend</span></div>' +
      '<canvas class="mon-canvas" id="' + chartId + '" height="220"></canvas>' +
      '<div class="chart-legend" id="legHist_s' + d.slave + '"></div>';
    wrap.appendChild(chartWrap);

    var labels   = rows.map(function(r) { return (r[0] || '') + ' ' + (r[1] || ''); });
    var datasets = [];
    for (var c = 0; c < nch; c++) {
      var ci    = c + 2;
      var cname = (lt === 1 ? (c === 0 ? 'PV' : 'SV') : 'CH' + (c + 1));
      datasets.push({
        label: cname,
        data: rows.map(function(r) { var v = parseFloat(r[ci]); return isNaN(v) ? null : v; }),
        borderColor:     CH_COLORS[c % CH_COLORS.length],
        backgroundColor: 'transparent',
        borderWidth:     2,
        pointRadius:     rows.length > 50 ? 0 : 3,
        tension:         0.4
      });
    }

    setTimeout(function() {
      var ctx = document.getElementById(chartId);
      if (!ctx) return;
      if (chartsHist[chartId]) { chartsHist[chartId].destroy(); delete chartsHist[chartId]; }
      chartsHist[chartId] = new Chart(ctx, {
        type: 'line',
        data: { labels: labels, datasets: datasets },
        options: chartOptions()
      });
      renderLegend('legHist_s' + d.slave, datasets);
    }, 50);
  }

  var pager = document.getElementById('histPager');
  if (pager) {
    pager.style.display = d.total > 1 ? 'flex' : 'none';
    var info = document.getElementById('histPageInfo');
    if (info) info.textContent = 'Page ' + (d.page + 1) + ' / ' + d.total;
    var btns = pager.querySelectorAll('button');
    if (btns[0]) btns[0].disabled = (d.page <= 0);
    if (btns[1]) btns[1].disabled = (d.page >= d.total - 1);
  }
}

function histPrev() { if (histPage > 0) { histPage--; loadHistory(); } }
function histNext() { if (histPage < histTotalPages - 1) { histPage++; loadHistory(); } }

// ============================================================
// CHART OPTIONS
// ============================================================
function chartOptions() {
  var isDk     = document.documentElement.getAttribute('data-theme') === 'dark';
  var gridCol  = isDk ? 'rgba(255,255,255,0.06)' : 'rgba(0,0,0,0.07)';
  var tickCol  = isDk ? '#5c5890' : '#8b93c4';
  var labelCol = isDk ? '#9b97cc' : '#6b7280';
  var yLabel   = currentUnit ? currentUnit : 'Value';

  return {
    responsive: true,
    animation:  { duration: 400 },
    interaction: { mode: 'index', intersect: false },
    plugins: {
      legend: { display: false },
      tooltip: {
        backgroundColor: isDk ? '#1a1838' : '#fff',
        titleColor:      isDk ? '#e8e6ff' : '#1e1b4b',
        bodyColor:       isDk ? '#9b97cc' : '#4b5280',
        borderColor:     isDk ? '#2a2750' : '#dde1f9',
        borderWidth:     1,
        padding:         10,
        cornerRadius:    8,
        callbacks: {
          label: function(ctx) {
            return ctx.dataset.label + ': ' + ctx.parsed.y + (currentUnit ? ' ' + currentUnit : '');
          }
        }
      }
    },
    scales: {
      x: {
        ticks: { color: tickCol, font: { size: 10 }, maxRotation: 45, maxTicksLimit: 12 },
        grid:  { color: gridCol },
        title: { display: true, text: 'Time', color: labelCol, font: { size: 10, weight: '600' } }
      },
      y: {
        ticks: { color: tickCol, font: { size: 10 } },
        grid:  { color: gridCol },
        title: { display: true, text: yLabel, color: labelCol, font: { size: 10, weight: '600' } }
      }
    }
  };
}

// ============================================================
// LEGEND RENDERER
// ============================================================
function renderLegend(elId, datasets) {
  var el = document.getElementById(elId);
  if (!el) return;
  el.innerHTML = '';
  datasets.forEach(function(ds) {
    var item = document.createElement('div');
    item.className = 'chart-legend-item';
    item.innerHTML =
      '<span class="legend-dot" style="background:' + ds.borderColor + '"></span>' +
      ds.label;
    el.appendChild(item);
  });
}
</script>
</body>
</html>)CHUNK12"));
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  // Only init NVS – do NOT erase every boot (preserves last good time)
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  delay(300);

  pinMode(RED_LED,  OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER,   OUTPUT);
  digitalWrite(RED_LED,  LOW);
  digitalWrite(BLUE_LED, LOW);
  buzzerInit();
  errorMessage = "";

  for (int i = 0; i < MAX_SLAVES; i++) {
    commFailCounts[i] = 0;
    liveHead[i]       = 0;
    liveCount[i]      = 0;
    batchBuffer[i]    = "";
    batchCount[i]     = 0;
  }

  // Device ID from MAC (last 6 hex chars)
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13];
  snprintf(macStr, sizeof(macStr), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  deviceId = String(macStr);

  Serial.println("\n=== Sigma Controls Logger Starting ===");
  Serial.println("[DEV] Device ID: " + deviceId);
  Serial.printf("[FW] Version %s  build %d\n", FIRMWARE_VERSION, FIRMWARE_BUILD);

  // Load last good time early
  loadLastGoodTime();

  // WiFi mode: AP+STA (SoftAP only used as fallback)
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  delay(300);

  // Init Serial2 with current (default) settings
  reinitSerial2();

  server.on("/",                HTTP_GET,  handleRoot);
  server.on("/start",           HTTP_POST, handleStart);
  server.on("/stop",            HTTP_POST, handleStop);
  server.on("/setTime",         HTTP_GET,  handleSetTime);
  server.on("/download",        HTTP_GET,  handleDownload);
  server.on("/available-dates", HTTP_GET,  handleGetAvailableDates);
  server.on("/files-for-date",  HTTP_GET,  handleGetFilesForDate);
  server.on("/status",          HTTP_GET,  handleStatus);
  server.on("/ota-check",        HTTP_POST, handleOtaCheck);
  server.on("/ota-check",        HTTP_GET,  handleOtaCheck);
  server.on("/setWifi",         HTTP_POST, handleSetWifi);
  server.on("/error-status",    HTTP_GET,  handleErrorStatus);
  server.on("/config",          HTTP_GET,  handleGetConfig);
  server.on("/live-data",       HTTP_GET,  handleLiveData);
  server.on("/history-data",    HTTP_GET,  handleHistoryData);
  // server.on("/jszip.js",        HTTP_GET,  handleJSZip);

  // Captive portal probes — keeps Android/iOS connected to SoftAP without internet
  server.on("/generate_204", HTTP_GET, []() { server.send(204); });
  server.on("/gen_204", HTTP_GET, []() { server.send(204); });
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
  });
  server.on("/library/test/success.html", HTTP_GET, []() {
    server.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
  });
  server.on("/connecttest.txt", HTTP_GET, []() { server.send(200, "text/plain", "Microsoft Connect Test"); });
  server.on("/ncsi.txt", HTTP_GET, []() { server.send(200, "text/plain", "Microsoft NCSI"); });
  server.on("/success.txt", HTTP_GET, []() { server.send(200, "text/plain", "success"); });
  server.on("/fwlink", HTTP_GET, []() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/canonical.html", HTTP_GET, []() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();
  Serial.println("[WEB] Started");

  // SD init with retries (SPI can fail on first try after power-up with peripherals)
  sdCardOK = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    SD.end();
    delay(200);
    if (SD.begin(SD_CS)) {
      sdCardOK = true;
      Serial.printf("[SD] OK (attempt %d) - %lu MB\n", attempt,
                    (unsigned long)(SD.cardSize() / (1024 * 1024)));
      break;
    }
    Serial.printf("[SD] mount failed attempt %d/5\n", attempt);
    delay(300);
  }
  if (!sdCardOK) {
    errorMessage = "SD card not found - insert card and reboot";
    digitalWrite(BLUE_LED, HIGH);
    playBeepPattern(3);
  } else {
    digitalWrite(BLUE_LED, LOW);
    errorMessage = "";
    readOrCreateConfig();
    reinitSerial2();
  }

  // Station WiFi – uses hardcoded SSID/password above (not config.json)
  connectStationWiFi();


  loggingEnabled = false;
  playBeepPattern(1);   // soft boot confirmation

  // OTA rollback: mark valid after ~45s of stable run (see maintainOta)
  Serial.println("[OTA] Will mark app valid after 45s stable run");

  // Auto-resume logging after power loss (unless user had pressed Stop)
  if (sdCardOK && loadLoggingState()) {
    Serial.println("[LOG] Auto-resume: user had left logging ON before power loss");
    if (currentLogDate.length() == 0) currentLogDate = getDateYY();
    loggingEnabled = true;
    lastLogMs = millis() - logIntervalMs;
    baseMillis = millis();
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    logUserAction("Auto-resume after power cycle");
    playBeepPattern(1);
    Serial.printf("[LOG] Resumed session date=%s interval=%lus\n",
                  currentLogDate.c_str(), logIntervalMs / 1000UL);
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
unsigned long lastSDCheckMs = 0;
#define SD_CHECK_INTERVAL 5000

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  // SD card presence check
  if (millis() - lastSDCheckMs >= SD_CHECK_INTERVAL) {
    lastSDCheckMs = millis();
    File root = SD.open("/");
    bool sdPresent = (bool)root;
    if (root) root.close();

    if (!sdPresent && sdCardOK) {
      // Card was removed
      sdCardOK = false;
      errorMessage = "SD card removed - SD error, rebooting in 30s...";
      digitalWrite(BLUE_LED, HIGH);
      playBeepPattern(3);
      if (loggingEnabled) errorSD("SD card removed during logging");

    } else if (sdPresent && !sdCardOK) {
      // Card was re-inserted
      sdCardOK = true;
      errorMessage = "";
      digitalWrite(BLUE_LED, LOW);
      SD.end();
      if (SD.begin(SD_CS)) {
        readOrCreateConfig();
        reinitSerial2();
      } else {
        sdCardOK = false;
        errorMessage = "SD remount failed - SD error";
        digitalWrite(BLUE_LED, HIGH);
        playBeepPattern(3);
      }
    }
  }

  // Maintain WiFi + time
  maintainWiFi();
  maintainOta();
  maintainTime();

  if (!loggingEnabled) {
    delay(10);
    return;
  }

  if (millis() - lastLogMs >= logIntervalMs) {
    lastLogMs = millis();
    // Time is advanced continuously by maintainTime()
    bool anyFail = false;

    for (int s = 0; s < MAX_SLAVES; s++) {
      if (!slaves[s].enabled) continue;

      bool ok = false;
      float pvVal = 0.0f;
      float svVal = 0.0f;
      String line = getDateFull() + "," + getTimeSS() + ",";

      if (globalLoggerType == 1) {
        // ---- PID mode ----
        node.begin(slaves[s].slaveID, Serial2);

        // Read PV register
        uint8_t r1 = node.readHoldingRegisters(slaves[s].pvReg, 1);
        delay(30);
        float rawPV = (r1 == node.ku8MBSuccess) ? (float)node.getResponseBuffer(0) : -1.0f;

        // Read SV register
        uint8_t r2 = node.readHoldingRegisters(slaves[s].svReg, 1);
        delay(30);
        float rawSV = (r2 == node.ku8MBSuccess) ? (float)node.getResponseBuffer(0) : -1.0f;

        ok = (r1 == node.ku8MBSuccess && r2 == node.ku8MBSuccess);

        if (ok) {
          float df = (divFactor > 0.0001f) ? divFactor : 1.0f;
          pvVal = rawPV / df;
          svVal = rawSV / df;
          // Only zero extreme invalid readings (open sensor etc.), not normal high temps
          if (pvVal < -200.0f || pvVal > 10000.0f) pvVal = 0.0f;
          if (svVal < -200.0f || svVal > 10000.0f) svVal = 0.0f;
          line += String(pvVal, 2) + "," + String(svVal, 2);
        } else {
          line += "0.0,0.0";
        }

        // Update live ring buffer
        {
          String ts = getDateFull() + " " + getTimeSS();
          LiveRow& row = liveBuf[s][liveHead[s]];
          strncpy(row.ts, ts.c_str(), sizeof(row.ts) - 1);
          row.ts[sizeof(row.ts) - 1] = 0;
          row.ch[0] = ok ? pvVal : 0.0f;
          row.ch[1] = ok ? svVal : 0.0f;
          liveHead[s] = (liveHead[s] + 1) % LIVE_BUF_SIZE;
          if (liveCount[s] < LIVE_BUF_SIZE) liveCount[s]++;
        }

      } else {
        // ---- TempScanner mode ----
        node.begin(slaves[s].slaveID, Serial2);
        uint8_t result = node.readHoldingRegisters(slaves[s].startReg, slaves[s].modbusChannels);
        ok = (result == node.ku8MBSuccess);
        delay(50);

        float df = (divFactor > 0.0001f) ? divFactor : 1.0f;
        for (int i = 0; i < slaves[s].saveChannels; i++) {
          float raw = ok ? (float)node.getResponseBuffer(i) : 0.0f;
          float val = raw / df;
          // Only zero extreme invalid readings
          if (val < -200.0f || val > 10000.0f) val = 0.0f;
          line += String(val, 2);
          if (i < slaves[s].saveChannels - 1) line += ",";
        }

        // Update live ring buffer
        {
          String ts = getDateFull() + " " + getTimeSS();
          LiveRow& row = liveBuf[s][liveHead[s]];
          strncpy(row.ts, ts.c_str(), sizeof(row.ts) - 1);
          row.ts[sizeof(row.ts) - 1] = 0;
          for (int i = 0; i < slaves[s].saveChannels && i < 24; i++) {
            float rv = ok ? (float)node.getResponseBuffer(i) : 0.0f;
            float cv = rv / df;
            if (cv < -200.0f || cv > 10000.0f) cv = 0.0f;
            row.ch[i] = cv;
          }
          liveHead[s] = (liveHead[s] + 1) % LIVE_BUF_SIZE;
          if (liveCount[s] < LIVE_BUF_SIZE) liveCount[s]++;
        }
      }

      // Handle communication failure
      if (!ok) {
        anyFail = true;
        char buf[80];
        snprintf(buf, sizeof(buf), "Modbus fail - Slave ID %d", slaves[s].slaveID);
        warnCOMM(s, buf);
      } else {
        commFailCounts[s] = 0;
      }

      line += "\n";
      batchBuffer[s] += line;
      batchCount[s]++;

      // Flush to SD card based on interval
      int bsize = (logIntervalMs <= 5000) ? 10 : (logIntervalMs <= 30000) ? 5 : 3;
      if (batchCount[s] >= bsize) {
        File f = SD.open(getSlaveFilename(s), FILE_APPEND);
        if (f) {
          f.print(batchBuffer[s]);
          f.close();
          batchBuffer[s] = "";
          batchCount[s]  = 0;
        } else {
          char buf[64];
          snprintf(buf, sizeof(buf), "SD write failed slave %d", s);
          errorSD(buf);
        }
      }
    }

    // Update RED LED based on overall COMM health
    if (anyFail) {
      digitalWrite(RED_LED, HIGH);
    } else {
      digitalWrite(RED_LED, LOW);
      if (errorMessage.indexOf("Modbus") >= 0 || errorMessage.indexOf("COMM") >= 0) {
        errorMessage = "";
      }
    }
  }

  delay(2);
}
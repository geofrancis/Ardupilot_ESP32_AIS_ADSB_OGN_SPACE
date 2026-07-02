// Force the MAVLink library to build using MAVLink 2.0 packet formats
#define MAVLINK_VERSION 2
#include <WiFi.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h> // Swapped library
#include <time.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "secrets.h"  // WIFI_SSID_x, WIFI_PASS_x, AIS_API_KEY, N2YO_API_KEY, VISUALCROSSING_API_KEY

using namespace websockets; // Global namespace activation for ArduinoWebsockets

// MAVLink diagnostic isolation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/common/mavlink.h"
#pragma GCC diagnostic pop

unsigned long last_space_weather_query = 0;
float space_wx_interval_min = 180.0f;  // Kp index updates every 3 hours

// --- Network Infrastructure ---
const char* OGN_HOST = "aprs.glidernet.org";
float lat;
float lon;
float tide;

// 4. Weather Code State
char weatherCode[32] = "";
void printWeatherCodeDescription(const char* code, char* targetBuf, size_t maxLen);

// ========================================================================
// 1. FREERTOS MUTEXES & THREAD-SAFE CONSTRUCTS
// ========================================================================
SemaphoreHandle_t xGpsMutex = NULL;
SemaphoreHandle_t xCacheMutex = NULL;
SemaphoreHandle_t xSerialMutex = NULL;
SemaphoreHandle_t xAisMutex = NULL;

struct GpsPosition {
  double lat;
  double lon;
  double alt;
  bool ready;
};
GpsPosition getGpsPosition();
bool isGpsReady();

// ========================================================================
// 2. NETWORK CONFIGURATION & HARDWARE PROFILES
// ========================================================================
struct WifiProfile {
  const char* ssid;
  const char* password;
};
const WifiProfile wifi_profiles[] = {
  { WIFI_SSID_1, WIFI_PASS_1 },
  { WIFI_SSID_2, WIFI_PASS_2 }
};
const size_t NUM_WIFI_PROFILES = sizeof(wifi_profiles) / sizeof(WifiProfile);

const char* ogn_host = OGN_HOST;
const char* ais_api_key = AIS_API_KEY;
const int LED_PIN = 22;
#define RX2_PIN 16
#define TX2_PIN 17
#define MAVLINK_SERIAL Serial2

// Component Routing Update
#define MAV_COMP_ID_ADSB 156
char query_ip[46];
char country[64];
char region[64];
char city[64];
char timezone[32];

const uint8_t MAV_SYS_ID = 1;
const uint8_t MAV_COMP_ID_SHARED_OUT = MAV_COMP_ID_ADSB;
const uint32_t DEBUG_VESSEL_MMSI = 999000001;
const char DEBUG_VESSEL_NAME[] = "DEBUG_EAST_10MI";

// Diagnostic System Performance Counters
uint32_t g_count_adsb_packets = 0;
uint32_t g_count_space_packets = 0;
uint32_t g_count_ogn_packets = 0;
uint32_t g_count_ais_packets = 0;
unsigned long g_last_ogn_rx_ms = 0;

// ========================================================================
// 3. LIVE MAVLINK PARAMETERS (WITH FLASH STORAGE PERSISTENCE)
// ========================================================================
float sat_enable = 1.0f;
float sat_scan_int = 60.0f;
float sat_rad_deg = 11.0f;
float sat_cat = 0.0f;
float adsb_radius = 90.0f;
float adsb_scan_int = 15.0f;
float ogn_port_val = 14580.0f;
float ais_dbg_en = 1.0f;
float ais_dbg_dist = 10.0f;
float ais_prnt_int = 5.0f;
float wifi_prof_id = 0.0f;
float wifi_retry_max = 5.0f;
float wifi_timeout = 15.0f;
float adsb_rate_tot = 35.0f;  // Max combined packets/sec
float adsb_rate_veh = 1.0f;   // Max per vehicle

// Weather control parameters
float wx_enable = 1.0f;         // 1 = enabled, 0 = disabled
float wx_interval_min = 15.0f;  // minutes between weather pulls

#define ONBOARD_PARAM_COUNT 17
struct OnboardParameter {
  char id[16];
  float* value_ptr;
  uint8_t type;
};

OnboardParameter local_params[ONBOARD_PARAM_COUNT] = {
  { "SAT_ENABLE", &sat_enable, MAV_PARAM_TYPE_REAL32 },
  { "SAT_SCAN_INT", &sat_scan_int, MAV_PARAM_TYPE_REAL32 },
  { "SAT_RAD_DEG", &sat_rad_deg, MAV_PARAM_TYPE_REAL32 },
  { "SAT_CAT", &sat_cat, MAV_PARAM_TYPE_REAL32 },
  { "ADSB_RADIUS", &adsb_radius, MAV_PARAM_TYPE_REAL32 },
  { "ADSB_SCAN_INT", &adsb_scan_int, MAV_PARAM_TYPE_REAL32 },
  { "OGN_PORT", &ogn_port_val, MAV_PARAM_TYPE_REAL32 },
  { "AIS_DBG_EN", &ais_dbg_en, MAV_PARAM_TYPE_REAL32 },
  { "AIS_DBG_DIST", &ais_dbg_dist, MAV_PARAM_TYPE_REAL32 },
  { "AIS_PRNT_INT", &ais_prnt_int, MAV_PARAM_TYPE_REAL32 },
  { "WIFI_PROF_ID", &wifi_prof_id, MAV_PARAM_TYPE_REAL32 },
  { "WIFI_RETRY_MAX", &wifi_retry_max, MAV_PARAM_TYPE_REAL32 },
  { "WIFI_TIMEOUT", &wifi_timeout, MAV_PARAM_TYPE_REAL32 },
  { "ADSB_RATE_TOT", &adsb_rate_tot, MAV_PARAM_TYPE_REAL32 },
  { "ADSB_RATE_VEH", &adsb_rate_veh, MAV_PARAM_TYPE_REAL32 },
  { "WX_ENABLE", &wx_enable, MAV_PARAM_TYPE_REAL32 },
  { "WX_INTERVAL", &wx_interval_min, MAV_PARAM_TYPE_REAL32 }
};

void loadParametersFromNVS() {
  Preferences prefs;
  prefs.begin("mav_params", false);
  Serial.println("\n[NVS Storage] Loading parameters from non-volatile memory...");
  for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
    if (prefs.isKey(local_params[i].id)) {
      *(local_params[i].value_ptr) = prefs.getFloat(local_params[i].id);
      Serial.printf(" -> %s: %.2f\n", local_params[i].id, *(local_params[i].value_ptr));
    } else {
      prefs.putFloat(local_params[i].id, *(local_params[i].value_ptr));
      Serial.printf(" -> %s: %.2f (Saved Default)\n", local_params[i].id, *(local_params[i].value_ptr));
    }
  }
  prefs.end();
}

void saveParameterToNVS(const char* param_id, float value) {
  Preferences prefs;
  prefs.begin("mav_params", false);
  prefs.putFloat(param_id, value);
  prefs.end();
  Serial.printf("[NVS Storage] Synchronized %s (%.2f) to flash sectors.\n", param_id, value);
}

// ========================================================================
// 3b. WEATHER ENGINE (VisualCrossing → MAVLink STATUSTEXT + ALERTS)
// ========================================================================
unsigned long last_weather_query = 0;
void sendMavlinkStatusText(const char* text) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(
    MAV_SYS_ID,
    MAV_COMP_ID_SHARED_OUT,
    &msg,
    MAV_SEVERITY_INFO,
    text,
    0,
    0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  MAVLINK_SERIAL.write(buf, len);
  xSemaphoreGive(xSerialMutex);
}

void epochToTimeStr(long epoch, char* buffer, size_t size) {
  time_t rawtime = (time_t)epoch;
  struct tm* timeinfo = gmtime(&rawtime);
  strftime(buffer, size, "%H:%M", timeinfo);
}

void fetchWeatherAndSendMavlink() {
  if (wx_enable < 0.5f) return;
  GpsPosition gps = getGpsPosition();
  if (!gps.ready) {
    Serial.println("[WX] Skipping weather fetch: GPS not ready.");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WX] Skipping weather fetch: WiFi not connected.");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  char url[512];
  snprintf(url, sizeof(url),
    "https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/"
    "%.5f,%.5f?unitGroup=metric"
    "&elements=temp,feelslike,tempmax,tempmin,dew,precip,precipprob,preciptype,"
    "snow,snowdepth,windspeed,windgust,winddir,cloudcover,visibility,"
    "sunriseEpoch,sunsetEpoch,moonphase,uvindex,solarradiation,conditions,icon"
    "&include=current,days&key=%s&contentType=json",
    gps.lat, gps.lon, VISUALCROSSING_API_KEY);
  if (!http.begin(client, url)) {
    Serial.println("[WX] HTTP begin failed.");
    return;
  }
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[WX] HTTP error: %d\n", code);
    http.end();
    return;
  }
  String payload = http.getString();
  http.end();
  JsonDocument filter;
  filter["currentConditions"]["temp"] = true;
  filter["currentConditions"]["feelslike"] = true;
  filter["currentConditions"]["dew"] = true;
  filter["currentConditions"]["precip"] = true;
  filter["currentConditions"]["precipprob"] = true;
  filter["currentConditions"]["preciptype"][0] = true;
  filter["currentConditions"]["snow"] = true;
  filter["currentConditions"]["snowdepth"] = true;
  filter["currentConditions"]["windspeed"] = true;
  filter["currentConditions"]["windgust"] = true;
  filter["currentConditions"]["winddir"] = true;
  filter["currentConditions"]["cloudcover"] = true;
  filter["currentConditions"]["visibility"] = true;
  filter["currentConditions"]["uvindex"] = true;
  filter["currentConditions"]["solarradiation"] = true;
  filter["currentConditions"]["conditions"] = true;
  filter["currentConditions"]["icon"] = true;
  filter["days"][0]["tempmax"] = true;
  filter["days"][0]["tempmin"] = true;
  filter["days"][0]["sunriseEpoch"] = true;
  filter["days"][0]["sunsetEpoch"] = true;
  filter["days"][0]["moonphase"] = true;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
  if (err) {
    Serial.printf("[WX] JSON parse error: %s\n", err.c_str());
    return;
  }
  float temp = doc["currentConditions"]["temp"] | 0.0f;
  float feels = doc["currentConditions"]["feelslike"] | 0.0f;
  float dew = doc["currentConditions"]["dew"] | 0.0f;
  float tempmax = doc["days"][0]["tempmax"] | 0.0f;
  float tempmin = doc["days"][0]["tempmin"] | 0.0f;
  float precip = doc["currentConditions"]["precip"] | 0.0f;
  float precipProb = doc["currentConditions"]["precipprob"] | 0.0f;
  const char* pType = doc["currentConditions"]["preciptype"][0] | "none";
  float snow = doc["currentConditions"]["snow"] | 0.0f;
  float snowDepth = doc["currentConditions"]["snowdepth"] | 0.0f;
  float windSpeed = doc["currentConditions"]["windspeed"] | 0.0f;
  float windGust = doc["currentConditions"]["windgust"] | 0.0f;
  int windDir = doc["currentConditions"]["winddir"] | 0;
  float cloud = doc["currentConditions"]["cloudcover"] | 0.0f;
  float vis = doc["currentConditions"]["visibility"] | 0.0f;
  float uv = doc["currentConditions"]["uvindex"] | 0.0f;
  float sol = doc["currentConditions"]["solarradiation"] | 0.0f;
  long sunriseE = doc["days"][0]["sunriseEpoch"] | 0;
  long sunsetE = doc["days"][0]["sunsetEpoch"] | 0;
  float moon = doc["days"][0]["moonphase"] | 0.0f;
  strncpy(weatherCode, doc["currentConditions"]["icon"] | "", sizeof(weatherCode) - 1);
  char sr[6], ss[6];
  epochToTimeStr(sunriseE, sr, sizeof(sr));
  epochToTimeStr(sunsetE, ss, sizeof(ss));
  char buf[13][50];
  snprintf(buf[0], 50, "WX 1/13: T:%.1fC Feel:%.1fC Dew:%.1fC", temp, feels, dew);
  snprintf(buf[1], 50, "WX 2/13: Max:%.1fC Min:%.1fC", tempmax, tempmin);
  snprintf(buf[2], 50, "WX 3/13: Rain:%.1fmm(%.0f%%) %s", precip, precipProb, pType);
  snprintf(buf[3], 50, "WX 4/13: Snow:%.1fcm Depth:%.1fcm", snow, snowDepth);
  snprintf(buf[4], 50, "WX 5/13: Wind:%.1f G:%.1f Dir:%d", windSpeed, windGust, windDir);
  snprintf(buf[5], 50, "WX 6/13: Cloud:%.0f%% Vis:%.1fkm", cloud, vis);
  snprintf(buf[6], 50, "WX 7/13: UV:%.1f Rad:%.0fW/m2", uv, sol);
  snprintf(buf[7], 50, "WX 8/13: SR:%s SS:%s MP:%.2f", sr, ss, moon);
  snprintf(buf[8], 50, "WX 9/13: Tide:%.1fm", tide);
  snprintf(buf[9], 50, "WX 10/13: IP:%s Lat:%.4f Lon:%.4f", query_ip, lat, lon);
  snprintf(buf[10], 50, "WX 11/13: %s, %s, %s", city, region, country);
  snprintf(buf[11], 50, "WX 12/13: Raw:%s", weatherCode);
  printWeatherCodeDescription(weatherCode, buf[12], 50);
  Serial.println("[WX] Injecting 13-part weather/geo sequence via MAVLink STATUSTEXT...");
  for (int i = 0; i < 13; i++) {
    sendMavlinkStatusText(buf[i]);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  if (windSpeed > 30.0f || windGust > 40.0f) {
    sendMavlinkStatusText("WX ALERT: Strong winds aloft.");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  if (precipProb > 70.0f && precip > 2.0f) {
    sendMavlinkStatusText("WX ALERT: Heavy precipitation likely.");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  if (vis < 3.0f) {
    sendMavlinkStatusText("WX ALERT: Reduced visibility.");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  if (uv >= 7.0f) {
    sendMavlinkStatusText("WX ALERT: High UV exposure.");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("[WX] Weather transmission burst complete.");
}

void fetchSpaceWeatherAndSendMavlink() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  char url[256];
  snprintf(url, sizeof(url), "https://kp.gfz.de/app/json/?start=2026-06-23T00:00:00Z&end=2026-06-24T23:59:59Z&index=Kp&status=nowcast");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) {
    Serial.println("[SPACE WX] HTTP secure connection sequence failed initialization.");
    return;
  }
  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.printf("[SPACE WX] HTTP GET exception: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return;
  }
  String payload = http.getString();
  http.end();
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.printf("[SPACE WX] Parsing collision: %s\n", error.c_str());
    return;
  }
  JsonArray kpArray = doc["Kp"].as<JsonArray>();
  JsonArray timeArray = doc["datetime"].as<JsonArray>();
  if (kpArray.size() > 0) {
    int lastIndex = kpArray.size() - 1;
    float latestKp = kpArray[lastIndex].as<float>();
    const char* latestTimeFull = timeArray[lastIndex] | "";
    char latestTime[6] = "";
    if (strlen(latestTimeFull) >= 16) {
      memcpy(latestTime, latestTimeFull + 11, 5);
      latestTime[5] = '\0';
    } else {
      strncpy(latestTime, latestTimeFull, 5);
    }
    const char* stormStatus = "QUIET";
    if (latestKp >= 4.0f && latestKp < 5.0f) stormStatus = "ACTIVE";
    else if (latestKp >= 5.0f && latestKp < 6.0f) stormStatus = "G1 MINOR STORM";
    else if (latestKp >= 6.0f && latestKp < 7.0f) stormStatus = "G2 MODERATE STORM";
    else if (latestKp >= 7.0f && latestKp < 8.0f) stormStatus = "G3 STRONG STORM";
    else if (latestKp >= 8.0f) stormStatus = "G4/G5 SEVERE STORM";
    char spaceBuf[50];
    snprintf(spaceBuf, sizeof(spaceBuf), "SPACE WX: Kp=%.1f (%s) UTC:%s", latestKp, stormStatus, latestTime);
    Serial.printf("[SPACE WX] Injecting MAVLink payload: %s\n", spaceBuf);
    sendMavlinkStatusText(spaceBuf);
    if (latestKp >= 5.0f) {
      vTaskDelay(pdMS_TO_TICKS(500));
      sendMavlinkStatusText("SPACE WX ALERT: High ionospheric activity. Risk of GPS/RTK degradation.");
    }
  } else {
    Serial.println("[SPACE WX] API returned empty dataset array.");
  }
}

void getGeoLocation() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    Serial.println("Fetching geolocation data...");
    http.setTimeout(5000);
    http.begin(ip_api_url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      JsonDocument doc;
      String payload = http.getString();
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.print("JSON Deserialization failed: ");
        Serial.println(error.f_str());
        return;
      }
      if (doc["status"] == "success") {
        Serial.println("RAW PAYLOAD:");
        Serial.println(payload);
        strncpy(query_ip, doc["query"] | "0.0.0.0", sizeof(query_ip) - 1);
        strncpy(country, doc["country"] | "Unknown", sizeof(country) - 1);
        strncpy(region, doc["regionName"] | "Unknown", sizeof(region) - 1);
        strncpy(city, doc["city"] | "Unknown", sizeof(city) - 1);
        strncpy(timezone, doc["timezone"] | "Unknown", sizeof(timezone) - 1);
        lat = doc["lat"] | 0.0f;
        lon = doc["lon"] | 0.0f;
        Serial.println("\n--- GEOLOCATION RESULTS ---");
        Serial.printf("IP Address: %s\n", query_ip);
        Serial.printf("Country:    %s\n", country);
        Serial.printf("Region:     %s\n", region);
        Serial.printf("City:       %s\n", city);
        Serial.printf("Timezone:   %s\n", timezone);
        Serial.printf("Latitude:   %.6f\n", lat);
        Serial.printf("Longitude:  %.6f\n", lon);
        Serial.println("---------------------------\n");
      } else {
        Serial.println("API returned a failed status.");
      }
    } else {
      Serial.print("Error code on HTTP request: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Wi-Fi Disconnected");
  }
}

void getAddressFromCoords(float lat, float lon) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Error: Not connected to Wi-Fi.");
    return;
  }
  HTTPClient http;
  char url[192];
  snprintf(url, sizeof(url), "https://nominatim.openstreetmap.org/reverse?lat=%.6f&lon=%.6f&format=jsonv2", lat, lon);
  Serial.println("\nSending Request to Nominatim...");
  http.setTimeout(5000);
  http.begin(url);
  http.setUserAgent("ESP32-Address-Finder/1.0 (contact: o.o.o.o@hotmail.com)");
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      const char* displayName = doc["display_name"];
      const char* road = doc["address"]["road"];
      const char* city = doc["address"]["city"];
      const char* postcode = doc["address"]["postcode"];
      Serial.println("=========================================");
      Serial.print("Full Address: ");
      Serial.println(displayName);
      Serial.println("-----------------------------------------");
      if (road) { Serial.print("Road: "); Serial.println(road); }
      if (city) { Serial.print("City: "); Serial.println(city); }
      if (postcode) { Serial.print("Postcode: "); Serial.println(postcode); }
      Serial.println("=========================================");
    } else {
      Serial.print("JSON Parsing failed: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("HTTP Error. Code: ");
    Serial.println(httpCode);
  }
  http.end();
}

// ========================================================================
// 4. DEAD RECKONING ENGINE (TARGET EXTRAPOLATION CACHE)
// ========================================================================
#define MAX_TRACKED_TARGETS 45
struct TrackedTarget {
  uint32_t id;
  char callsign[9];
  double lat;
  double lon;
  float alt_ft;
  float heading_deg;
  float speed_knots;
  float climb_rate_fps;
  uint8_t emitter_type;
  unsigned long last_real_ms;
  unsigned long last_step_ms;
  unsigned long last_mavlink_sent_ms;
  bool active;
};
TrackedTarget global_cache[MAX_TRACKED_TARGETS];

void send_mavlink_adsb(uint32_t icao, double lat, double lon, int32_t alt_ft, float heading_deg, const char* callsign, uint8_t emitter_type);

void updateTargetCache(uint32_t id, double lat, double lon, float alt_ft, float heading, float speed, float climb_fps, const char* callsign, uint8_t emitter) {
  unsigned long now = millis();
  int target_idx = -1;
  int empty_idx = -1;
  xSemaphoreTake(xCacheMutex, portMAX_DELAY);
  for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
    if (global_cache[i].active && global_cache[i].id == id) {
      target_idx = i;
      break;
    }
    if (!global_cache[i].active && empty_idx == -1) {
      empty_idx = i;
    }
  }
  if (target_idx == -1) {
    if (empty_idx == -1) {
      xSemaphoreGive(xCacheMutex);
      return;
    }
    target_idx = empty_idx;
    global_cache[target_idx].active = true;
    global_cache[target_idx].id = id;
    global_cache[target_idx].heading_deg = (heading >= 0.0f) ? heading : 0.0f;
    global_cache[target_idx].last_mavlink_sent_ms = 0;
    if (emitter == 15) {
      global_cache[target_idx].speed_knots = 1450.0f;
      global_cache[target_idx].heading_deg = (heading >= 0.0f) ? heading : 0.0f;
    }
  }
  TrackedTarget& t = global_cache[target_idx];
  if ((heading < 0.0f || emitter == 15) && t.last_real_ms > 0) {
    double elapsed_sec = (now - t.last_real_ms) / 1000.0;
    if (elapsed_sec > 3.0) {
      double dy = (lat - t.lat) * 111320.0;
      double dx = (lon - t.lon) * 111320.0 * cos(lat * M_PI / 180.0);
      float dist_meters = sqrt(dx * dx + dy * dy);
      if (emitter == 15) {
        t.speed_knots = (dist_meters / elapsed_sec) / 0.514444f;
      } else if (speed >= 0.0f) {
        t.speed_knots = speed;
      }
      if (dist_meters > 1.5f) {
        float angle = atan2(dx, dy) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;
        t.heading_deg = angle;
      }
    }
  } else if (heading >= 0.0f) {
    t.heading_deg = heading;
    t.speed_knots = speed;
  }
  t.lat = lat;
  t.lon = lon;
  t.alt_ft = alt_ft;
  t.climb_rate_fps = climb_fps;
  t.emitter_type = emitter;
  t.last_real_ms = now;
  t.last_step_ms = now;
  strncpy(t.callsign, callsign, 8);
  t.callsign[8] = '\0';
  xSemaphoreGive(xCacheMutex);
}

void processExtrapolatedTargets() {
  unsigned long now = millis();
  xSemaphoreTake(xCacheMutex, portMAX_DELAY);
  static unsigned long adsb_rate_window_ms = 0;
  static uint32_t adsb_sent_this_sec = 0;
  if (now - adsb_rate_window_ms >= 1000) {
    adsb_sent_this_sec = 0;
    adsb_rate_window_ms = now;
  }
  for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
    if (!global_cache[i].active) continue;
    if (now - global_cache[i].last_real_ms > 120000) {
      global_cache[i].active = false;
      continue;
    }
    float dt = (now - global_cache[i].last_step_ms) / 1000.0f;
    if (dt < 0.2f) continue;
    global_cache[i].last_step_ms = now;
    float speed_mps = global_cache[i].speed_knots * 0.514444f;
    float distance_moved_meters = speed_mps * dt;
    float heading_rad = global_cache[i].heading_deg * M_PI / 180.0f;
    double delta_lat = (distance_moved_meters * cos(heading_rad)) / 111320.0;
    double lat_rad = global_cache[i].lat * M_PI / 180.0;
    double delta_lon = (distance_moved_meters * sin(heading_rad)) / (111320.0 * cos(lat_rad));
    global_cache[i].lat += delta_lat;
    global_cache[i].lon += delta_lon;
    global_cache[i].alt_ft += (global_cache[i].climb_rate_fps * dt);
    unsigned long target_interval = (global_cache[i].emitter_type == 15) ? 0 : 1000;
    if (adsb_rate_veh > 0.0f) {
      unsigned long vehicle_max_interval = (unsigned long)(1000.0f / adsb_rate_veh);
      if (vehicle_max_interval > target_interval) {
        target_interval = vehicle_max_interval;
      }
    }
    if (now - global_cache[i].last_mavlink_sent_ms >= target_interval) {
      if (adsb_rate_tot > 0.0f && adsb_sent_this_sec >= (uint32_t)adsb_rate_tot) {
        continue;
      }
      send_mavlink_adsb(
        global_cache[i].id, global_cache[i].lat, global_cache[i].lon,
        (int32_t)global_cache[i].alt_ft, global_cache[i].heading_deg, global_cache[i].callsign, global_cache[i].emitter_type);
      global_cache[i].last_mavlink_sent_ms = now;
      adsb_sent_this_sec++;
    }
  }
  xSemaphoreGive(xCacheMutex);
}

// ========================================================================
// 5. SYSTEM RUNTIME CORE VARIABLES
// ========================================================================
WiFiClient ognClient;
double current_lat = 0.0;
double current_lon = 0.0;
double current_alt_meters = 0.0;
bool gps_ready = false;
unsigned long last_heartbeat_adsb = 0;
unsigned long last_heartbeat_ais = 0;
unsigned long last_ogn_heartbeat = 0;
unsigned long last_gps_wait_print = 0;
unsigned long last_adsb_api_query = 0;
unsigned long last_satellite_query = 0;
unsigned long last_ais_print_time = 0;
unsigned long last_packet_flash_ms = 0;
unsigned long last_wifi_check_ms = 0;
unsigned long last_dead_reckon_ms = 0;
unsigned long last_diagnostic_print = 0;
uint32_t led_blink_interval = 1000;
const unsigned long FLASH_DURATION = 50;

void handleGpsFixAcquisition(const char* source);
void send_mavlink_param(uint8_t index);
void handleWiFiConnection();

GpsPosition getGpsPosition() {
  xSemaphoreTake(xGpsMutex, portMAX_DELAY);
  GpsPosition pos = { current_lat, current_lon, current_alt_meters, gps_ready };
  xSemaphoreGive(xGpsMutex);
  return pos;
}

bool isGpsReady() {
  xSemaphoreTake(xGpsMutex, portMAX_DELAY);
  bool ready = gps_ready;
  xSemaphoreGive(xGpsMutex);
  return ready;
}

// ========================================================================
// 6. AIS SERVICE (WEBSOCKETS SUB-ASYNC RING)
// ========================================================================
namespace services::ais {
constexpr size_t kMaxVessels = 64;
struct Vessel {
  uint32_t mmsi;
  char name[32];
  char type[16];
  char speed[16];
  float sog_knots;
  float lat;
  float lon;
  float heading_deg;
  unsigned long last_seen_ms;
  unsigned long last_mavlink_sent_ms;
};

const char* kHost = "stream.aisstream.io";
const char* kPath = "/v0/stream";

// ISRG Root X1 CA Certificate (Valid until 2035)
const char* root_ca ="-----BEGIN CERTIFICATE-----\n" \
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n" \
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n" \
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n" \
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n" \
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n" \
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n" \
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n" \
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n" \
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n" \
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n" \
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n" \
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n" \
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n" \
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n" \
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n" \
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n" \
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n" \
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n" \
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n" \
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n" \
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n" \
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n" \
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n" \
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n" \
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n" \
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n" \
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n" \
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n" \
"-----END CERTIFICATE-----\n";

WebsocketsClient s_ws; // Updated to ArduinoWebsockets class
Vessel s_vessels[kMaxVessels];
size_t s_vessel_count = 0;
char s_api_key[65] = "";
bool s_started = false;
bool s_connected = false;
bool s_dirty = false;
bool s_subscription_pending = false;
char s_status_text[48] = "AIS OFFLINE";
double s_center_lat = 0.0;
double s_center_lon = 0.0;
double s_radius_miles = 0.0;

void setStatus(const char* text) {
  strncpy(s_status_text, text, 47);
  s_dirty = true;
}

bool hasApiKey() {
  return s_api_key[0] != '\0';
}

bool isConnected() {
  return s_connected;
}

Vessel* findOrAllocateVessel(uint32_t mmsi) {
  for (size_t i = 0; i < s_vessel_count; ++i) {
    if (s_vessels[i].mmsi == mmsi) return &s_vessels[i];
  }
  if (s_vessel_count < kMaxVessels) {
    Vessel* v = &s_vessels[s_vessel_count++];
    memset(v, 0, sizeof(*v));
    v->mmsi = mmsi;
    v->last_mavlink_sent_ms = 0;
    return v;
  }
  return &s_vessels[0];
}

void handleTextMessage(const char* payload) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return;
  const char* type = doc["MessageType"] | "";
  JsonObject body = doc["Message"][type];
  if (body.isNull()) return;
  xSemaphoreTake(xAisMutex, portMAX_DELAY);
  uint32_t mmsi = body["UserID"] | 0;
  Vessel* v = findOrAllocateVessel(mmsi);
  v->lat = body["Latitude"] | 0.0f;
  v->lon = body["Longitude"] | 0.0f;
  if (!body["TrueHeading"].isNull()) v->heading_deg = body["TrueHeading"].as<float>();
  if (!body["Sog"].isNull()) v->sog_knots = body["Sog"].as<float>();
  if (doc["MetaData"]["ShipName"].is<const char*>()) {
    strncpy(v->name, doc["MetaData"]["ShipName"], sizeof(v->name) - 1);
  }
  v->last_seen_ms = millis();
  s_dirty = true;
  g_count_ais_packets++;
  xSemaphoreGive(xAisMutex);
}

// Unified ArduinoWebsockets callbacks
void onMessageCallback(WebsocketsMessage message) {
  handleTextMessage(message.data().c_str());
}

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    s_connected = true;
    s_subscription_pending = true;
    setStatus("AIS CONNECTED");
    Serial.println("[AIS Engine] Connected to WebSocket Endpoint successfully.");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    s_connected = false;
    setStatus("AIS DISCONNECTED");
    Serial.println("[AIS Engine] Lost Connection to WebSocket Stream Server.");
  }
}

void buildSubscriptionPayload(char* buf, size_t len) {
  double radius_km = s_radius_miles * 1.609344;
  double lat_offset = radius_km / 111.32;
  double lat_rad = s_center_lat * M_PI / 180.0;
  double lon_km_per_deg = 111.32 * cos(lat_rad);
  if (lon_km_per_deg <= 0.000001) lon_km_per_deg = 0.000001;
  double lon_offset = radius_km / lon_km_per_deg;
  double lat_min = s_center_lat - lat_offset;
  double lat_max = s_center_lat + lat_offset;
  double lon_min = s_center_lon - lon_offset;
  double lon_max = s_center_lon + lon_offset;
  snprintf(buf, len,
    "{\"APIKey\":\"%s\",\"BoundingBoxes\":[[[%.6f,%.6f],[%.6f,%.6f]]]}",
    s_api_key, lat_min, lon_min, lat_max, lon_max);
}

void init(const char* key) {
  strncpy(s_api_key, key, 64);
}

void setBoundingBox(double center_lat, double center_lon, double radius_miles) {
  xSemaphoreTake(xAisMutex, portMAX_DELAY);
  s_center_lat = center_lat;
  s_center_lon = center_lon;
  s_radius_miles = radius_miles;
  s_subscription_pending = true;
  xSemaphoreGive(xAisMutex);
}

void insertOrUpdateDebugVessel(uint32_t mmsi, const char* name, double lat, double lon) {
  xSemaphoreTake(xAisMutex, portMAX_DELAY);
  for (size_t i = 0; i < s_vessel_count; ++i) {
    if (s_vessels[i].mmsi == mmsi) {
      s_vessels[i].lat = lat;
      s_vessels[i].lon = lon;
      s_vessels[i].last_seen_ms = millis();
      s_dirty = true;
      xSemaphoreGive(xAisMutex);
      return;
    }
  }
  if (s_vessel_count < kMaxVessels) {
    Vessel* v = &s_vessels[s_vessel_count++];
    memset(v, 0, sizeof(*v));
    v->mmsi = mmsi;
    strncpy(v->name, name, sizeof(v->name) - 1);
    v->lat = lat;
    v->lon = lon;
    v->last_seen_ms = millis();
    v->last_mavlink_sent_ms = 0;
    s_dirty = true;
  }
  xSemaphoreGive(xAisMutex);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED && hasApiKey()) {
    if (!s_started) {
      Serial.println("[AIS Engine] Initializing Secure WebSocket Handshake Dial...");
      s_ws.onMessage(onMessageCallback);
      s_ws.onEvent(onEventsCallback);
      s_ws.setCACert(root_ca); // Strictly validate using Root CA

      String wssUrl = "wss://" + String(kHost) + String(kPath);
      Serial.printf("[AIS Engine] Connecting strictly to: %s\n", wssUrl.c_str());
      s_ws.connect(wssUrl);
      s_started = true;
    }
    
    s_ws.poll(); // Substituted poll logic into thread

    if (s_subscription_pending && s_connected) {
      char payload[384];
      buildSubscriptionPayload(payload, sizeof(payload));
      Serial.println("[AIS Engine] Sending subscription...");
      if (s_ws.send(payload)) {
        s_subscription_pending = false;
        Serial.println("[AIS Engine] Streaming Geofence Area Payload Dispatched.");
      } else {
        Serial.println("[AIS Engine] Failed to dispatch streaming payload.");
      }
    }
  } else if (WiFi.status() != WL_CONNECTED && s_started) {
    s_started = false;
    s_connected = false;
  }
}

size_t vesselCount() {
  return s_vessel_count;
}

bool consumeDirty() {
  bool d = s_dirty;
  s_dirty = false;
  return d;
}
}

// ========================================================================
// 7. API DATA EXTRACTORS
// ========================================================================
void fetchSatellitesOverhead() {
  if (sat_enable <= 0.5f) return;
  GpsPosition gps = getGpsPosition();
  if (!gps.ready) return;
  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  HTTPClient http;
  http.setTimeout(10000);
  char requestURL[256];
  snprintf(requestURL, sizeof(requestURL),
    "https://api.n2yo.com/rest/v1/satellite/above/%.5f/%.5f/%.2f/%d/%d/&apiKey=%s",
    gps.lat, gps.lon, gps.alt, (int)sat_rad_deg, (int)sat_cat, N2YO_API_KEY);
  if (http.begin(secureClient, requestURL)) {
    int httpResponseCode = http.GET();
    if (httpResponseCode == HTTP_CODE_OK) {
      String jsonPayload = http.getString();
      JsonDocument doc;
      if (!deserializeJson(doc, jsonPayload)) {
        int totalSatellitesDetected = doc["info"]["satcount"].as<int>();
        if (totalSatellitesDetected > 0 && !doc["above"].isNull()) {
          JsonArray satelliteList = doc["above"];
          for (size_t i = 0; i < satelliteList.size(); i++) {
            const char* satelliteName = satelliteList[i]["satname"] | "";
            long noradCatalogId = satelliteList[i]["satid"];
            double trackingLat = satelliteList[i]["satlat"].as<double>();
            double trackingLng = satelliteList[i]["satlng"].as<double>();
            float trackingAltKm = satelliteList[i]["satalt"] | 420.0f;
            int32_t alt_ft = (int32_t)(trackingAltKm * 3280.84f);
            updateTargetCache((uint32_t)noradCatalogId, trackingLat, trackingLng,
                              (float)alt_ft, -1.0f, 14500.0f, 0.0f,
                              satelliteName, 15);
            g_count_space_packets++;
            vTaskDelay(pdMS_TO_TICKS(5));
          }
        }
      }
    }
    http.end();
  }
}

void fetchAndSendToMAVLink() {
  GpsPosition gps = getGpsPosition();
  if (!gps.ready) return;
  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  char apiUrl[192];
  snprintf(apiUrl, sizeof(apiUrl), "https://opendata.adsb.fi/api/v3/lat/%.6f/lon/%.6f/dist/%d", gps.lat, gps.lon, (int)adsb_radius);
  if (http.begin(secureClient, apiUrl)) {
    http.useHTTP10(true);
    http.addHeader("Accept-Encoding", "identity");
    http.addHeader("User-Agent", "ESP32-MAVLink-GCS-Tracker/1.0");
    int httpResponseCode = http.GET();
    if (httpResponseCode == HTTP_CODE_OK) {
      JsonDocument filter;
      filter["ac"][0]["hex"] = true;
      filter["ac"][0]["flight"] = true;
      filter["ac"][0]["lat"] = true;
      filter["ac"][0]["lon"] = true;
      filter["ac"][0]["alt_baro"] = true;
      filter["ac"][0]["gs"] = true;
      filter["ac"][0]["track"] = true;
      filter["ac"][0]["baro_rate"] = true;
      filter["ac"][0]["squawk"] = true;
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
      if (!error) {
        JsonArray aircraftList = doc["ac"].as<JsonArray>();
        int trackedCount = 0;
        for (JsonObject ac : aircraftList) {
          const char* flight = ac["flight"] | "";
          char callsignBuf[9] = { 0 };
          strncpy(callsignBuf, flight, 8);
          const char* hexStr = ac["hex"].as<const char*>();
          if (!hexStr) continue;
          uint32_t icaoAddress = strtoul(hexStr, NULL, 16);
          float gs_knots = ac["gs"].is<float>() ? ac["gs"].as<float>() : 0.0f;
          float baro_rate_fps = (ac["baro_rate"].as<float>() * 0.0166667f);
          float track_deg = -1.0f;
          if (!ac["track"].isNull()) {
            track_deg = ac["track"].as<float>();
          }
          updateTargetCache(icaoAddress, ac["lat"].as<double>(), ac["lon"].as<double>(),
                            ac["alt_baro"].as<float>(), track_deg, gs_knots,
                            baro_rate_fps, callsignBuf, ADSB_EMITTER_TYPE_LIGHT);
          trackedCount++;
          g_count_adsb_packets++;
        }
        Serial.printf("[ADS-B API] Extracted %d airspace profiles into cache registers.\n", trackedCount);
      }
    }
    http.end();
  }
}

void fetchTideData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.setTimeout(5000);
    http.begin("https://environment.data.gov.uk/flood-monitoring/id/stations/E74024");
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      JsonDocument doc;
      deserializeJson(doc, payload);
      tide = doc["items"]["measures"]["latestReading"]["value"];
    }
  } else {
    Serial.println("Error fetching data!");
  }
}

// ========================================================================
// 8. WIFILINK MANAGEMENT ENGINE
// ========================================================================
void handleWiFiConnection() {
  static enum { WIFI_IDLE, WIFI_CONNECTING, WIFI_RETRY } state = WIFI_IDLE;
  static unsigned long start_ms = 0;
  static uint32_t attempts = 0;
  static uint8_t profile_idx = 0;
  static const char* target_ssid = nullptr;
  static const char* target_pass = nullptr;
  if (WiFi.status() == WL_CONNECTED) {
    if (state != WIFI_IDLE) {
      state = WIFI_IDLE;
      attempts = 0;
      Serial.printf("[WiFi Engine] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
      led_blink_interval = 100;
    }
    return;
  }
  unsigned long now = millis();
  unsigned long timeout_ms = (unsigned long)(wifi_timeout * 1000.0f);
  switch (state) {
    case WIFI_IDLE:
      profile_idx = (uint8_t)std::max(0.0f, std::min(wifi_prof_id, (float)(NUM_WIFI_PROFILES - 1)));
      target_ssid = wifi_profiles[profile_idx].ssid;
      target_pass = wifi_profiles[profile_idx].password;
      Serial.printf("\n[WiFi Engine] Connecting via Profile [%d]: %s\n", profile_idx, target_ssid);
      WiFi.disconnect();
      WiFi.begin(target_ssid, target_pass);
      start_ms = now;
      state = WIFI_CONNECTING;
      break;
    case WIFI_CONNECTING:
      if (now - start_ms >= timeout_ms) {
        attempts++;
        if (attempts < (uint32_t)wifi_retry_max) {
          Serial.printf("[WiFi Engine] Retry %u/%u\n", (unsigned int)attempts, (unsigned int)wifi_retry_max);
          WiFi.disconnect();
          WiFi.begin(target_ssid, target_pass);
          start_ms = now;
        } else {
          Serial.println("[WiFi Engine] Handshake timeout.");
          led_blink_interval = 500;
          state = WIFI_RETRY;
        }
      }
      break;
    case WIFI_RETRY:
      if (now - start_ms >= 30000) {
        state = WIFI_IDLE;
      }
      break;
  }
}

void connectToOGN() {
  GpsPosition gps = getGpsPosition();
  if (!gps.ready) return;
  Serial.println("[OGN Socket] Attempting raw APRS TCP Server alignment...");
  if (ognClient.connect(ogn_host, (uint16_t)ogn_port_val)) {
    char loginStr[128];
    snprintf(loginStr, sizeof(loginStr),
      "user ardupilot pass -1 vers ESP32-MAVLink 1.0 filter r/%.4f/%.4f/100\r\n",
      gps.lat, gps.lon);
    ognClient.print(loginStr);
    Serial.println("[OGN Socket] Connected & Handshake filter packet transmitted.");
  } else {
    Serial.println("[OGN Socket] Connection failure.");
  }
}

void handleOGNStream() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!ognClient.connected()) {
    connectToOGN();
  } else {
    while (ognClient.available()) {
      char line[128];
      size_t n = ognClient.readBytesUntil('\n', line, sizeof(line) - 1);
      line[n] = '\0';
      g_count_ogn_packets++;
      g_last_ogn_rx_ms = millis();
    }
  }
}

// ========================================================================
// 9. UPSTREAM/DOWNSTREAM TELEMETRY HANDLERS (MAVLINK TRANSCEIVER)
// ========================================================================
void send_mavlink_heartbeat(uint8_t comp_id, uint8_t type) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(MAV_SYS_ID, comp_id, &msg, type, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  MAVLINK_SERIAL.write(buf, len);
  xSemaphoreGive(xSerialMutex);
}

void send_mavlink_adsb(uint32_t icao, double lat, double lon, int32_t alt_ft, float heading_deg, const char* callsign, uint8_t emitter_type) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int32_t lat_e7 = (int32_t)(lat * 10000000);
  int32_t lon_e7 = (int32_t)(lon * 10000000);
  int32_t alt_mm = (int32_t)(alt_ft * 304.8);
  uint16_t heading_cdeg = (uint16_t)(heading_deg * 100.0f);
  if (heading_cdeg > 35999) heading_cdeg = 0;
  char _callsign[9] = { 0 };
  strncpy(_callsign, callsign, 8);
  mavlink_msg_adsb_vehicle_pack(
    MAV_SYS_ID, MAV_COMP_ID_SHARED_OUT, &msg,
    icao, lat_e7, lon_e7, ADSB_ALTITUDE_TYPE_GEOMETRIC, alt_mm,
    heading_cdeg, 0, 0, _callsign, emitter_type, 1,
    ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_CALLSIGN | ADSB_FLAGS_VALID_HEADING, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  MAVLINK_SERIAL.write(buf, len);
  xSemaphoreGive(xSerialMutex);
}

void sendMavlinkAisData() {
  unsigned long now = millis();
  xSemaphoreTake(xAisMutex, portMAX_DELAY);
  size_t count = services::ais::vesselCount();
  for (size_t i = 0; i < count; i++) {
    auto& v = services::ais::s_vessels[i];
    if (now - v.last_mavlink_sent_ms < 1000) {
      continue;
    }
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_ais_vessel_pack(
      MAV_SYS_ID, MAV_COMP_ID_SHARED_OUT, &msg,
      v.mmsi, (int32_t)(v.lat * 1e7), (int32_t)(v.lon * 1e7),
      0, (uint16_t)(v.heading_deg * 100.0), (uint16_t)(v.sog_knots * 100.0),
      0, 0, 0, 0, 0, 0, 0, "UNKNOWN", v.name, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    MAVLINK_SERIAL.write(buf, len);
    xSemaphoreGive(xSerialMutex);
    v.last_mavlink_sent_ms = now;
  }
  xSemaphoreGive(xAisMutex);
}

void send_mavlink_param(uint8_t index) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  char param_id[16] = { 0 };
  strncpy(param_id, local_params[index].id, 16);
  mavlink_msg_param_value_pack(
    MAV_SYS_ID, MAV_COMP_ID_SHARED_OUT, &msg, param_id,
    *(local_params[index].value_ptr), local_params[index].type, ONBOARD_PARAM_COUNT, index);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  MAVLINK_SERIAL.write(buf, len);
  xSemaphoreGive(xSerialMutex);
}

void handleGpsFixAcquisition(const char* source) {
  if (!gps_ready) {
    gps_ready = true;
    services::ais::setBoundingBox(current_lat, current_lon, 50.0);
    Serial.printf("\n[AIRLOCK LINK OPEN] GPS position alignment achieved via %s: %.6f, %.6f\n", source, current_lat, current_lon);
    if (wx_enable > 0.5f) {
      last_weather_query = millis() - (unsigned long)(wx_interval_min * 60000.0f);
      last_space_weather_query = millis() - (unsigned long)(space_wx_interval_min * 60000.0f);
      Serial.println("[SYS] GPS lock acquired. Core 0 weather pipelines flagged for immediate broadcast.");
    }
  }
  if (ais_dbg_en > 0.5f) {
    double lat_rad = current_lat * M_PI / 180.0;
    double lon_km_per_deg = 111.32 * cos(lat_rad);
    if (lon_km_per_deg <= 0.000001) lon_km_per_deg = 0.000001;
    double delta_deg_lon = ((double)ais_dbg_dist * 1.609344) / lon_km_per_deg;
    services::ais::insertOrUpdateDebugVessel(DEBUG_VESSEL_MMSI, DEBUG_VESSEL_NAME, current_lat, current_lon + delta_deg_lon);
  }
}

void readMAVLink() {
  while (MAVLINK_SERIAL.available()) {
    uint8_t c = MAVLINK_SERIAL.read();
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);
            if (pos.lat != 0 && pos.lon != 0) {
              xSemaphoreTake(xGpsMutex, portMAX_DELAY);
              current_lat = pos.lat / 10000000.0;
              current_lon = pos.lon / 10000000.0;
              current_alt_meters = pos.alt / 1000.0;
              handleGpsFixAcquisition("GLOBAL_POSITION_INT");
              xSemaphoreGive(xGpsMutex);
            }
            break;
          }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t gps_msg;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_msg);
            if (gps_msg.fix_type >= 3 && gps_msg.lat != 0 && gps_msg.lon != 0) {
              xSemaphoreTake(xGpsMutex, portMAX_DELAY);
              current_lat = gps_msg.lat / 10000000.0;
              current_lon = gps_msg.lon / 10000000.0;
              current_alt_meters = gps_msg.alt / 1000.0;
              handleGpsFixAcquisition("GPS_RAW_INT");
              xSemaphoreGive(xGpsMutex);
            }
            break;
          }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          {
            mavlink_param_request_list_t req_list;
            mavlink_msg_param_request_list_decode(&msg, &req_list);
            if (req_list.target_system == MAV_SYS_ID && req_list.target_component == MAV_COMP_ID_SHARED_OUT) {
              for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
                send_mavlink_param(i);
                vTaskDelay(pdMS_TO_TICKS(5));
              }
            }
            break;
          }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
          {
            mavlink_param_request_read_t read_req;
            mavlink_msg_param_request_read_decode(&msg, &read_req);
            if (read_req.target_system == MAV_SYS_ID && read_req.target_component == MAV_COMP_ID_SHARED_OUT) {
              if (read_req.param_index >= 0 && read_req.param_index < ONBOARD_PARAM_COUNT) {
                send_mavlink_param(read_req.param_index);
              } else {
                char nameBuf[17] = { 0 };
                strncpy(nameBuf, read_req.param_id, 16);
                for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
                  if (strcmp(nameBuf, local_params[i].id) == 0) {
                    send_mavlink_param(i);
                    break;
                  }
                }
              }
            }
            break;
          }
        case MAVLINK_MSG_ID_PARAM_SET:
          {
            mavlink_param_set_t set_param;
            mavlink_msg_param_set_decode(&msg, &set_param);
            if (set_param.target_system == MAV_SYS_ID && set_param.target_component == MAV_COMP_ID_SHARED_OUT) {
              char nameBuf[17] = { 0 };
              strncpy(nameBuf, set_param.param_id, 16);
              for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
                if (strcmp(nameBuf, local_params[i].id) == 0) {
                  *(local_params[i].value_ptr) = set_param.param_value;
                  saveParameterToNVS(local_params[i].id, set_param.param_value);
                  send_mavlink_param(i);
                  break;
                }
              }
            }
            break;
          }
      }
    }
  }
}

// ========================================================================
// 10. MULTI-CORE WORKER THREAD ENGINE (FreeRTOS)
// ========================================================================
void vTelemetryTask(void* pvParameters) {
  Serial.println("[OS] Telemetry processing core successfully online on Core 1.");
  while (1) {
    readMAVLink();
    processExtrapolatedTargets();
    sendMavlinkAisData();
    unsigned long now = millis();
    if (now - last_heartbeat_adsb >= 1000) {
      send_mavlink_heartbeat(MAV_COMP_ID_ADSB, MAV_TYPE_ANTENNA_TRACKER);
      last_heartbeat_adsb = now;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vNetworkTask(void* pvParameters) {
  Serial.println("[OS] Network connectivity task successfully online on Core 0.");
  while (1) {
    handleWiFiConnection();
    if (WiFi.status() == WL_CONNECTED) {
      unsigned long now = millis();
      services::ais::loop();
      if (now - last_adsb_api_query >= (unsigned long)(adsb_scan_int * 1000)) {
        fetchAndSendToMAVLink();
        last_adsb_api_query = now;
      }
      if (now - last_satellite_query >= (unsigned long)(sat_scan_int * 1000)) {
        fetchSatellitesOverhead();
        last_satellite_query = now;
      }
      handleOGNStream();
      if (wx_enable > 0.5f) {
        unsigned long interval_ms = (unsigned long)(wx_interval_min * 60000.0f);
        if (now - last_weather_query >= interval_ms) {
          Serial.println("[WX] Triggering scheduled terrestrial weather fetch...");
          fetchTideData();
          getGeoLocation();
          fetchWeatherAndSendMavlink();
          getAddressFromCoords(current_lat, current_lon);
          last_weather_query = now;
        }
        unsigned long space_interval_ms = (unsigned long)(space_wx_interval_min * 60000.0f);
        if (now - last_space_weather_query >= space_interval_ms) {
          Serial.println("[SPACE WX] Triggering scheduled space weather fetch...");
          fetchSpaceWeatherAndSendMavlink();
          last_space_weather_query = now;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void printWeatherCodeDescription(const char* code, char* targetBuf, size_t maxLen) {
  const char* desc = "Unknown";
  if (strcmp(code, "snow") == 0)                      desc = "Snow";
  else if (strcmp(code, "snow-showers-day") == 0)     desc = "Day Snow Showers";
  else if (strcmp(code, "snow-showers-night") == 0)   desc = "Night Snow Showers";
  else if (strcmp(code, "thunder-rain") == 0)         desc = "T-Storm w/ Rain";
  else if (strcmp(code, "thunder-showers-day") == 0)  desc = "Day T-Storms";
  else if (strcmp(code, "thunder-showers-night") == 0) desc = "Night T-Storms";
  else if (strcmp(code, "rain") == 0)                 desc = "Rain";
  else if (strcmp(code, "showers-day") == 0)          desc = "Day Rain Showers";
  else if (strcmp(code, "showers-night") == 0)        desc = "Night Rain Showers";
  else if (strcmp(code, "fog") == 0)                  desc = "Foggy";
  else if (strcmp(code, "wind") == 0)                 desc = "High Winds";
  else if (strcmp(code, "cloudy") == 0)               desc = "Cloudy";
  else if (strcmp(code, "partly-cloudy-day") == 0)    desc = "Partly Cloudy Day";
  else if (strcmp(code, "partly-cloudy-night") == 0)  desc = "Partly Cloudy Night";
  else if (strcmp(code, "clear-day") == 0)            desc = "Clear Day";
  else if (strcmp(code, "clear-night") == 0)          desc = "Clear Night";
  Serial.print("Condition: ");
  Serial.println(desc);
  if (targetBuf != nullptr) {
    snprintf(targetBuf, maxLen, "WX 13/13: Description:%s", desc);
  }
}

// ========================================================================
// 11. BOOT INITIALIZATION & FALLBACK LOOPS
// ========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] Initializing Transponder Aggregator Framework...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  xGpsMutex = xSemaphoreCreateMutex();
  xCacheMutex = xSemaphoreCreateMutex();
  xSerialMutex = xSemaphoreCreateMutex();
  xAisMutex = xSemaphoreCreateMutex();
  MAVLINK_SERIAL.begin(57600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  loadParametersFromNVS();
  services::ais::init(ais_api_key);
  xTaskCreatePinnedToCore(vNetworkTask, "NetworkTask", 12288, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vTelemetryTask, "TelemetryTask", 8192, NULL, 2, NULL, 1);
  Serial.println("[BOOT] Initial setup checks complete. Multi-core operations active.");
}

void loop() {
  unsigned long now = millis();
  if (now - last_diagnostic_print >= 5000) {
    Serial.println("==================== [SYSTEM HEALTH MONITOR] ====================");
    Serial.printf(" Terrestrial ADS-B (ADSB.fi)   : %u tracked\n", (unsigned int)g_count_adsb_packets);
    Serial.printf(" Space-Based ADS-B (Satellites): %u tracked\n", (unsigned int)g_count_space_packets);
    Serial.printf(" Glider Network (OGN/APRS)     : %u tracked\n", (unsigned int)g_count_ogn_packets);
    Serial.printf(" Marine Surface (AIS Stream)   : %u tracked\n", (unsigned int)g_count_ais_packets);
    Serial.printf(" Tide Level                    : %.2f m\n", tide);
    Serial.print("  Raw Weather Code: ");
    Serial.println(weatherCode);
    Serial.print("  Description:      ");
    Serial.println("=================================================================");
    last_diagnostic_print = now;
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
}

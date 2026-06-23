// Force the MAVLink library to build using MAVLink 2.0 packet formats
#define MAVLINK_VERSION 2

#include <WiFi.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <time.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>  
#include <algorithm>
#include <cmath>
#include <cstring>

#include "secrets.h" // Include the new secrets file
// MAVLink diagnostic isolation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/common/mavlink.h"
#pragma GCC diagnostic pop



// --- Network Infrastructure ---
const char* OGN_HOST = "aprs.glidernet.org";

// ========================================================================
// 1. FREERTOS MUTEXES & THREAD-SAFE CONSTRUCTS
// ========================================================================
SemaphoreHandle_t xGpsMutex    = NULL; 
SemaphoreHandle_t xCacheMutex  = NULL; 
SemaphoreHandle_t xSerialMutex = NULL; 
SemaphoreHandle_t xAisMutex    = NULL; 

struct GpsPosition {
    double lat;
    double lon;
    double alt;
    bool ready;
};

// ========================================================================
// 2. NETWORK CONFIGURATION & HARDWARE PROFILES
// ========================================================================
struct WifiProfile {
    const char* ssid;
    const char* password;
};

// Now using variables from secrets.h
const WifiProfile wifi_profiles[] = {
    {WIFI_SSID_1, WIFI_PASS_1},      
    {WIFI_SSID_2, WIFI_PASS_2} 
};
const size_t NUM_WIFI_PROFILES = sizeof(wifi_profiles) / sizeof(WifiProfile);

// Now using variables from secrets.h
const char* ogn_host = OGN_HOST;
const char* ais_api_key = AIS_API_KEY;


const int LED_PIN = 22;
#define RX2_PIN 16
#define TX2_PIN 17
#define MAVLINK_SERIAL Serial2


const uint8_t MAV_SYS_ID = 1;
const uint8_t MAV_COMP_ID_ADSB_OUT = MAV_COMP_ID_ADSB; 
const uint8_t MAV_COMP_ID_AIS_OUT = 191;               
const uint32_t DEBUG_VESSEL_MMSI = 999000001;
const char DEBUG_VESSEL_NAME[] = "DEBUG_EAST_10MI";

// Diagnostic System Performance Counters (Separated ADS-B Sources)
uint32_t g_count_adsb_packets  = 0; // Terrestrial ADS-B
uint32_t g_count_space_packets = 0; // Space-Based ADS-B / Satellites
uint32_t g_count_ogn_packets   = 0; // OGN Glider Network
uint32_t g_count_ais_packets   = 0; // Maritime AIS
unsigned long g_last_ogn_rx_ms = 0;

// ========================================================================
// 3. LIVE MAVLINK PARAMETERS (WITH FLASH STORAGE PERSISTENCE)
// ========================================================================
float sat_enable    = 1.0f;     
float sat_scan_int  = 60.0f;    
float sat_rad_deg   = 11.0f;    
float sat_cat       = 0.0f;     
float adsb_radius   = 90.0f;    
float adsb_scan_int = 15.0f;    
float ogn_port_val  = 14580.0f; 
float ais_dbg_en    = 1.0f;     
float ais_dbg_dist  = 10.0f;    
float ais_prnt_int  = 5.0f;     
float wifi_prof_id  = 0.0f;     
float wifi_retry_max = 5.0f;    
float wifi_timeout   = 15.0f;   
float adsb_rate_tot  = 35.0f;   // Maximum combined packets/sec pushed upstream
float adsb_rate_veh  = 1.0f;     // Maximum update frequency allowed per space asset/vehicle

#define ONBOARD_PARAM_COUNT 15
struct OnboardParameter {
    char id[16];
    float* value_ptr;
    uint8_t type;
};

OnboardParameter local_params[ONBOARD_PARAM_COUNT] = {
    {"SAT_ENABLE",   &sat_enable,    MAV_PARAM_TYPE_REAL32},
    {"SAT_SCAN_INT", &sat_scan_int,  MAV_PARAM_TYPE_REAL32},
    {"SAT_RAD_DEG",  &sat_rad_deg,   MAV_PARAM_TYPE_REAL32},
    {"SAT_CAT",      &sat_cat,       MAV_PARAM_TYPE_REAL32},
    {"ADSB_RADIUS",  &adsb_radius,   MAV_PARAM_TYPE_REAL32},
    {"ADSB_SCAN_INT",&adsb_scan_int, MAV_PARAM_TYPE_REAL32},
    {"OGN_PORT",     &ogn_port_val,  MAV_PARAM_TYPE_REAL32},
    {"AIS_DBG_EN",   &ais_dbg_en,    MAV_PARAM_TYPE_REAL32},
    {"AIS_DBG_DIST", &ais_dbg_dist,  MAV_PARAM_TYPE_REAL32},
    {"AIS_PRNT_INT", &ais_prnt_int,  MAV_PARAM_TYPE_REAL32},
    {"WIFI_PROF_ID", &wifi_prof_id,  MAV_PARAM_TYPE_REAL32},
    {"WIFI_RETRY_MAX",&wifi_retry_max,MAV_PARAM_TYPE_REAL32},
    {"WIFI_TIMEOUT", &wifi_timeout,  MAV_PARAM_TYPE_REAL32},
    {"ADSB_RATE_TOT",&adsb_rate_tot, MAV_PARAM_TYPE_REAL32},
    {"ADSB_RATE_VEH",&adsb_rate_veh, MAV_PARAM_TYPE_REAL32}
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

void send_mavlink_adsb(uint32_t icao, double lat, double lon, int32_t alt_ft, float heading_deg, String callsignStr, uint8_t emitter_type);

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
            float dist_meters = sqrt(dx*dx + dy*dy);

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

    // Dynamic 1-Second Sliding Window tracking for Global Rate-Limiting
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

        // Enabled continuous dead reckoning updates for all targets
        float speed_mps = global_cache[i].speed_knots * 0.514444f;
        float distance_moved_meters = speed_mps * dt;
        float heading_rad = global_cache[i].heading_deg * M_PI / 180.0f;

        double delta_lat = (distance_moved_meters * cos(heading_rad)) / 111320.0;
        double lat_rad = global_cache[i].lat * M_PI / 180.0;
        double delta_lon = (distance_moved_meters * sin(heading_rad)) / (111320.0 * cos(lat_rad));

        global_cache[i].lat += delta_lat;
        global_cache[i].lon += delta_lon;
        global_cache[i].alt_ft += (global_cache[i].climb_rate_fps * dt);

        // Compute localized base transmission interval
        unsigned long target_interval = (global_cache[i].emitter_type == 15) ? 0 : 1000;
        if (adsb_rate_veh > 0.0f) {
            unsigned long vehicle_max_interval = (unsigned long)(1000.0f / adsb_rate_veh);
            if (vehicle_max_interval > target_interval) {
                target_interval = vehicle_max_interval;
            }
        }

        // Evaluate Per-Vehicle Update Cadence
        if (now - global_cache[i].last_mavlink_sent_ms >= target_interval) {
            
            // Evaluate Global Traffic Limit
            if (adsb_rate_tot > 0.0f && adsb_sent_this_sec >= (uint32_t)adsb_rate_tot) {
                continue; // Skip transmitting this loop cycle to prevent bus saturation
            }

            send_mavlink_adsb(
                global_cache[i].id, global_cache[i].lat, global_cache[i].lon, 
                (int32_t)global_cache[i].alt_ft, global_cache[i].heading_deg, String(global_cache[i].callsign), global_cache[i].emitter_type
            );
            
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
bool displayUpdatePending = false;

void handleGpsFixAcquisition(const char* source);
void send_mavlink_param(uint8_t index);
void handleWiFiConnection();

GpsPosition getGpsPosition() {
    xSemaphoreTake(xGpsMutex, portMAX_DELAY);
    GpsPosition pos = {current_lat, current_lon, current_alt_meters, gps_ready};
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

    constexpr char kHost[] = "stream.aisstream.io";
    constexpr uint16_t kPort = 443;
    constexpr char kPath[] = "/v0/stream";
    
    WebSocketsClient s_ws;
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

    void setStatus(const char* text) { strncpy(s_status_text, text, 47); s_dirty = true; }
    bool hasApiKey() { return s_api_key[0] != '\0'; }
    bool isConnected() { return s_connected; }

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

    void onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
        if (type == WStype_CONNECTED) {
            s_connected = true;
            s_subscription_pending = true;
            setStatus("AIS CONNECTED");
            Serial.println("[AIS Engine] Connected to WebSocket Endpoint successfully.");
        } else if (type == WStype_DISCONNECTED) {
            s_connected = false;
            setStatus("AIS DISCONNECTED");
            Serial.println("[AIS Engine] Lost Connection to WebSocket Stream Server.");
        } else if (type == WStype_TEXT) {
            handleTextMessage((char*)payload);
        }
    }

    String buildSubscriptionPayload() {
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

        String payload = "{\"APIKey\":\"";
        payload += s_api_key;
        payload += "\",\"BoundingBoxes\":[[[";
        payload += String(lat_min, 6);
        payload += ",";
        payload += String(lon_min, 6);
        payload += "],[";
        payload += String(lat_max, 6);
        payload += ",";
        payload += String(lon_max, 6);
        payload += "]]]}";
        return payload;
    }

    void init(const char* key) { strncpy(s_api_key, key, 64); }

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
                s_ws.beginSSL(kHost, kPort, kPath);
                s_ws.onEvent(onWebSocketEvent);
                s_started = true;
            }
            s_ws.loop();
            if (s_subscription_pending && s_connected) {
                String payload = buildSubscriptionPayload();
                s_ws.sendTXT(payload);
                s_subscription_pending = false;
                Serial.println("[AIS Engine] Streaming Geofence Area Payload Dispatched.");
            }
        } else if (WiFi.status() != WL_CONNECTED && s_started) {
            s_started = false; 
            s_connected = false;
        }
    }

    size_t vesselCount() { return s_vessel_count; }
    bool consumeDirty() { bool d = s_dirty; s_dirty = false; return d; }
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

    String requestURL = "https://api.n2yo.com/rest/v1/satellite/above/" + 
                        String(gps.lat, 5) + "/" + String(gps.lon, 5) + "/" + 
                        String(gps.alt, 2) + "/" + String((int)sat_rad_deg) + "/" + 
                        String((int)sat_cat) + "/&apiKey=" + N2YO_API_KEY;

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
                        String satelliteName = satelliteList[i]["satname"].as<String>();
                        satelliteName.trim();
                        long noradCatalogId  = satelliteList[i]["satid"];
                        double trackingLat    = satelliteList[i]["satlat"].as<double>();
                        double trackingLng    = satelliteList[i]["satlng"].as<double>();
                        float trackingAltKm  = satelliteList[i]["satalt"] | 420.0f; 
                        int32_t alt_ft       = (int32_t)(trackingAltKm * 3280.84f);  
                        
                        updateTargetCache((uint32_t)noradCatalogId, trackingLat, trackingLng, 
                                          (float)alt_ft, -1.0f, 14500.0f, 0.0f, 
                                          satelliteName.c_str(), 15);
                        
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
    
    String apiUrl = "https://opendata.adsb.fi/api/v3/lat/" + String(gps.lat, 6) + 
                    "/lon/" + String(gps.lon, 6) + "/dist/" + String((int)adsb_radius);
                    
    if (http.begin(secureClient, apiUrl)) {
        http.useHTTP10(true); 
        http.addHeader("Accept-Encoding", "identity"); 
        http.addHeader("User-Agent", "ESP32-MAVLink-GCS-Tracker/1.0");

        int httpResponseCode = http.GET();
        if (httpResponseCode == HTTP_CODE_OK) {
            JsonDocument filter;
            filter["ac"][0]["hex"]       = true;
            filter["ac"][0]["flight"]    = true;
            filter["ac"][0]["lat"]       = true;
            filter["ac"][0]["lon"]       = true;
            filter["ac"][0]["alt_baro"]  = true;
            filter["ac"][0]["gs"]        = true;
            filter["ac"][0]["track"]     = true;
            filter["ac"][0]["baro_rate"] = true;
            filter["ac"][0]["squawk"]    = true;

            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
            
            if (!error) {
                JsonArray aircraftList = doc["ac"].as<JsonArray>();
                int trackedCount = 0;
                for (JsonObject ac : aircraftList) {
                    String flight = ac["flight"].as<String>();
                    flight.trim();
                    char callsignBuf[9] = {0};
                    strncpy(callsignBuf, flight.c_str(), 8);

                    const char* hexStr = ac["hex"].as<const char*>();
                    if (!hexStr) continue;
                    uint32_t icaoAddress = strtoul(hexStr, NULL, 16);

                    float gs_knots = ac["gs"].is<float>() ? ac["gs"].as<float>() : 0.0f;
                    float baro_rate_fps = (ac["baro_rate"].as<float>() * 0.0166667f); 

                    float track_deg = -1.0f; 
                    if (ac.containsKey("track") && !ac["track"].isNull()) {
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

// ========================================================================
// 8. WIFILINK MANAGEMENT ENGINE
// ========================================================================
void handleWiFiConnection() {
    if (WiFi.status() == WL_CONNECTED) return;

    size_t profile_idx = (size_t)std::max(0.0f, std::min(wifi_prof_id, (float)(NUM_WIFI_PROFILES - 1)));
    const char* target_ssid = wifi_profiles[profile_idx].ssid;
    const char* target_pass = wifi_profiles[profile_idx].password;

    Serial.printf("\n[WiFi Engine] Connecting via Profile [%d]: %s\n", profile_idx, target_ssid);
    WiFi.disconnect();
    WiFi.begin(target_ssid, target_pass);

    unsigned long start_ms = millis();
    uint32_t attempts = 0;
    unsigned long timeout_limit_ms = (unsigned long)(wifi_timeout * 1000.0f);

    while (WiFi.status() != WL_CONNECTED && attempts < (uint32_t)wifi_retry_max) {
        if (millis() - start_ms >= timeout_limit_ms) {
            attempts++;
            WiFi.disconnect();
            WiFi.begin(target_ssid, target_pass);
            start_ms = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi Engine] Connected. IP: ");
        Serial.println(WiFi.localIP());
        led_blink_interval = 100;
    } else {
        Serial.println("[WiFi Engine] Handshake timeout.");
        led_blink_interval = 500;
    }
}

void connectToOGN() {
    GpsPosition gps = getGpsPosition();
    if (!gps.ready) return;
    Serial.println("[OGN Socket] Attempting raw APRS TCP Server alignment...");
    if (ognClient.connect(ogn_host, (uint16_t)ogn_port_val)) {
        String loginStr = "user ardupilot pass -1 vers ESP32-MAVLink 1.0 filter r/";
        loginStr += String(gps.lat, 4) + "/" + String(gps.lon, 4) + "/100\r\n";
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
            String line = ognClient.readStringUntil('\n');
            g_count_ogn_packets++;
            g_last_ogn_rx_ms = millis();
        }
    }
}

uint32_t callsignToICAO(String callsign) {
    uint32_t hash = 0;
    for (unsigned int i = 0; i < callsign.length(); i++) {
        hash = (hash * 31) + callsign[i];
    }
    return hash;
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

void send_mavlink_adsb(uint32_t icao, double lat, double lon, int32_t alt_ft, float heading_deg, String callsignStr, uint8_t emitter_type) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    int32_t lat_e7 = (int32_t)(lat * 10000000);
    int32_t lon_e7 = (int32_t)(lon * 10000000);
    int32_t alt_mm = (int32_t)(alt_ft * 304.8);

    uint16_t heading_cdeg = (uint16_t)(heading_deg * 100.0f);
    if (heading_cdeg > 35999) heading_cdeg = 0;

    char callsign[9] = {0};
    strncpy(callsign, callsignStr.c_str(), 8);

    mavlink_msg_adsb_vehicle_pack(
        MAV_SYS_ID, MAV_COMP_ID_ADSB_OUT, &msg, 
        icao, lat_e7, lon_e7, ADSB_ALTITUDE_TYPE_GEOMETRIC, alt_mm, 
        heading_cdeg, 0, 0, callsign, emitter_type, 1, 
        ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_CALLSIGN | ADSB_FLAGS_VALID_HEADING, 0
    );
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
            MAV_SYS_ID, MAV_COMP_ID_AIS_OUT, &msg,
            v.mmsi, (int32_t)(v.lat * 1e7), (int32_t)(v.lon * 1e7),
            0, (uint16_t)(v.heading_deg * 100.0), (uint16_t)(v.sog_knots * 100.0),
            0, 0, 0, 0, 0, 0, 0, "UNKNOWN", v.name, 0, 0
        );
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
    char param_id[16] = {0};
    strncpy(param_id, local_params[index].id, 16);
    
    mavlink_msg_param_value_pack(
        MAV_SYS_ID, MAV_COMP_ID_ADSB_OUT, &msg, param_id, 
        *(local_params[index].value_ptr), local_params[index].type, ONBOARD_PARAM_COUNT, index
    );
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
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
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
                case MAVLINK_MSG_ID_GPS_RAW_INT: {
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
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    mavlink_param_request_list_t req_list;
                    mavlink_msg_param_request_list_decode(&msg, &req_list);
                    
                    // CRITICAL: Only respond if the request is explicitly targeted to this component
                    if (req_list.target_system == MAV_SYS_ID && req_list.target_component == MAV_COMP_ID_ADSB_OUT) {
                        for (uint8_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
                            send_mavlink_param(i);
                            vTaskDelay(pdMS_TO_TICKS(5));
                        }
                    }
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                    mavlink_param_request_read_t read_req;
                    mavlink_msg_param_request_read_decode(&msg, &read_req);
                    
                    // CRITICAL: Ignore requests directed at the ArduPilot Autopilot core
                    if (read_req.target_system == MAV_SYS_ID && read_req.target_component == MAV_COMP_ID_ADSB_OUT) {
                        if (read_req.param_index >= 0 && read_req.param_index < ONBOARD_PARAM_COUNT) {
                            send_mavlink_param(read_req.param_index);
                        } else {
                            char nameBuf[17] = {0};
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
                case MAVLINK_MSG_ID_PARAM_SET: {
                    mavlink_param_set_t set_param;
                    mavlink_msg_param_set_decode(&msg, &set_param);
                    
                    // CRITICAL: Protect ArduPilot configuration space from mismatched data writes
                    if (set_param.target_system == MAV_SYS_ID && set_param.target_component == MAV_COMP_ID_ADSB_OUT) {
                        char nameBuf[17] = {0};
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
            send_mavlink_heartbeat(MAV_COMP_ID_ADSB_OUT, MAV_TYPE_ANTENNA_TRACKER);
            last_heartbeat_adsb = now;
        }
        if (now - last_heartbeat_ais >= 1000) {
            send_mavlink_heartbeat(MAV_COMP_ID_AIS_OUT, MAV_TYPE_ANTENNA_TRACKER);
            last_heartbeat_ais = now;
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
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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

    xGpsMutex    = xSemaphoreCreateMutex();
    xCacheMutex  = xSemaphoreCreateMutex();
    xSerialMutex = xSemaphoreCreateMutex();
    xAisMutex    = xSemaphoreCreateMutex();

    MAVLINK_SERIAL.begin(500000, SERIAL_8N1, RX2_PIN, TX2_PIN);

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
        Serial.printf(" Terrestrial ADS-B (ADSB.fi) : %u tracked\n", g_count_adsb_packets);
        Serial.printf(" Space-Based ADS-B (Satellites): %u tracked\n", g_count_space_packets);
        Serial.printf(" Glider Network (OGN/APRS)     : %u tracked\n", g_count_ogn_packets);
        Serial.printf(" Marine Surface (AIS Stream)   : %u tracked\n", g_count_ais_packets);
        Serial.println("=================================================================");
        last_diagnostic_print = now;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}

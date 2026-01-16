// motionDetector.cpp
// Integrated version with Banked Kalman Filter solution
#include "motionDetector.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

// Configuration macros (keep as original if defined in header)
#ifndef ENABLE_ALARM_THRESHOLD
#define ENABLE_ALARM_THRESHOLD 0
#endif

#ifndef MINIMUM_RSSI
#define MINIMUM_RSSI -100
#endif

#ifndef ABSOLUTE_RSSI_LIMIT
#define ABSOLUTE_RSSI_LIMIT -120
#endif

// ---------------- Kalman Filter (simple 1D) ----------------
struct KalmanFilter {
    float q; // process noise covariance
    float r; // measurement noise covariance
    float x; // value
    float p; // estimation error covariance
    float k; // kalman gain

    KalmanFilter(float processNoise = 1.0f, float measurementNoise = 4.0f, float initialValue = 0.0f)
        : q(processNoise), r(measurementNoise), x(initialValue), p(1.0f), k(0.0f) {}

    // Update with new measurement, return filtered estimate
    float update(float measurement) {
        // Prediction update
        p = p + q;
        // Measurement update
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1.0f - k) * p;
        return x;
    }

    void reset(float initialValue = 0.0f) {
        x = initialValue;
        p = 1.0f;
        k = 0.0f;
    }
};

// ---------------- Advanced Kalman for ACTIVE MODE (constant-velocity RSSI model) ----------------
struct Kalman2D {
    // State
    float x0; // RSSI
    float x1; // RSSI rate of change

    // Covariance matrix P
    float p00, p01, p10, p11;

    // Noise parameters
    float q_rssi;   // process noise for RSSI
    float q_rate;   // process noise for rate
    float r_meas;   // measurement noise

    Kalman2D(float q0 = 1.0f, float q1 = 0.1f, float r = 6.0f, float initRSSI = -60.0f)
        : x0(initRSSI), x1(0.0f),
          p00(10.0f), p01(0.0f), p10(0.0f), p11(1.0f),
          q_rssi(q0), q_rate(q1), r_meas(r) {}

    void reset(float initRSSI = -60.0f) {
        x0 = initRSSI;
        x1 = 0.0f;
        p00 = 10.0f; p01 = 0.0f; p10 = 0.0f; p11 = 1.0f;
    }

    // dt = time step in seconds
    float update(float z, float dt) {
        // ----- Prediction -----
        // x = F x, where F = [[1, dt],[0,1]]
        x0 = x0 + dt * x1;
        // x1 unchanged

        // P = F P F^T + Q
        float p00_new = p00 + dt * (p10 + p01) + dt * dt * p11 + q_rssi;
        float p01_new = p01 + dt * p11;
        float p10_new = p10 + dt * p11;
        float p11_new = p11 + q_rate;

        p00 = p00_new; p01 = p01_new;
        p10 = p10_new; p11 = p11_new;

        // ----- Update -----
        // Measurement model: z = [1 0] x + v
        float y = z - x0;           // innovation
        float s = p00 + r_meas;     // innovation covariance
        float k0 = p00 / s;         // Kalman gain
        float k1 = p10 / s;

        // State update
        x0 = x0 + k0 * y;
        x1 = x1 + k1 * y;

        // Covariance update
        float p00_u = (1.0f - k0) * p00;
        float p01_u = (1.0f - k0) * p01;
        float p10_u = p10 - k1 * p00;
        float p11_u = p11 - k1 * p01;

        p00 = p00_u; p01 = p01_u;
        p10 = p10_u; p11 = p11_u;

        return x0;
    }
};

// ---------------- Per-BSSID Kalman Banks + Channel-aware Filters ----------------
#define MAX_KALMAN_BANKS 8
#define MAX_CHANNELS 14
#define BANK_UNUSED 0
#define BANK_USED 1

struct BssidBank {
    uint8_t mac[6];
    Kalman2D kf;        // per-BSSID 2D Kalman (RSSI, rate)
    int channel;        // channel observed on
    int last_rssi;      // last raw RSSI observed
    unsigned long last_seen_ms; // last update time
    uint32_t age_counter; // used for simple LRU (larger = older)
    uint8_t state;      // BANK_USED/BANK_UNUSED
};

// ----- Globals - Radio & Kalman -----
static KalmanFilter rssiKalman(1.0f, 4.0f, -60.0f);
static bool useKalman = true;

static Kalman2D activeKalman(1.5f, 0.2f, 8.0f, -60.0f);
static bool useActiveKalman = true;
static unsigned long lastActiveUpdateMs = 0;

static BssidBank bssidBanks[MAX_KALMAN_BANKS];
static uint32_t global_age_counter = 1;
static Kalman2D channelKFs[MAX_CHANNELS + 1];
static bool channelKFInit[MAX_CHANNELS + 1] = {false};

#ifdef ESP32
static volatile int lastPromiscuousRSSI = 0;
static volatile bool promRssiAvailable = false;
static volatile bool promiscuousEnabled = false;
#endif

// ----- Globals - Motion Detector Core -----
int enableThreshold = ENABLE_ALARM_THRESHOLD;
bool enableAutoRegressive = false;

#define MAX_SAMPLEBUFFERSIZE 256
static int *sampleBuffer = NULL;
static size_t sampleBufferSize = MAX_SAMPLEBUFFERSIZE;
static size_t sampleBufferIndex = 0;
static bool sampleBufferValid = false;

#define MAX_AVERAGEBUFFERSIZE 64
static size_t mobileAverageFilterSize = MAX_AVERAGEBUFFERSIZE;
static size_t mobileAverageBufferSize = MAX_AVERAGEBUFFERSIZE;
static int mobileAverage = 0;
static int *mobileAverageBuffer = NULL;
static size_t mobileAverageBufferIndex = 0;
static bool mobileAverageBufferValid = false;

int variance = RADAR_BOOTING;
int variancePrev = 0;
int varianceSample = 0;
int varianceAR = 0;
int varianceIntegral = 0;

#define MAX_VARIANCE 65535
int varianceThreshold = 3;
size_t varianceIntegratorLimitMax = MAX_SAMPLEBUFFERSIZE;
size_t varianceIntegratorLimit = 3;
static size_t varianceBufferSize = MAX_SAMPLEBUFFERSIZE;
static int *varianceBuffer = NULL;
int detectionLevel = 0;
static size_t varianceBufferIndex = 0;
static bool varianceBufferValid = false;

int enableCSVout = 0;
int minimumRSSI = MINIMUM_RSSI;

// ----- ESP32 Specific Globals -----
#define SCANMODE_STA 0
#define SCANMODE_SOFTAP 1
#define SCANMODE_WIFIPROBE 2

static int scanMode = SCANMODE_STA;
uint8_t strongestClientBSSID[6] = {0};
int strongestClientRSSI = -100;
int strongestClientfound = 0;
int modeRes = 0;
uint8_t BSSIDinUse[6] = {0};

uint8_t strongestBSSID[6] = {0};
int strongestRSSI = -100;
int strongestChannel = 0;
int strongestAPfound = 0;

// ----- Helper: safe allocation & zero init -----
static bool safe_alloc_int_array(int **ptr, size_t count, bool use_psram = false) {
    if (ptr == NULL) return false;
    if (*ptr != NULL) return true;
    if (count == 0) return false;
#ifdef CONFIG_SPIRAM_SUPPORT
    if (use_psram) {
        *ptr = (int*) ps_malloc(sizeof(int) * count);
    } else {
        *ptr = (int*) malloc(sizeof(int) * count);
    }
#else
    *ptr = (int*) malloc(sizeof(int) * count);
#endif
    if (*ptr == NULL) return false;
    memset(*ptr, 0, sizeof(int) * count);
    return true;
}

// ---------------- Radio Layer Helpers ----------------

// Initialize banks
void motionDetector_radio_banks_init() {
    for (int i = 0; i < MAX_KALMAN_BANKS; ++i) {
        bssidBanks[i].state = BANK_UNUSED;
        bssidBanks[i].last_rssi = -200;
        bssidBanks[i].channel = 0;
        bssidBanks[i].last_seen_ms = 0;
        bssidBanks[i].age_counter = 0;
    }
    for (int ch = 0; ch <= MAX_CHANNELS; ++ch) {
        channelKFInit[ch] = false;
    }
}

static int find_bank_by_mac(const uint8_t *mac) {
    for (int i = 0; i < MAX_KALMAN_BANKS; ++i) {
        if (bssidBanks[i].state == BANK_USED) {
            if (memcmp(bssidBanks[i].mac, mac, 6) == 0) return i;
        }
    }
    return -1;
}

static int allocate_bank_for_mac(const uint8_t *mac, int channel, float q0 = 1.5f, float q1 = 0.2f, float r = 8.0f, float initRSSI = -60.0f) {
    int idx = find_bank_by_mac(mac);
    if (idx >= 0) return idx;

    for (int i = 0; i < MAX_KALMAN_BANKS; ++i) {
        if (bssidBanks[i].state == BANK_UNUSED) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        uint32_t oldest = 0;
        int oldest_i = -1;
        for (int i = 0; i < MAX_KALMAN_BANKS; ++i) {
            if (bssidBanks[i].state == BANK_USED) {
                if (bssidBanks[i].age_counter > oldest) {
                    oldest = bssidBanks[i].age_counter;
                    oldest_i = i;
                }
            } else {
                oldest_i = i; break;
            }
        }
        if (oldest_i < 0) return -1;
        idx = oldest_i;
    }

    memcpy(bssidBanks[idx].mac, mac, 6);
    bssidBanks[idx].kf = Kalman2D(q0, q1, r, initRSSI);
    bssidBanks[idx].channel = channel;
    bssidBanks[idx].last_rssi = (int)initRSSI;
    bssidBanks[idx].last_seen_ms = millis();
    bssidBanks[idx].age_counter = 0;
    bssidBanks[idx].state = BANK_USED;
    return idx;
}

static void update_channel_kf(int channel, float z, float dt) {
    if (channel < 0 || channel > MAX_CHANNELS) return;
    if (!channelKFInit[channel]) {
        channelKFs[channel] = Kalman2D(1.5f, 0.2f, 8.0f, z);
        channelKFInit[channel] = true;
        return;
    }
    channelKFs[channel].update(z, dt);
}

#ifdef ESP32
static void wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (buf == nullptr) return;
    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*) buf;
    int8_t rssi = pkt->rx_ctrl.rssi;
    if ((rssi < -120) || (rssi > 0)) return;
    lastPromiscuousRSSI = (int) rssi;
    promRssiAvailable = true;
}

static void wifi_promiscuous_enhanced_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (buf == nullptr) return;
    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*) buf;
    int8_t rssi = pkt->rx_ctrl.rssi;
    if ((rssi < -120) || (rssi > 0)) return;

    const uint8_t *payload = pkt->payload;
    if (payload == NULL) return;
    if (pkt->rx_ctrl.sig_len < 24) return;
    const uint8_t *mac = payload + 10;
    int ch = pkt->rx_ctrl.channel;

    int bank = allocate_bank_for_mac(mac, ch);
    if (bank < 0) return;

    unsigned long now = millis();
    float dt = 0.25f;
    if (bssidBanks[bank].last_seen_ms != 0) dt = (now - bssidBanks[bank].last_seen_ms) * 0.001f;
    if (dt <= 0) dt = 0.05f;
    bssidBanks[bank].last_seen_ms = now;
    bssidBanks[bank].last_rssi = rssi;
    bssidBanks[bank].channel = ch;
    bssidBanks[bank].age_counter = ++global_age_counter;

    float filtered = bssidBanks[bank].kf.update((float) rssi, dt);
    update_channel_kf(ch, filtered, dt);
}

void motionDetector_radio_enable_promiscuous(bool enable) {
    if (enable && !promiscuousEnabled) {
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_cb);
        promiscuousEnabled = true;
    } else if (!enable && promiscuousEnabled) {
        esp_wifi_set_promiscuous_rx_cb(NULL);
        esp_wifi_set_promiscuous(false);
        promiscuousEnabled = false;
        promRssiAvailable = false;
        lastPromiscuousRSSI = 0;
    }
}

void motionDetector_radio_enable_bssid_banks(bool enable) {
    if (enable) {
        motionDetector_radio_banks_init();
        esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_enhanced_cb);
        esp_wifi_set_promiscuous(true);
        promiscuousEnabled = true;
    } else {
        esp_wifi_set_promiscuous_rx_cb(NULL);
        esp_wifi_set_promiscuous(false);
        promiscuousEnabled = false;
    }
}
#else
void motionDetector_radio_enable_promiscuous(bool enable) { (void)enable; }
void motionDetector_radio_enable_bssid_banks(bool enable) { (void)enable; }
#endif

int motionDetector_radio_read_raw_rssi() {
    int rssi = (int) WiFi.RSSI();
    if (rssi != 0) return rssi;

#ifdef ESP32
    if (promRssiAvailable) {
        return lastPromiscuousRSSI;
    }

    int scanRes = WiFi.scanNetworks(false, false, false, 200, 0);
    if (scanRes > 0) {
        int strongest = -200;
        for (int i = 0; i < scanRes; ++i) {
            int r = WiFi.RSSI(i);
            if ((r > strongest) && (r < 0) && (r > -150)) strongest = r;
        }
        WiFi.scanDelete();
        if (strongest > -150) return strongest;
    }
#endif
    return 0;
}

void motionDetector_radio_use_kalman(bool enable, float processNoise, float measurementNoise, float initialValue) {
    useKalman = enable;
    if (enable) {
        if (processNoise > 0.0f) rssiKalman.q = processNoise;
        if (measurementNoise > 0.0f) rssiKalman.r = measurementNoise;
        rssiKalman.reset(initialValue);
    }
}

int motionDetector_radio_read_filtered_rssi() {
    int raw = motionDetector_radio_read_raw_rssi();
    if (raw == 0) return 0;
    if (!useKalman) return raw;
    float filtered = rssiKalman.update((float) raw);
    return (int) roundf(filtered);
}

int motionDetector_radio_read_active_filtered_rssi() {
    int raw = motionDetector_radio_read_raw_rssi();
    if (raw == 0) return 0;
    if (!useActiveKalman) return raw;

    unsigned long now = millis();
    float dt = 0.3f;
    if (lastActiveUpdateMs != 0) {
        dt = (now - lastActiveUpdateMs) * 0.001f;
        if (dt < 0.05f) dt = 0.05f;
        if (dt > 2.0f) dt = 2.0f;
    }
    lastActiveUpdateMs = now;

    float filtered = activeKalman.update((float)raw, dt);
    return (int) roundf(filtered);
}

void motionDetector_radio_use_active_kalman(bool enable, float q_rssi, float q_rate, float r_meas, float initRSSI) {
    useActiveKalman = enable;
    if (enable) {
        if (q_rssi > 0) activeKalman.q_rssi = q_rssi;
        if (q_rate > 0) activeKalman.q_rate = q_rate;
        if (r_meas > 0) activeKalman.r_meas = r_meas;
        activeKalman.reset(initRSSI);
        lastActiveUpdateMs = 0;
    }
}

float motionDetector_radio_compute_fused_motion_energy(int topN) {
    struct Candidate { int idx; int rssi; float rate; } cands[MAX_KALMAN_BANKS];
    int n = 0;
    unsigned long now = millis();
    for (int i = 0; i < MAX_KALMAN_BANKS; ++i) {
        if (bssidBanks[i].state == BANK_USED) {
            if ((now - bssidBanks[i].last_seen_ms) > 30000UL) {
                bssidBanks[i].state = BANK_UNUSED;
                continue;
            }
            cands[n].idx = i;
            cands[n].rssi = bssidBanks[i].last_rssi;
            cands[n].rate = bssidBanks[i].kf.x1;
            ++n;
        }
    }
    if (n == 0) return 0.0f;

    for (int i = 0; i < n; ++i) {
        int best = i;
        for (int j = i+1; j < n; ++j) if (cands[j].rssi > cands[best].rssi) best = j;
        if (best != i) { Candidate tmp = cands[i]; cands[i] = cands[best]; cands[best] = tmp; }
    }

    if (topN <= 0 || topN > n) topN = n;

    float energy = 0.0f;
    float weightSum = 0.0f;
    for (int i = 0; i < topN; ++i) {
        float r = (float) cands[i].rssi;
        float w = 1.0f / (1.0f + expf(( -r - 40.0f) * 0.1f ));
        float contribution = w * (cands[i].rate * cands[i].rate);
        energy += contribution;
        weightSum += w;
    }
    if (weightSum > 0.0f) energy = energy / weightSum;
    return energy;
}

float motionDetector_radio_channel_motion_energy(int channel, int topN) {
    if (channel < 0 || channel > MAX_CHANNELS) channel = 0;
    if (channel == 0) {
        return motionDetector_radio_compute_fused_motion_energy(topN);
    }
    if (!channelKFInit[channel]) return 0.0f;
    return fabsf(channelKFs[channel].x1);
}

// ---------------- Core Processing ----------------

int motionDetector_init() {
    if (!safe_alloc_int_array(&sampleBuffer, sampleBufferSize, false)) return 0;
    if (!safe_alloc_int_array(&mobileAverageBuffer, mobileAverageBufferSize, false)) return 0;
    if (!safe_alloc_int_array(&varianceBuffer, varianceBufferSize, false)) return 0;

    sampleBufferIndex = 0;
    sampleBufferValid = false;
    mobileAverageBufferIndex = 0;
    mobileAverageBufferValid = false;
    varianceBufferIndex = 0;
    varianceBufferValid = false;

    variance = RADAR_BOOTING;
    varianceAR = 0;
    varianceIntegral = 0;
    varianceSample = 0;

    return 1;
}

int motionDetector_init_PSRAM() {
#ifdef CONFIG_SPIRAM_SUPPORT
    if (!safe_alloc_int_array(&sampleBuffer, sampleBufferSize, true)) return 0;
    if (!safe_alloc_int_array(&mobileAverageBuffer, mobileAverageBufferSize, true)) return 0;
    if (!safe_alloc_int_array(&varianceBuffer, varianceBufferSize, true)) return 0;
#else
    return motionDetector_init();
#endif
    sampleBufferIndex = 0;
    sampleBufferValid = false;
    mobileAverageBufferIndex = 0;
    mobileAverageBufferValid = false;
    varianceBufferIndex = 0;
    varianceBufferValid = false;
    variance = RADAR_BOOTING;
    return 1;
}

int motionDetector_deinit() {
    if (sampleBuffer != NULL) { free(sampleBuffer); sampleBuffer = NULL; }
    if (mobileAverageBuffer != NULL) { free(mobileAverageBuffer); mobileAverageBuffer = NULL; }
    if (varianceBuffer != NULL) { free(varianceBuffer); varianceBuffer = NULL; }

    sampleBufferSize = MAX_SAMPLEBUFFERSIZE;
    mobileAverageFilterSize = MAX_AVERAGEBUFFERSIZE;
    mobileAverageBufferSize = MAX_AVERAGEBUFFERSIZE;
    varianceBufferSize = MAX_SAMPLEBUFFERSIZE;

    sampleBufferIndex = varianceBufferIndex = mobileAverageBufferIndex = 0;
    sampleBufferValid = varianceBufferValid = mobileAverageBufferValid = false;

    variance = RADAR_BOOTING;
    varianceAR = 0;
    varianceIntegral = 0;
    varianceSample = 0;

    return 1;
}

int motionDetector_config(int sampleBufSize, int mobileAvgSize, int varThreshold, int varIntegratorLimit, bool enableAR) {
    if (sampleBufSize <= 0) sampleBufSize = 1;
    if ((size_t)sampleBufSize > MAX_SAMPLEBUFFERSIZE) sampleBufSize = MAX_SAMPLEBUFFERSIZE;
    sampleBufferSize = (size_t)sampleBufSize;
    varianceBufferSize = sampleBufferSize;

    if (mobileAvgSize <= 0) mobileAvgSize = 1;
    if ((size_t)mobileAvgSize > MAX_AVERAGEBUFFERSIZE) mobileAvgSize = MAX_AVERAGEBUFFERSIZE;
    mobileAverageFilterSize = (size_t)mobileAvgSize;

    if (varThreshold < 0) varThreshold = 0;
    if (varThreshold > MAX_VARIANCE) varThreshold = MAX_VARIANCE;
    varianceThreshold = varThreshold;

    if (varIntegratorLimit <= 0) varIntegratorLimit = 1;
    if ((size_t)varIntegratorLimit > varianceIntegratorLimitMax) varIntegratorLimit = (int)varianceIntegratorLimitMax;
    varianceIntegratorLimit = (size_t)varIntegratorLimit;

    enableAutoRegressive = enableAR;

    if (sampleBuffer || mobileAverageBuffer || varianceBuffer) {
        motionDetector_deinit();
        motionDetector_init();
    }
    return 1;
}

int motionDetector_process(int sample) {
    if ((sampleBuffer == NULL) || (mobileAverageBuffer == NULL) || (varianceBuffer == NULL)) {
        return RADAR_UNINITIALIZED;
    }

    if (sample < minimumRSSI) {
        return RADAR_RSSI_TOO_LOW;
    }

    sampleBuffer[sampleBufferIndex] = sample;
    sampleBufferIndex = (sampleBufferIndex + 1) % sampleBufferSize;
    if (!sampleBufferValid && sampleBufferIndex == 0) sampleBufferValid = true;

    if (!sampleBufferValid && (sampleBufferIndex < mobileAverageFilterSize)) {
        variance = RADAR_BOOTING;
        return variance;
    }

    long long sum = 0;
    for (size_t i = 0; i < mobileAverageFilterSize; ++i) {
        size_t idx = (sampleBufferIndex + sampleBufferSize - 1 - i) % sampleBufferSize;
        sum += sampleBuffer[idx];
    }
    mobileAverage = (int)(sum / (long long)mobileAverageFilterSize);

    mobileAverageBuffer[mobileAverageBufferIndex] = mobileAverage;
    mobileAverageBufferIndex = (mobileAverageBufferIndex + 1) % mobileAverageBufferSize;
    if (!mobileAverageBufferValid && mobileAverageBufferIndex == 0) mobileAverageBufferValid = true;

    long long diff = (long long)sample - (long long)mobileAverage;
    long long vs = diff * diff;
    if (vs > INT32_MAX) vs = INT32_MAX;
    varianceSample = (int)vs;

    varianceBuffer[varianceBufferIndex] = varianceSample;

    long long vintegral = 0;
    for (size_t i = 0; i < varianceIntegratorLimit; ++i) {
        size_t idx = (varianceBufferIndex + varianceBufferSize - i) % varianceBufferSize;
        vintegral += varianceBuffer[idx];
    }
    varianceBufferIndex = (varianceBufferIndex + 1) % varianceBufferSize;
    if (!varianceBufferValid && varianceBufferIndex == 0) varianceBufferValid = true;

    if (vintegral > INT32_MAX) vintegral = INT32_MAX;
    varianceIntegral = (int)vintegral;

    varianceAR = (varianceIntegral + varianceAR) / 2;

    if (enableAutoRegressive) {
        variance = varianceAR;
    } else {
        variance = varianceIntegral;
    }

    if (enableThreshold > 0) {
        if (variance >= varianceThreshold) {
            detectionLevel = variance;
            return detectionLevel;
        } else if (variance >= 0) {
            detectionLevel = 0;
            return detectionLevel;
        }
    }
    return variance;
}

// ---------------- Wrapper Functions ----------------

int motionDetector() {
    int RSSIlevel = motionDetector_radio_read_filtered_rssi();
    if (RSSIlevel == 0) return RADAR_INOPERABLE;
    return motionDetector_process(RSSIlevel);
}

int bistatic_get_rssi_SoftAP_strongestClient() {
    int rssi = 0;
    wifi_sta_list_t stationList;
    esp_err_t scanRes = esp_wifi_ap_get_sta_list(&stationList);
    if (scanRes != ESP_OK) return 0;

    if (!strongestClientfound) {
        strongestClientRSSI = -100;
        for (int i = 0; i < (int)stationList.num; ++i) {
            wifi_sta_info_t station = stationList.sta[i];
            uint8_t *currentBSSID = station.mac;
            int8_t currentRSSI = station.rssi;
            if ((currentRSSI > -100) && (currentRSSI < 0)) {
                if (currentRSSI > strongestClientRSSI) {
                    strongestClientfound = 1;
                    strongestClientRSSI = currentRSSI;
                    memcpy(strongestClientBSSID, currentBSSID, 6);
                }
            }
        }
    }

    if (strongestClientfound) {
        int bssidScanOK = 0;
        for (int i = 0; i < (int)stationList.num; ++i) {
            wifi_sta_info_t station = stationList.sta[i];
            uint8_t *currentBSSID = station.mac;
            if (memcmp(currentBSSID, strongestClientBSSID, 6) == 0) {
                bssidScanOK = 1;
                rssi = station.rssi;
                break;
            }
        }
        if (!bssidScanOK) {
            strongestClientfound = 0;
            return 0;
        }
    } else {
        if (modeRes & WIFI_MODE_APSTA) {
            if (WiFi.status() == WL_CONNECTED) scanMode = SCANMODE_STA;
            else scanMode = SCANMODE_WIFIPROBE;
        }
        rssi = 0;
    }
    memcpy(BSSIDinUse, strongestClientBSSID, 6);
    return rssi;
}

int bistatic_get_rssi_ScanStrongestAP() {
    int rssi = 0;
    int scanRes = 0;
    if (!strongestAPfound) {
        scanRes = WiFi.scanNetworks(false, false, false, 300, 0);
    } else {
        scanRes = WiFi.scanNetworks(false, false, false, 200, strongestChannel);
    }

    if (!strongestAPfound) {
        strongestRSSI = -100;
        for (int i = 0; i < scanRes; ++i) {
            uint8_t *currentBSSID = WiFi.BSSID(i);
            int currentRSSI = WiFi.RSSI(i);
            int currentChannel = WiFi.channel(i);
            if ((currentRSSI > -100) && (currentRSSI < 0)) {
                if (currentRSSI > strongestRSSI) {
                    strongestAPfound = 1;
                    strongestRSSI = currentRSSI;
                    strongestChannel = currentChannel;
                    memcpy(strongestBSSID, currentBSSID, 6);
                }
            }
        }
    }

    if (strongestAPfound) {
        int bssidScanOK = 0;
        for (int i = 0; i < scanRes; ++i) {
            uint8_t *currentBSSID = WiFi.BSSID(i);
            if (memcmp(currentBSSID, strongestBSSID, 6) == 0) {
                bssidScanOK = 1;
                rssi = WiFi.RSSI(i);
                break;
            }
        }
        if (!bssidScanOK) {
            strongestAPfound = 0;
            strongestChannel = 0;
            WiFi.scanDelete();
            return 0;
        }
    } else {
        strongestChannel = 0;
        rssi = 0;
        if (modeRes & WIFI_MODE_APSTA) scanMode = SCANMODE_SOFTAP;
    }
    memcpy(BSSIDinUse, strongestBSSID, 6);
    WiFi.scanDelete();
    return rssi;
}

void serialPrintBSSID(const uint8_t *localBSSID) {
    if (localBSSID == NULL) { Serial.print("NULL"); return; }
    for (int i = 0; i < 6; ++i) {
        if (localBSSID[i] < 0x10) Serial.print('0');
        Serial.print(localBSSID[i], HEX);
        if (i < 5) Serial.print(':');
    }
}

int motionDetector_esp() {
    int RSSIlevel = 0;
    int res = 0;
    modeRes = (int) WiFi.getMode();
    if (modeRes & WIFI_MODE_NULL) return WIFI_MODEINVALID;

    if ((modeRes & WIFI_MODE_APSTA) || (modeRes & WIFI_MODE_STA)) {
        RSSIlevel = motionDetector_radio_read_filtered_rssi();
    }

    if (RSSIlevel == 0) {
        if ((modeRes & WIFI_MODE_APSTA) || (modeRes & WIFI_MODE_AP)) {
            if (scanMode == SCANMODE_SOFTAP) RSSIlevel = bistatic_get_rssi_SoftAP_strongestClient();
            else if (scanMode == SCANMODE_WIFIPROBE) RSSIlevel = bistatic_get_rssi_ScanStrongestAP();

            if ((RSSIlevel == 0) && (scanMode == SCANMODE_SOFTAP)) {
                scanMode = SCANMODE_WIFIPROBE;
                RSSIlevel = bistatic_get_rssi_ScanStrongestAP();
            }
            if ((RSSIlevel == 0) && (scanMode == SCANMODE_WIFIPROBE)) {
                scanMode = SCANMODE_SOFTAP;
                RSSIlevel = bistatic_get_rssi_SoftAP_strongestClient();
            }
            if (RSSIlevel == 0) scanMode = SCANMODE_SOFTAP;
        }
        if (modeRes & WIFI_MODE_STA) {
            scanMode = SCANMODE_WIFIPROBE;
            RSSIlevel = bistatic_get_rssi_ScanStrongestAP();
        }
        if (RSSIlevel == 0) {
            scanMode = SCANMODE_SOFTAP;
            return RADAR_INOPERABLE;
        }
    }

    res = motionDetector_process(RSSIlevel);
    if (enableCSVout) {
        Serial.println("VarianceLevel");
        serialPrintBSSID(BSSIDinUse);
        Serial.print("_");
        Serial.println(RSSIlevel);
        Serial.println(res);
    }
    return res;
}

// ---------------- Setters ----------------

int motionDetector_enable_serial_CSV_graph_data(int serialCSVen) {
    if (serialCSVen >= 0) enableCSVout = serialCSVen;
    return enableCSVout;
}

int motionDetector_set_minimum_RSSI(int rssiMin) {
    if (rssiMin > 0) rssiMin = 0;
    if (rssiMin < ABSOLUTE_RSSI_LIMIT) rssiMin = ABSOLUTE_RSSI_LIMIT;
    minimumRSSI = rssiMin;
    return minimumRSSI;
}

int motionDetector_enable_alarm(int thresholdEnable) {
    if (thresholdEnable < 0) thresholdEnable = 0;
    enableThreshold = thresholdEnable;
    return enableThreshold;
}

int motionDetector_set_alarm_threshold(int alarmThreshold) {
    if (alarmThreshold < 0) alarmThreshold = 0;
    varianceThreshold = alarmThreshold;
    return varianceThreshold;
}

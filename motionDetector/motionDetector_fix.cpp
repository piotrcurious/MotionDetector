// improved_motionDetector.cpp
#include "motionDetector.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Configuration macros (keep as original if defined in header)
#ifndef ENABLE_ALARM_THRESHOLD
#define ENABLE_ALARM_THRESHOLD 0
#endif

// ----- Globals - initialized safely -----
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

int bufferIndex = 0; // legacy - kept for compatibility (not used below)

int variance = RADAR_BOOTING; // must start as -1 per your comment (RADAR_BOOTING assumed -1)
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

// ----- Helper: safe allocation & zero init -----
static bool safe_alloc_int_array(int **ptr, size_t count, bool use_psram = false) {
    if (ptr == NULL) return false;
    if (*ptr != NULL) return true; // already allocated
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
    // zero initialize
    memset(*ptr, 0, sizeof(int) * count);
    return true;
}

// ----- Initialization / Deinit -----
int motionDetector_init() {
    // allocate in internal RAM
    if (!safe_alloc_int_array(&sampleBuffer, sampleBufferSize, false)) return 0;
    if (!safe_alloc_int_array(&mobileAverageBuffer, mobileAverageBufferSize, false)) return 0;
    if (!safe_alloc_int_array(&varianceBuffer, varianceBufferSize, false)) return 0;

    sampleBufferIndex = 0;
    sampleBufferValid = false;
    mobileAverageBufferIndex = 0;
    mobileAverageBufferValid = false;
    varianceBufferIndex = 0;
    varianceBufferValid = false;

    variance = RADAR_BOOTING; // keep your sentinel
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
    // fallback to internal RAM if psram not available
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
    if (sampleBuffer != NULL) {
        free(sampleBuffer);
        sampleBuffer = NULL;
    }
    if (mobileAverageBuffer != NULL) {
        free(mobileAverageBuffer);
        mobileAverageBuffer = NULL;
    }
    if (varianceBuffer != NULL) {
        free(varianceBuffer);
        varianceBuffer = NULL;
    }

    sampleBufferSize = MAX_SAMPLEBUFFERSIZE;
    mobileAverageFilterSize = MAX_AVERAGEBUFFERSIZE;
    mobileAverageBufferSize = MAX_AVERAGEBUFFERSIZE;
    varianceBufferSize = MAX_SAMPLEBUFFERSIZE;

    sampleBufferIndex = varianceBufferIndex = mobileAverageBufferIndex = 0;
    sampleBufferValid = varianceBufferValid = mobileAverageBufferValid = false;

    variance = RADAR_BOOTING; // reset sentinel
    varianceAR = 0;
    varianceIntegral = 0;
    varianceSample = 0;

    return 1;
}

// ----- Configuration -----
int motionDetector_config(int sampleBufSize /*=256*/, int mobileAvgSize /*=64*/, int varThreshold /*=3*/, int varIntegratorLimit /*=3*/, bool enableAR /*=false*/) {
    // clamp sizes to allowed maximums and minimums
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

    // if buffers already allocated, reallocate to new sizes (simple approach: deinit + init)
    if (sampleBuffer || mobileAverageBuffer || varianceBuffer) {
        motionDetector_deinit();
        // attempt to allocate with new sizes in internal RAM
        motionDetector_init();
    }

    return 1;
}

// ----- Core processing -----
int motionDetector_process(int sample /*=0*/) {
    if ((sampleBuffer == NULL) || (mobileAverageBuffer == NULL) || (varianceBuffer == NULL)) {
        return RADAR_UNINITIALIZED;
    }

    if (sample < minimumRSSI) {
        return RADAR_RSSI_TOO_LOW;
    }

    // store new sample at current write index
    sampleBuffer[sampleBufferIndex] = sample;
    // advance write index (bufferIndex points to next write pos)
    sampleBufferIndex = (sampleBufferIndex + 1) % sampleBufferSize;
    if (!sampleBufferValid && sampleBufferIndex == 0) sampleBufferValid = true;

    if (!sampleBufferValid && (sampleBufferIndex < mobileAverageFilterSize)) {
        // we don't yet have enough samples to compute requested mobile average
        variance = RADAR_BOOTING;
        return variance;
    }

    // compute mobile average from the latest mobileAverageFilterSize samples
    long long sum = 0;
    for (size_t i = 0; i < mobileAverageFilterSize; ++i) {
        // newest sample is at (sampleBufferIndex - 1)
        size_t idx = (sampleBufferIndex + sampleBufferSize - 1 - i) % sampleBufferSize;
        sum += sampleBuffer[idx];
    }
    mobileAverage = (int)(sum / (long long)mobileAverageFilterSize);

    // store latest mobile average into circular buffer (for logging / future use)
    mobileAverageBuffer[mobileAverageBufferIndex] = mobileAverage;
    mobileAverageBufferIndex = (mobileAverageBufferIndex + 1) % mobileAverageBufferSize;
    if (!mobileAverageBufferValid && mobileAverageBufferIndex == 0) mobileAverageBufferValid = true;

    // compute variance sample (squared deviation)
    long long diff = (long long)sample - (long long)mobileAverage;
    long long vs = diff * diff;
    if (vs > INT32_MAX) vs = INT32_MAX; // clamp to avoid overflow in 32-bit environments
    varianceSample = (int)vs;

    // push varianceSample into variance circular buffer at current index
    varianceBuffer[varianceBufferIndex] = varianceSample;

    // compute variance integrator over last varianceIntegratorLimit samples (including current)
    long long vintegral = 0;
    for (size_t i = 0; i < varianceIntegratorLimit; ++i) {
        size_t idx = (varianceBufferIndex + varianceBufferSize - i) % varianceBufferSize;
        vintegral += varianceBuffer[idx];
    }
    // update varianceBufferIndex after integral calculation
    varianceBufferIndex = (varianceBufferIndex + 1) % varianceBufferSize;
    if (!varianceBufferValid && varianceBufferIndex == 0) varianceBufferValid = true;

    if (vintegral > INT32_MAX) vintegral = INT32_MAX;
    varianceIntegral = (int)vintegral;

    // autoregressive smoothing (simple IIR)
    varianceAR = (varianceIntegral + varianceAR) / 2;

    // choose output variance according to mode
    if (enableAutoRegressive) {
        variance = varianceAR;
    } else {
        variance = varianceIntegral;
    }

    // threshold handling
    if (enableThreshold > 0) {
        if (variance >= varianceThreshold) {
            detectionLevel = variance;
            return detectionLevel;
        } else if (variance >= 0) {
            detectionLevel = 0;
            return detectionLevel;
        }
    }

    // if threshold disabled, return raw variance value
    return variance;
}

// ----- Simple wrapper that reads RSSI and calls process -----
int motionDetector() {
    int RSSIlevel = (int)WiFi.RSSI();

    // If RSSIlevel == 0 we probably have no upstream AP (WiFi library behavior)
    if (RSSIlevel == 0) {
        return RADAR_INOPERABLE;
    }

    return motionDetector_process(RSSIlevel);
}

// ----- Bistatic / ESP32 specific helpers (improved) -----
#define SCANMODE_STA 0
#define SCANMODE_SOFTAP 1
#define SCANMODE_WIFIPROBE 2

static int scanMode = SCANMODE_STA;

uint8_t strongestClientBSSID[6] = {0};
int strongestClientRSSI = -100;
int strongestClientfound = 0;

int modeRes = 0;
uint8_t BSSIDinUse[6] = {0};

int bistatic_get_rssi_SoftAP_strongestClient() {
    int rssi = 0;
    wifi_sta_list_t stationList;
    esp_err_t scanRes = esp_wifi_ap_get_sta_list(&stationList);

    if (scanRes != ESP_OK) {
        return 0;
    }

    // find strongest client on first run
    if (!strongestClientfound) {
        strongestClientRSSI = -100;
        for (int i = 0; i < (int)stationList.num; ++i) {
            wifi_sta_info_t station = stationList.sta[i];
            uint8_t *currentBSSID = station.mac;
            int currentRSSI = station.rssi;
            // valid RSSI expected in negative dBm range
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
        // no clients connected -> pick a scan mode fallback based on WiFi mode
        if (modeRes & WIFI_MODE_APSTA) {
            if (WiFi.status() == WL_CONNECTED) scanMode = SCANMODE_STA;
            else scanMode = SCANMODE_WIFIPROBE;
        }
        rssi = 0;
    }

    memcpy(BSSIDinUse, strongestClientBSSID, 6);
    return rssi;
}

uint8_t strongestBSSID[6] = {0};
int strongestRSSI = -100;
int strongestChannel = 0;
int strongestAPfound = 0;

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
    if (localBSSID == NULL) {
        Serial.print("NULL");
        return;
    }
    for (int i = 0; i < 6; ++i) {
        if (localBSSID[i] < 0x10) Serial.print('0'); // leading zero for readability
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
        RSSIlevel = (int)WiFi.RSSI();
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

            if (RSSIlevel == 0) {
                scanMode = SCANMODE_SOFTAP;
            }
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

// ----- small setters -----
int motionDetector_enable_serial_CSV_graph_data(int serialCSVen) {
    if (serialCSVen >= 0) enableCSVout = serialCSVen;
    return enableCSVout;
}

int motionDetector_set_minimum_RSSI(int rssiMin) {
    // clamp to reasonable RSSI bounds: typically negative dBm values up to 0
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

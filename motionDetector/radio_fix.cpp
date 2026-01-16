// motionDetector_radio_kalman.cpp // Improved radio layer with Kalman filter smoothing and optional promiscuous sniffing for ESP32 // Drop-in companion to your motionDetector core code. Keeps original API; adds radio helpers and Kalman smoothing.

#include "motionDetector.h" #include <WiFi.h> #include "esp_wifi.h" #include <string.h> #include <stdint.h>

// ---------------- Kalman Filter (simple 1D) ---------------- struct KalmanFilter { float q; // process noise covariance float r; // measurement noise covariance float x; // value float p; // estimation error covariance float k; // kalman gain

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

// ---------------- Globals (radio + kalman) ---------------- static KalmanFilter rssiKalman(1.0f, 4.0f, -60.0f); // tuned defaults (processNoise, measurementNoise, initial rssi) static bool useKalman = true;

#ifdef ESP32 // Promiscuous sniffing state static volatile int lastPromiscuousRSSI = 0; // last observed RSSI from sniffed packets static volatile bool promRssiAvailable = false; static volatile bool promiscuousEnabled = false;

// Callback signature for promiscuous packets (ESP-IDF / Arduino-ESP32) static void wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) { // buf is actually wifi_promiscuous_pkt_t* if (buf == nullptr) return; wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*) buf; int8_t rssi = pkt->rx_ctrl.rssi; // signed dBm value // Basic sanity check of the RSSI range if ((rssi < -120) || (rssi > 0)) return; lastPromiscuousRSSI = (int) rssi; promRssiAvailable = true; }

// Enable or disable promiscuous sniffing void motionDetector_radio_enable_promiscuous(bool enable) { if (enable && !promiscuousEnabled) { esp_wifi_set_promiscuous(true); esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_cb); promiscuousEnabled = true; } else if (!enable && promiscuousEnabled) { esp_wifi_set_promiscuous_rx_cb(NULL); esp_wifi_set_promiscuous(false); promiscuousEnabled = false; promRssiAvailable = false; lastPromiscuousRSSI = 0; } } #else // stub for non-ESP32 platforms void motionDetector_radio_enable_promiscuous(bool enable) { (void)enable; } #endif

// ---------------- Radio helper: get best available RSSI ---------------- // Tries to read the most reliable RSSI available in this order: // 1) WiFi.RSSI() if connected and non-zero // 2) last promiscuous RSSI observed (if enabled and available) // 3) active scan strongest AP (if available) // returns 0 if no valid RSSI available (keep existing library behavior)

int motionDetector_radio_read_raw_rssi() { int rssi = (int) WiFi.RSSI(); // treat 0 as invalid / not connected (consistent with original code) if (rssi != 0) return rssi;

#ifdef ESP32 if (promRssiAvailable) { // read volatile once int snap = lastPromiscuousRSSI; // optionally clear flag here, but we keep latest available return snap; }

// fallback: quick active scan on current channel (non-blocking not supported here easily)
// We'll perform a short single-channel scan to find any APs nearby and return the strongest RSSI
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

// ---------------- Public: enable/disable Kalman smoothing ---------------- void motionDetector_radio_use_kalman(bool enable, float processNoise, float measurementNoise, float initialValue) { useKalman = enable; if (enable) { // if user provided noise params as <= 0, keep defaults if (processNoise > 0.0f) rssiKalman.q = processNoise; if (measurementNoise > 0.0f) rssiKalman.r = measurementNoise; rssiKalman.reset(initialValue); } }

// ---------------- Integration helper: read-and-filter RSSI to be used by motionDetector_process ---------------- // This returns the filtered RSSI (if Kalman enabled) or the raw RSSI otherwise. int motionDetector_radio_read_filtered_rssi() { int raw = motionDetector_radio_read_raw_rssi(); if (raw == 0) return 0; // preserve original "no-signal" behavior if (!useKalman) return raw; float filtered = rssiKalman.update((float) raw); // round toward zero and return as int return (int) roundf(filtered); }

// ---------------- Example wrapper replacement for motionDetector() which uses the radio layer ---------------- // Keep your original motionDetector() if you prefer; this is an improved variant that tries the radio layer first. int motionDetector_radio_layer() { int RSSIlevel = motionDetector_radio_read_filtered_rssi(); if (RSSIlevel == 0) return RADAR_INOPERABLE; return motionDetector_process(RSSIlevel); }

// ---------------- Notes (compile-time and runtime) ---------------- // - This file adds optional promiscuous sniffing on ESP32. To enable call motionDetector_radio_enable_promiscuous(true); // - To enable Kalman smoothing: motionDetector_radio_use_kalman(true, 1.0f, 4.0f, -60.0f); // - Tuning: decrease measurementNoise r to make filter follow measurements more closely; increase r to smooth more. // - Promiscuous sniffing may interfere with normal WiFi operation in some modes (channel lock). Use carefully. // - For multi-BSSID fusion / per-BSSID Kalman filters consider extending the promiscuous callback to store per-MAC filters in a small LRU cache.

// End of file

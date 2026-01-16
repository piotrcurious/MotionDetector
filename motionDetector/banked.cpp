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

// ---------------- Advanced Kalman for ACTIVE MODE (constant-velocity RSSI model) ---------------- // This section extends the radio layer for ACTIVE SCAN MODE, where we can afford a richer model. // State vector x = [ RSSI , dRSSI/dt ]^T // This captures both signal level and its temporal gradient (movement-induced fading).

struct Kalman2D { // State float x0; // RSSI float x1; // RSSI rate of change

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

// dt = time step in seconds (for active scan, typically 0.2–0.5s)
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

// Active-mode Kalman instance (separate from passive/STA filter) static Kalman2D activeKalman(1.5f, 0.2f, 8.0f, -60.0f); static bool useActiveKalman = true; static unsigned long lastActiveUpdateMs = 0;

// Call this instead of motionDetector_radio_read_filtered_rssi() in ACTIVE SCAN mode int motionDetector_radio_read_active_filtered_rssi() { int raw = motionDetector_radio_read_raw_rssi(); if (raw == 0) return 0;

if (!useActiveKalman) return raw;

unsigned long now = millis();
float dt = 0.3f; // default active scan interval
if (lastActiveUpdateMs != 0) {
    dt = (now - lastActiveUpdateMs) * 0.001f;
    if (dt < 0.05f) dt = 0.05f;
    if (dt > 2.0f) dt = 2.0f;
}
lastActiveUpdateMs = now;

float filtered = activeKalman.update((float)raw, dt);
return (int) roundf(filtered);

}

// Enable/disable advanced Kalman in active mode and retune void motionDetector_radio_use_active_kalman(bool enable, float q_rssi, float q_rate, float r_meas, float initRSSI) { useActiveKalman = enable; if (enable) { if (q_rssi > 0) activeKalman.q_rssi = q_rssi; if (q_rate > 0) activeKalman.q_rate = q_rate; if (r_meas > 0) activeKalman.r_meas = r_meas; activeKalman.reset(initRSSI); lastActiveUpdateMs = 0; } }

// Interpretation hints: // - x0 (RSSI) is the smoothed signal level // - x1 (dRSSI/dt) approximates motion-induced fading rate; |x1| spikes correlate strongly with movement // - Feeding x1 (or x1^2) into your variance/integrator dramatically improves sensitivity to motion while //   suppressing static multipath noise

// ---------------- Per-BSSID Kalman Banks + Channel-aware Filters ---------------- // Implements a small LRU cache of Kalman2D filters keyed by BSSID and channel. // Each observed MAC address gets its own Kalman2D instance; updates from promiscuous // packets or active scans update the corresponding bank. Banks are limited to // MAX_KALMAN_BANKS to bound RAM usage. Channel-level Kalman filters fuse all // banks observed on the same WiFi channel to create a channel-aware estimate.

#define MAX_KALMAN_BANKS 8 #define MAX_CHANNELS 14 #define BANK_UNUSED 0 #define BANK_USED 1

struct BssidBank { uint8_t mac[6]; Kalman2D kf;        // per-BSSID 2D Kalman (RSSI, rate) int channel;        // channel observed on int last_rssi;      // last raw RSSI observed unsigned long last_seen_ms; // last update time uint32_t age_counter; // used for simple LRU (larger = older) uint8_t state;      // BANK_USED/BANK_UNUSED };

static BssidBank bssidBanks[MAX_KALMAN_BANKS]; static uint32_t global_age_counter = 1;

// Per-channel fusion Kalman (1D or 2D depending on need) - we use Kalman2D to keep consistent state static Kalman2D channelKFs[MAX_CHANNELS + 1]; static bool channelKFInit[MAX_CHANNELS + 1] = {false};

// Initialize banks (call at startup) void motionDetector_radio_banks_init() { for (int i = 0; i < MAX_KALMAN_BANKS; ++i) { bssidBanks[i].state = BANK_UNUSED; bssidBanks[i].last_rssi = -200; bssidBanks[i].channel = 0; bssidBanks[i].last_seen_ms = 0; bssidBanks[i].age_counter = 0; } for (int ch = 0; ch <= MAX_CHANNELS; ++ch) { channelKFInit[ch] = false; } }

// Helper: find bank by MAC; returns index or -1 static int find_bank_by_mac(const uint8_t *mac) { for (int i = 0; i < MAX_KALMAN_BANKS; ++i) { if (bssidBanks[i].state == BANK_USED) { if (memcmp(bssidBanks[i].mac, mac, 6) == 0) return i; } } return -1; }

// Helper: allocate or evict LRU bank, returns index or -1 on failure static int allocate_bank_for_mac(const uint8_t *mac, int channel, float q0 = 1.5f, float q1 = 0.2f, float r = 8.0f, float initRSSI = -60.0f) { // if exists, return it int idx = find_bank_by_mac(mac); if (idx >= 0) return idx;

// find an unused bank first
for (int i = 0; i < MAX_KALMAN_BANKS; ++i) {
    if (bssidBanks[i].state == BANK_UNUSED) {
        idx = i;
        break;
    }
}
// otherwise evict LRU
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

// initialize bank
memcpy(bssidBanks[idx].mac, mac, 6);
bssidBanks[idx].kf = Kalman2D(q0, q1, r, initRSSI);
bssidBanks[idx].channel = channel;
bssidBanks[idx].last_rssi = (int)initRSSI;
bssidBanks[idx].last_seen_ms = millis();
bssidBanks[idx].age_counter = 0;
bssidBanks[idx].state = BANK_USED;
return idx;

}

// Update channel KF with a new fused measurement (z = channel-level RSSI) static void update_channel_kf(int channel, float z, float dt) { if (channel < 0 || channel > MAX_CHANNELS) return; if (!channelKFInit[channel]) { channelKFs[channel] = Kalman2D(1.5f, 0.2f, 8.0f, z); channelKFInit[channel] = true; return; } channelKFs[channel].update(z, dt); }

// Promiscuous callback enhancement: parse MAC and update per-BSSID bank #ifdef ESP32 static void wifi_promiscuous_enhanced_cb(void* buf, wifi_promiscuous_pkt_type_t type) { if (buf == nullptr) return; wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*) buf; int8_t rssi = pkt->rx_ctrl.rssi; if ((rssi < -120) || (rssi > 0)) return;

// Extract MAC from 802.11 header in payload. The header format depends on frame type.
// We'll attempt to read Address 2 (transmitter) at offset 10 for most data/management frames,
// but this is not universal. Robust parsing requires parsing frame control bits.
const uint8_t *payload = pkt->payload;
if (payload == NULL) return;

// Basic validation: packet length must be at least 24 bytes for header
if (pkt->rx_ctrl.sig_len < 24) return;

// Rough attempt to extract transmitter MAC (addr2) located at payload + 10 in many frames
const uint8_t *mac = payload + 10;

// channel is available in rx_ctrl
int ch = pkt->rx_ctrl.channel;

// allocate or find bank for this MAC
int bank = allocate_bank_for_mac(mac, ch);
if (bank < 0) return;

// update bank Kalman with raw RSSI. We approximate dt from last_seen_ms
unsigned long now = millis();
float dt = 0.25f;
if (bssidBanks[bank].last_seen_ms != 0) dt = (now - bssidBanks[bank].last_seen_ms) * 0.001f;
if (dt <= 0) dt = 0.05f;
bssidBanks[bank].last_seen_ms = now;
bssidBanks[bank].last_rssi = rssi;
bssidBanks[bank].channel = ch;
bssidBanks[bank].age_counter = ++global_age_counter;

float filtered = bssidBanks[bank].kf.update((float) rssi, dt);

// update channel fusion KF using this bank's filtered RSSI
update_channel_kf(ch, filtered, dt);

} #endif

// Compute fused motion energy across banks (use top-N strongest banks) float motionDetector_radio_compute_fused_motion_energy(int topN) { // build list of banks with last_rssi > -150 struct Candidate { int idx; int rssi; float rate; } cands[MAX_KALMAN_BANKS]; int n = 0; unsigned long now = millis(); for (int i = 0; i < MAX_KALMAN_BANKS; ++i) { if (bssidBanks[i].state == BANK_USED) { // expire old banks older than 30s if ((now - bssidBanks[i].last_seen_ms) > 30000UL) { bssidBanks[i].state = BANK_UNUSED; continue; } cands[n].idx = i; cands[n].rssi = bssidBanks[i].last_rssi; // x1 is approximated by kf.x1 cands[n].rate = bssidBanks[i].kf.x1; ++n; } } if (n == 0) return 0.0f;

// sort candidates by rssi descending (simple selection sort for small N)
for (int i = 0; i < n; ++i) {
    int best = i;
    for (int j = i+1; j < n; ++j) if (cands[j].rssi > cands[best].rssi) best = j;
    if (best != i) { Candidate tmp = cands[i]; cands[i] = cands[best]; cands[best] = tmp; }
}

if (topN <= 0 || topN > n) topN = n;

// compute weighted motion energy: sum of (w_i * rate_i^2) where w_i from RSSI strength
float energy = 0.0f;
float weightSum = 0.0f;
for (int i = 0; i < topN; ++i) {
    float r = (float) cands[i].rssi; // negative value e.g. -40
    // convert RSSI to linear weight: higher RSSI -> higher weight
    float w = 1.0f / (1.0f + expf(( -r - 40.0f) * 0.1f )); // smooth mapping
    float contribution = w * (cands[i].rate * cands[i].rate);
    energy += contribution;
    weightSum += w;
}
if (weightSum > 0.0f) energy = energy / weightSum;
return energy;

}

// Public API: enable per-BSSID banks and channel fusion void motionDetector_radio_enable_bssid_banks(bool enable) { #ifdef ESP32 if (enable) { motionDetector_radio_banks_init(); // wire the enhanced promiscuous callback esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_enhanced_cb); esp_wifi_set_promiscuous(true); } else { esp_wifi_set_promiscuous_rx_cb(NULL); esp_wifi_set_promiscuous(false); } #endif }

// Public API: get channel fused motion energy for the channel currently in use (or channel 0 for "all") float motionDetector_radio_channel_motion_energy(int channel, int topN) { if (channel < 0 || channel > MAX_CHANNELS) channel = 0; if (channel == 0) { // fuse across banks regardless of channel return motionDetector_radio_compute_fused_motion_energy(topN); } if (!channelKFInit[channel]) return 0.0f; // return magnitude of estimated rate for channel KF return fabsf(channelKFs[channel].x1); }

// Usage note: preferred pipeline for ACTIVE mode with per-BSSID + channel-aware fusion // 1) Enable per-BSSID banks: motionDetector_radio_enable_bssid_banks(true) // 2) In your active scan loop call motionDetector_radio_read_raw_rssi() or rely on promiscuous packets // 3) Periodically query motion energy: float E = motionDetector_radio_channel_motion_energy(currentChannel, 4) // 4) Convert E into your detection metric (e.g., threshold or integrate over time)

// End of file

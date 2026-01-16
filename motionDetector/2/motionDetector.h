#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

// Status codes
#define RADAR_BOOTING -1
#define RADAR_INOPERABLE 0
#define RADAR_UNINITIALIZED -2
#define RADAR_RSSI_TOO_LOW -3
#define WIFI_MODEINVALID -4

// Default limits
#define MINIMUM_RSSI -100
#define ABSOLUTE_RSSI_LIMIT -120

// Core API
int motionDetector_init();
int motionDetector_init_PSRAM();
int motionDetector_deinit();
int motionDetector_config(int sampleBufSize, int mobileAvgSize, int varThreshold, int varIntegratorLimit, bool enableAR);
int motionDetector_process(int sample);

// Radio Layer API
void motionDetector_radio_enable_promiscuous(bool enable);
void motionDetector_radio_enable_bssid_banks(bool enable);
void motionDetector_radio_use_kalman(bool enable, float processNoise = 1.0f, float measurementNoise = 4.0f, float initialValue = -60.0f);
void motionDetector_radio_use_active_kalman(bool enable, float q_rssi = 1.5f, float q_rate = 0.2f, float r_meas = 8.0f, float initRSSI = -60.0f);
int motionDetector_radio_read_raw_rssi();
int motionDetector_radio_read_filtered_rssi();
int motionDetector_radio_read_active_filtered_rssi();
float motionDetector_radio_compute_fused_motion_energy(int topN);
float motionDetector_radio_channel_motion_energy(int channel, int topN);

// Wrapper API
int motionDetector();
int motionDetector_esp();

// Configuration Setters
int motionDetector_enable_serial_CSV_graph_data(int serialCSVen);
int motionDetector_set_minimum_RSSI(int rssiMin);
int motionDetector_enable_alarm(int thresholdEnable);
int motionDetector_set_alarm_threshold(int alarmThreshold);

#endif // MOTION_DETECTOR_H

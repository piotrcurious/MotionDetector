# Integration Guide for Banked Kalman Motion Detector

To fully integrate the Banked Kalman solution into your existing code, follow these steps:

### 1. Enable BSSID Banks in `setup()`
In your `setup()` function, after `motionDetector_init()`, enable the BSSID banks to start tracking individual devices:

```cpp
void setup() {
  // ... existing setup code ...
  motionDetector_init();
  motionDetector_config(64, 16, 3, 3, false);
  
  // NEW: Enable advanced BSSID tracking
  motionDetector_radio_enable_bssid_banks(true);
  // ...
}
```

### 2. Feed Probe Requests into the Kalman Bank
Update your `probeRequest` callback to feed the received signal data into the Kalman filter bank. This allows the motion detector to "learn" from probe requests even when not connected:

```cpp
void probeRequest(WiFiEvent_t event, WiFiEventInfo_t info) {
  // ... existing code ...
  
  // NEW: Feed this probe into the Kalman bank
  // Use the MAC address, RSSI, and channel from the event info
  motionDetector_radio_update_from_probe(
    info.wifi_ap_probereqrecved.mac, 
    info.wifi_ap_probereqrecved.rssi, 
    WiFi.channel() // or info.wifi_ap_probereqrecved.channel if available
  );
  
  // ... rest of your logic ...
}
```

### 3. Use Fused Motion Energy (Optional)
If you want to use the advanced "fused energy" metric (which combines motion data from all tracked devices), you can query it in your `loop()`:

```cpp
void loop() {
  // ...
  if (EEPROM.readByte(0) == 1) {
    // Standard processing
    motionLevel = motionDetector_esp();
    
    // OPTIONAL: Get fused motion energy from top 4 strongest devices
    float fusedEnergy = motionDetector_radio_compute_fused_motion_energy(4);
    if (fusedEnergy > 5.0) { // Example threshold
       Serial.println("High motion energy detected across multiple devices!");
    }
    
    // ...
  }
}
```

### 4. Sync Minimum RSSI with EEPROM
Ensure your motion detector uses the minimum RSSI value stored in your EEPROM:

```cpp
// In loop(), before calling motionDetector_esp()
int minRssi = EEPROM.readByte(3) * -1;
motionDetector_set_minimum_RSSI(minRssi);
```

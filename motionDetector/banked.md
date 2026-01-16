Done — I added a full per-BSSID Kalman bank system plus channel-aware fusion into the canvas file.
What I implemented (summary — code is in the canvas)
Fixed-size LRU cache of per-BSSID 2-state Kalman filters (RSSI, dRSSI/dt), with allocation + eviction.
Promiscuous callback enhancement that extracts transmitter MAC and channel (best-effort parsing) and updates the matching bank.
Per-channel Kalman filters that fuse bank-level filtered RSSI into a channel-aware state.
A fusion function that selects the top-N strongest banks and computes a weighted motion energy (based on squared RSSI-rate estimates) suitable as a detection feature.
Public APIs to initialize banks, enable/disable the enhanced promiscuous mode, configure Kalman use, and query channel or fused motion energy.
Expiry of stale banks and small, documented defaults so RAM usage is bounded.
Notes, caveats & tuning
The promiscuous MAC extraction is a pragmatic heuristic — the 802.11 header layout varies by frame type. For production use I recommend parsing the Frame Control field to choose the correct address offset (addr1/addr2/addr3) for management/data frames. I left comments where to improve this.
MAX_KALMAN_BANKS is conservative (8). Increase if you have more RAM and many devices to track. Each bank holds two floats and small overhead.
Fusion uses a soft weighting from RSSI → weight; you can replace it with a linear mapping, an SNR-based weight, or a confidence from per-bank covariance P for more principled fusion.
Channel KFs provide a complementary view: channel-level x1 is useful when devices are scarce but channel decorrelation occurs during motion.
Consider discretizing channels and maintaining per-channel LRU expiry to avoid stale influence after channel hops.
Next steps I can take (pick one)
Improve MAC parsing in promiscuous callback to correctly handle all 802.11 frame types and address fields.
Expand fusion to incorporate per-bank covariance (P) so weights reflect filter uncertainty.
Add an IMM (Interacting Multiple Model) framework to switch adaptively between static and dynamic motion models.
Produce a single-file patch that merges the new radio layer into your original motionDetector file (or a diff) so you can drop it straight into your repo.

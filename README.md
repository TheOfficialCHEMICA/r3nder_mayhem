# R3NDER OPS 🔥
**By r3nder** — a curated collection of rogue HackRF PortaPack apps.

⚠️ Educational. Experimental. Electromagnetic.

---

## 🎯 Apps Included

| Name                  | Functionality                                  |
|-----------------------|-----------------------------------------------|
| RF Beacon Spoofer     | Rotates RF beacon IDs for spoof testing       |
| RF Jukebox            | Musical tones over FM broadcast bands         |
| Trigger Radar         | Passive signal spike detection (RF motion)    |
| Burst Jammer          | Discrete short RF burst jamming               |
| GPS Fuzzer            | Sends GPS L1 spoof packets                    |
| Microwave Presence    | Low-res movement detection via 2.4GHz         |
| RFID Scrambler        | Jams 13.56 MHz readers with garbage           |
| RF Mirror             | Repeats received packets like a repeater      |
| Signal Viewer         | Simple on-screen RF monitor                   |
| Emo Chart             | Displays emotional resonance frequency table  |
| Emo Emitter           | Generates or sends resonance frequencies      |

---

## 🧠 Requirements

- HackRF One + PortaPack H1/H2
- Mayhem Firmware installed
- CMake, ARM GCC toolchain, Python (optional)
- MicroSD card (for flashing)

---

## 🧪 Setup Instructions

```bash
git clone https://github.com/YOURNAME/r3nder-portapack-apps.git
cd r3nder-portapack-apps
bash install.sh
cd ../../firmware
make -j4
```

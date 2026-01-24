# `/src` utilities (Scribit open-firmware)

This folder contains **standalone** Python utilities for working with a Scribit wall-drawing robot running **open firmware**.  
They are intended to be simple, hackable tools for **manual control**, **calibration**, and **SVG → G-code** generation.

Current files:

- `scribit_jog_cli.py` — interactive keyboard “jog” controller  
- `scribit_svg_to_gcode.py` — convert SVG drawings into Scribit-compatible G-code  

---

## 1. `scribit_jog_cli.py` — interactive jog controller

### What it does
- Provides a **terminal (curses) UI** to manually move the Scribit using the keyboard.
- Sends **small, relative movements** to the robot so it can be safely used anywhere on the wall.
- Runs a tiny **HTTP server** that serves short G-code snippets.
- Uses **MQTT** to instruct the Scribit to fetch and execute those G-code files.

This tool is mainly for:
- testing motion
- calibration
- centering / recovering position
- debugging firmware behavior

### How it works (high level)
1. Each key press maps to a relative cable-length move:
   - left/right cord in or out
   - diagonal combinations
2. The script exposes URLs like:
   ```
   http://<host-ip>/g/LEFTISH.gcode
   ```
3. When a key is pressed, it publishes an MQTT `print` command telling the robot to download and execute that URL.
4. All motion is **relative** (`G91`) and expressed directly in **left/right cable deltas**, so no absolute XY state is assumed.

> Note: current open firmware requires the HTTP server to be on **port 80**, and the URL must not include a `:port`.

### Example usage
```bash
sudo python3 scribit_jog_cli.py \
  --robot-id 30aea4d9fc5c \
  --mqtt-host 192.168.240.2 \
  --host-ip 192.168.240.2 \
  --step 2.0 \
  --feed 900
```

### Typical controls
- Arrow keys / `WASD` : jog in intuitive directions
- `Q / E`             : left cord in / out
- `Z / C`             : right cord in / out
- `J / K`             : rotate pen carousel CCW/CW by 'step' degrees (relative Z)
- `1 / 2 / 3 / 4`     : pen 1/2/3/4 DOWN (slot Z then G101)
- `! / @ / # / $`     : pen 1/2/3/4 UP   (slot Z only)
- `H`                 : G77 (home Z-cylinder / carousel using hall sensor)
- `[ / ]`             : decrease / increase step size
- `- / =`             : decrease / increase feed rate
- `x`                 : stop/reset current motion
- `ESC / Ctrl-G`      : quit

### Safety note: pen carousel rotation

> **Note:** Pen down is implemented by rotating the pen carousel **clockwise** to a specific latch position. Once a pen is fully down, **do not rotate the carousel further clockwise**, as this can damage the mechanism. **Counterclockwise rotation is always safe** and may be used to recover or adjust the carousel position.

---

## 2. `scribit_svg_to_gcode.py` — SVG → Scribit G-code converter

### What it does
- Parses an **SVG file** (stroked paths).
- Maps it onto the Scribit wall coordinate system.
- Outputs Scribit-style **relative G-code** based on cable lengths.
- Generates two output files:
  - `drawing.gcode` — the actual drawing
  - `bbox_dots.gcode` — small dots at the mapped bounding-box corners (sanity check)
- Assumes the robot starts at the center of the canvas

This tool is intended for **offline verification** and safe generation of drawing programs.

### Example usage
```bash
python3 scribit_svg_to_gcode.py input.svg \
  --D_mm 1860 \
  --fit_frac 0.7 \
  --step_mm 1.0 \
  --travel_step_mm 5.0 \
  --f_travel 600 \
  --f_draw 300
```

---


## SVG → G-code conversion logic


### Coordinate system convention
The wall coordinate system uses the **upper-left corner as (0, 0)**, and the **Y axis increases downward**. This matches SVG’s coordinate convention and is used consistently throughout the G-code generation logic.


### 1. Wall geometry and kinematics
The Scribit position is defined by **two cable lengths**:

- Left cable:  
  ```
  L = sqrt(x² + y²)
  ```
- Right cable:  
  ```
  R = sqrt((D − x)² + y²)
  ```

Where:
- `(x, y)` is the pen position on the wall
- `D` is the distance between the two mounting nails

All motion is emitted as **relative cable deltas**:
```
G1 X(dL) Y(-dR)
```

---

### 2. Mapping SVG space → wall space
1. All drawable SVG paths are collected.
2. A bounding box is computed in SVG coordinates.
3. A **uniform scale** is chosen so the drawing fits within:
   ```
   fit_frac × D
   ```
4. The drawing is centered on the wall at:
   ```
   (D/2, D/2)
   ```

This preserves aspect ratio and ensures the drawing stays in bounds.

---

### 3. Curve sampling and step control
SVG paths may contain curves. To ensure:
- smooth motion
- predictable cable tension
- no large jumps

Each path is **densified**:
- Total wall length is estimated.
- The path is sampled into segments of roughly `step_mm`.
- Each segment becomes one relative `G1` move.

---

### 4. Pen handling and colors
- SVG stroke colors are normalized (hex, rgb(), etc.).
- Colors are mapped to Scribit pen slots (up to 4).
- The generated G-code inserts:
  - pen select
  - pen up / pen down
  - safe travel moves between strokes

---

## Notes
- These tools assume the Scribit open firmware workflow:
  1. G-code is hosted on an HTTP server
  2. The robot fetches it via an MQTT `print` command
- The jog tool and SVG converter are intentionally **decoupled**:
  - one is for interactive control
  - the other is for deterministic, offline generation

---

Future scripts placed in this folder should follow the same philosophy:  
**simple, inspectable, and safe for hardware experimentation**.

---

## 3. Example MQTT commands
### View MQTT server log
```bash
  journalctl -u mosquitto -f &
```

### Monitor MQTT Traffic (subscribe to the topic)
```bash
mosquitto_sub -h 192.168.240.2 -p 1883 -u scribit -P 'scribit' \
  -t "tout/30aea4d9fc5c/#" -v
```

### Make Scribit ready (leave BOOT, go IDLE)
```bash
mosquitto_pub -h 192.168.240.2 -p 1883 -u scribit -P 'scribit' \
  -t "tin/30aea4d9fc5c/status" -m "{}"
```

### Print (download gcode and draw on the wall)
1. bring the robot to the center with `scribit_jog_cli.py`
2. setup an HTTP server for download `python3 -m http.server 80 &`
3. send `print` command through MQTT
```bash
mosquitto_pub -h 192.168.240.2 -p 1883 -u scribit -P 'scribit' \
  -t "tin/30aea4d9fc5c/print" -m "http://192.168.240.2/drawing.gcode;M18"
```

### Reset
```bash
mosquitto_pub -h 192.168.240.2 -u scribit -P 'scribit' -t "tin/30aea4d9fc5c/reset" -m "N"   # stop current print stream
mosquitto_pub -h 192.168.240.2 -u scribit -P 'scribit' -t "tin/30aea4d9fc5c/reset" -m "Y"   # hard reset (reboot)
```
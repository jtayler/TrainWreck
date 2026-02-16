# TrainWreck
TrainWreck is a Nano-based control app to run a model Z guage train into the ground and off the rails!

This app is a **fully scripted model train control system** with display, signals, smooth motion, and themed routes.

### Core Features

**1. Smooth S-Curve Acceleration & Braking**

* Custom ramping (non-linear “bell curve” step size).
* MPH conversion from PWM (0–72 MPH scale).
* Controlled ramp up / ramp down.

**2. Easy Speed/Direction Control**

* Forward / Reverse switching.
* Direction shown on display.
* Alternates easily in route loops.

**3. Traffic Light Integration**

* 🔴 Red = stopped / hault
* 🟢 Green = acceleration, fast leg
* 🟡 Yellow = decelerating, slow leg

**4. OLED Dashboard (128x64 SH1106)**
Display Shows:
* Route name (top)
* Large speed readout (MPH)
* Direction (FORWARD / REVERSE / HALTED)
* Status line (RAMP TO, DOWN TO, FAST LEG, etc.)

Clean, centered layout with inverted speed panel.

**5. Dip Behavior (Speed Drops While Running)**
* Periodic slow segments during runs.
* Simulates grades, curves, or realism.
* Adjustable dip count and duration.

**6. Station Docking (IR Sensor)**
* Automatic station stops always work.
* Timed crawl hold before full stop.
* Prevents random stop points.

**7. Scripted Route Library**
Each route easily defines fun:

* Direction changes
* Speed profiles
* Stop timing
* Dip frequency
* Randomized speeds in some routes

Examples:

* Vanderbilt Central
* Hudson Limited
* Grand Central Line
* Rio-Jess Express
* Orient Express
* Long Train Running
* Circle Line

**8. Fully Autonomous Operation**
`loop()` runs every route in sequence continuously.

---

### In Short

It’s a **self-running theatrical train simulator**:

* Realistic motion physics
* Signal coordination
* Visual dashboard
* Station logic
* Named rail lines
* Dynamic pacing

It behaves like a miniature dispatch system — not just a motor controller.

### Stuff You Need

Total Parts: $30-$70 + Bring Your Own Train Set

Parts:
Arduino Nano V3.0 (about ~$10)
https://www.amazon.com/dp/B0713XK923

QRE1113 Analog Infrared Reflective Sensor Module (about $1.25)
https://www.aliexpress.us/item/3256809752874660.html

DRV8871 Motor Controller (about $1.50)
https://www.amazon.com/dp/B0DGFFGLF1

SSH1106 OLED LCD Display (about $6.50)
https://www.amazon.com/dp/B01N1LZT8L

ULN2003 LED Controller (about ~$3–$7)
https://www.amazon.com/dp/B07P5C2KWX

Toy LED Traffic light (around $20–$40)
https://www.amazon.com/dp/B098H3923S


# TrainWreck
TrainWreck is a Nano-based control app to run a model Z guage train into the ground and off the rails.

This app is a **fully scripted model train control system** with display, signals, smooth motion, and themed routes.

### Core Features

**1. Smooth S-Curve Acceleration & Braking**

* Custom ramping (non-linear “bell curve” step size).
* Controlled ramp up / ramp down.
* No abrupt jumps in speed.
* MPH conversion from PWM (0–72 MPH scale).

**2. Direction Control**

* Forward / Reverse switching.
* Direction shown on screen.
* Alternates automatically in route loops.

**3. Traffic Light Integration**

* 🔴 Red = stopped / hault
* 🟢 Green = acceleration, fast leg
* 🟡 Yellow = decelerating, slow leg

**4. OLED Dashboard (128x64 SH1106)**
Displays:

* Route name (top)
* Large speed readout (MPH)
* Direction (FORWARD / REVERSE / HALTED)
* Status line (RAMP TO, DOWN TO, FAST LEG, etc.)

Clean, centered layout with inverted speed panel.

**5. Dip Behavior (Speed Drops While Running)**

* Periodic slow segments during runs.
* Simulates grades, curves, or realism.
* Adjustable dip count and duration.

**6. Station Docking Logic (IR Sensor Ready)**

* IR sensor trigger.
* Timed crawl hold before full stop.
* Prevents hard stops at station.

**7. Scripted Route Library**
Each route defines:

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

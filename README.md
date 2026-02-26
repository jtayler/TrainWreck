# RailStation — Control Tower  
### Powered by RailOS™

> **A luxury smart power controller for any model train.**  
> RailStation replaces a “dumb” power supply with a **control tower** with smooth motion physics, giant display and automatic configuration for your train and track to land for station docking with choreographed station light signals and lighting.  
> **Easy to enjoy.** Grab the knob anytime for manual override—let go and it glides back into automation. The large display changes with a click to show the list of Routes and you can scroll and pipck another Route, or click and hold a long-press to adjust settings and save them.

---

## What it does (in one minute)

- **Auto-calibrates to your layout** so station finding literally “just works.”
- Runs **scripted routes** with realistic pacing, raping speed and a human feel.
- Drives a **real traffic light** (top-and-center) + **station lights** with choreography.
- Supports **Z to G scale** by respecting a configurable power ceiling.
- The built-in series of fun Lines will run for **2–3 hours continuously** and will repeat in an endless loop.

---

## Signature features

### Smooth motion you can feel
RailOS uses non-linear ramping so starts/stops feel intentional—no lurch, no toy throttle vibe.

### Large, luxurious display
A large, elegant display turns your train into the centerpiece — with a beautifully simple interface that makes control effortless.

### Station docking that actually lands
A TINY IR sensor is used for automatic station arrival behavior: approach → crawl → stop → dwell → depart.

### Real signals, not decoration
**Traffic Light**
- **🔴 Red**: Halted / at-station
- **🟡 Yellow**: Decelerating / slow leg
- **🟢 Green**: Accelerating / fast leg

### Station lights with choreography
Four “posts” are programmable as a group:
- arrival blink patterns
- alternating pairs
- departure urgency
- slow fade-out to dark

Plus one extra output for “anything”: gate motor, windmill, sign, beacon, etc.

### Manual override that feels natural
Turn the knob → you’re in control instantly.  
Release it → after a brief pause, RailOS blends you back into the active route smoothly.

---

## Route Library (examples)
A RailStation isn’t just speed—it’s **story**. Each route has a personality:
- Pennsylvania Line
- California Zephyr
- The Orient Express
- Reading Railroad
- Flying Scotsman
- Hudson River Ltd
- 20th Century Ltd
- Taking Pelham 123  
…and more.

Routes are **editable, reorderable, deletable** and your changes **stick**.

---

## The “Human Element” controls (simple, but deep)

RailOS keeps it easy: you pick a line, and a few “human” traits shape the feel.

### Schedule (when are we running?)
- **HIGH_FREQ** — subway-like: short waits, quick turnarounds, more movement  
- **PEAK** — purposeful: brisk legs, tighter dwell times, fewer “lingers”  
- **OFF_PEAK** — relaxed: longer dwells, calmer pacing, more breathing room  

### Equipment (what kind of train is this?)
- **BULLET** — confident acceleration, longer fast legs, shorter crawls  
- **SHUTTLE** — frequent stops, shorter legs, snappy station work  
- **FREIGHT** — heavier feel: longer ramps, longer slow legs, deliberate departures  

### Service (how predictable is it?)
- **NONSTOP** — fewer station events, more continuous running  
- **LIMITED** — selective stops, “express” pacing and timing  
- **UNPREDICTABLE** — controlled variation: small changes that feel human, not random  

### Range (what kind of run is this?)
- **LOCAL** — close stations, frequent choreography, short dwell cycles  
- **SHORT_RUN** — medium legs, balanced station time  
- **LONG_HAUL** — longer cruising legs, fewer but more “cinematic” station moments  

These traits automatically influence:
- station dwell time
- leg length and speed targets
- ramp aggressiveness
- lighting choreography timing
- controlled variation (the “human element”)

---

## Scales & power
RailStation works across scales by capping output with **MAX_SPEED**:
- tiny trains (T/Z) behave safely
- large trains (O/G) still feel powerful
- same controller, different ceiling

---

## RailOS 1.0
RailOS is the C++ firmware that runs RailStation Control Tower:
- motion physics + ramp engine
- station sensor logic
- profile persistence (edits stick)
- signal + station light choreography
- continuous route runner
- manual override blending

---

## Our Motto
**Easy to enjoy.**  
**Your train. Our brain.**

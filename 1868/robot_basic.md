# Robot Basics

## Is CAN bus fast enough for industrial robots?

It depends on where the control loops run and how fast you need to update. Classic CAN is fine for slower or distributed setups. It's marginal-to-insufficient for a multi-axis arm with centralized high-rate control. That's why most industrial robots use EtherCAT or another real-time Ethernet.

**The numbers:**
- **Classic CAN** runs at 1 Mbit/s and carries 8 bytes per frame. A frame is about 110–130 bits with stuffing, so the bus tops out around 7–8k frames/s. Keep it to about 60–70% load for deterministic latency, which leaves about 5k usable frames/s.
- **A 6-axis arm at 1 kHz** needs about 1 command and 1 feedback frame per axis per cycle. That's 12k frames/s, more than classic CAN can carry. At 250–500 Hz it fits, barely.
- **CAN FD** runs at 2–8 Mbit/s in the data phase and carries 64 bytes per frame. You can pack every axis into one or two frames, so 1 kHz for 6 axes becomes feasible.
- **EtherCAT** runs at 100 Mbit/s. Cycle times of 125 µs–1 ms are normal, and clock-sync jitter is under 1 µs, which is the industrial-robot standard.

**When CAN is enough:**
- The current and velocity loops (typically 10–20 kHz) are closed inside each servo drive, and the bus only carries position or trajectory setpoints at about 100–500 Hz.
- There are few axes: grippers, AGVs/AMRs, mobile bases, small arms. Many cobot-style actuators run CANopen (CiA 402) or a vendor CAN protocol at 1 Mbit/s.
- Cable runs are short. At 1 Mbit/s, CAN is limited to about 40 m.

**When it isn't:**
- 6 or more axes needing 1 kHz or faster coordinated updates, as in force/impedance control or high-speed path tracking.
- You need tight multi-axis sync. CANopen SYNC works, but its jitter is far worse than EtherCAT's distributed clocks.
- You also need high-bandwidth data on the same bus, such as F/T sensors, encoders streamed to the controller, or diagnostics.

**Rule of thumb:** use CAN FD for small or cost-sensitive arms, classic CAN for slow or peripheral devices, and EtherCAT for a production industrial robot.

### CAN FD in detail

**Why there's a separate "data phase":** each CAN FD frame uses two bit rates.
1. **Arbitration phase (slow, typically 500 kbit/s–1 Mbit/s).** This covers the ID and control bits. Several nodes may transmit at once, and the highest-priority ID wins bit by bit. For that to work, every bit must travel to the far end of the cable and back before the next bit starts. Cable propagation delay therefore caps the speed, giving the same ~1 Mbit/s, ~40 m limit as classic CAN.
2. **Data phase (fast, 2–8 Mbit/s).** Once arbitration is over, only one node is transmitting, so the round-trip constraint is gone. The sender switches to a faster bit rate (signaled by the BRS bit) for the payload and CRC, then drops back to the slow rate for the ACK and end of frame.

So the 2–8 Mbit/s figure applies only to the payload part of the frame. The header always goes at the slow rate.

**Why 64 bytes matters more than the speed:**
- Classic CAN spends about 50–60 bits of overhead to carry 8 bytes, so only ~50% of the bits on the wire are data.
- CAN FD carries up to 64 bytes for about the same fixed overhead, so ~80–90% of the bits are data.
- Allowed payload lengths are 0–8, 12, 16, 20, 24, 32, 48 or 64 bytes.

**The 6-axis, 1 kHz example:**
- About 8 bytes per axis per cycle (position or torque setpoint plus status). 6 axes × 8 B = 48 B, which fits in **one** CAN FD frame.
- At 1 Mbit/s arbitration and 5 Mbit/s data, a 64-byte frame takes about 30 µs + 110 µs ≈ **140 µs**.
- With one command frame and one feedback frame per axis, that's 12 short frames, roughly 12 × ~60 µs ≈ 0.7 ms of every 1 ms cycle. That works, but it's tight.
- Packing all commands into one broadcast frame cuts the load a lot. Feedback usually still needs one frame per axis.
- Classic CAN would need 12 frames of ~125 µs each = 1.5 ms per 1 ms cycle, so it can't keep up.

**Practical catches:**
- **8 Mbit/s is hard to reach.** It needs short stubs, good termination and FD-rated transceivers. 2–5 Mbit/s is what most robots actually run.
- **Every node must support FD.** A classic-CAN-only node treats FD frames as errors and disrupts the bus.
- **The reverse is fine:** an FD controller can still send and receive classic frames.
- **Stronger CRC** (17 or 21 bits instead of 15), so the longer payload doesn't weaken error detection.

## Cobot-style actuator

A self-contained robot joint module. The motor, gearbox, sensors, brake and driver electronics are all packaged into one unit, and the arm is built by bolting these joints together.

**What's typically inside:**
- **Frameless BLDC motor** built directly into the housing.
- **Reducer**: usually a strain-wave (harmonic) gear, 50:1–160:1, for zero backlash in a compact size. Quasi-direct-drive versions use a low-ratio planetary gear (6:1–10:1) for backdrivability.
- **Dual encoders**: one on the motor side for commutation and one on the output side for absolute joint position. The difference between them also helps with torque or deflection estimation.
- **Integrated servo drive**: runs the current, velocity and position loops on the joint itself.
- **Safety brake**: holds position when power is off.
- **Optional joint torque sensor**: used for collision detection and force control.
- **Hollow shaft**: cables pass through the joint, so you just daisy-chain power plus a bus (CAN/CAN FD, EtherCAT or RS-485) from one joint to the next.

**How it differs from a traditional industrial robot joint:**

| | Cobot-style actuator | Traditional industrial |
|---|---|---|
| Drive electronics | Inside the joint | In a separate control cabinet |
| Wiring | One bus + power, daisy-chained | Thick motor/encoder cable per axis |
| Priority | Compact, light, safe around people, backdrivable | Speed, payload, rigidity |
| Bus | Often CAN/CAN FD or EtherCAT | EtherCAT or proprietary |

Because each joint closes its own fast loops, the bus only carries setpoints and feedback at a few hundred Hz to 1 kHz. That's why CAN is workable for these joints.

> [!NOTE]
> **What is a joint torque sensor?**
>
> A sensor that measures the actual torque passing through a robot joint, usually right at the gearbox output. That tells the robot how hard the joint is pushing or being pushed. This is what lets cobots feel collisions and be guided by hand.
>
> **Why not just use motor current?** Motor current gives the torque at the motor. Between the motor and the link sits the gearbox, which adds:
> - Friction, especially in harmonic drives. It changes with speed, temperature, load and wear.
> - Efficiency loss of about 60–85%, plus hysteresis.
>
> So current-based estimates of external force are noisy and inaccurate, particularly for small forces. A sensor after the gearbox sees the real joint torque directly.
>
> **Where it sits:**
> ```
> Motor → Gearbox → [Torque sensor] → Output flange → next link
> ```
>
> **Common sensor types:**
>
> | Type | How it works | Notes |
> |---|---|---|
> | **Strain gauge on a flexure** | A spoked or hub-shaped metal element twists slightly; bonded strain gauges in a bridge measure the strain | Most common. Accurate, mature. Needs amplifier electronics and temperature compensation. |
> | **Deflection-based (two encoders)** | A deliberately compliant element sits between two encoders; torque = stiffness × measured twist angle | Used in series elastic actuators. Reuses the joint's encoders. |
> | **Capacitive / optical** | Measure the tiny twist of the flexure by capacitance or light | Compact, less common. |
> | **Magnetostrictive** | Torque changes the magnetic properties of the shaft | Non-contact, more common in automotive than robots. |
>
> **What it enables:**
> - **Collision detection:** sense unexpected contact quickly and stop. This is key for power and force limiting, the safety mode cobots use under ISO/TS 15066.
> - **Hand guiding / lead-through teaching:** push the arm and it follows.
> - **Force and impedance control:** controlled pressing, polishing, assembly, insertion.
> - **Gravity and friction compensation:** better torque control and smoother motion.
> - **Payload identification:** estimate the mass of whatever the robot is holding.
>
> **Trade-offs:**
> - **Lower stiffness:** the flexure must twist to be measured, which adds compliance and slightly reduces precision and bandwidth.
> - **Cost and complexity:** one sensor plus electronics per joint.
> - **Error sources:** temperature drift, and crosstalk from bending forces that aren't torque. The flexure design has to reject those.
>
> **Compared with a wrist F/T sensor:** a 6-axis force/torque sensor at the flange measures forces at the tool only. Joint torque sensors cover the whole arm, so they can detect a collision on the elbow too.
>
> **Examples:** the KUKA LBR iiwa and Franka Emika arms have a torque sensor in every joint. Many other cobots, like Universal Robots, estimate joint torque from motor current and add a wrist F/T sensor for precise force tasks.

> [!NOTE]
> **What is a flange?**
>
> A flat, ring- or disc-shaped plate with bolt holes, used to bolt two parts together. On a robot the word usually means the **tool flange**: the mounting plate at the end of the last joint where you attach grippers, tools or sensors.
>
> **Flanges on a robot:**
> - **Tool flange / mounting flange:** the end of the arm, on the last wrist axis (J6 on a 6-axis arm). The end effector bolts on here.
> - **Joint output flange:** the rotating plate at a gearbox or actuator output that the next link bolts to. "Output flange" in the diagram above means this.
> - **Base flange:** where the robot base bolts to the floor, a pedestal or a mobile platform.
>
> **Tool flange details:**
> - **Standard bolt pattern:** most robots follow **ISO 9409-1**, so tools are interchangeable across brands. A code like `ISO 9409-1-50-4-M6` means a 50 mm pitch circle with 4 × M6 bolts.
> - **Locating features:** a centering bore or spigot plus a dowel-pin hole, so the tool always mounts in exactly the same position and orientation.
> - **Electrical/pneumatic connections:** often a tool connector nearby, or passed through a hollow wrist, for power, I/O and air.
> - **Common stack:** flange → (adapter plate) → (F/T sensor) → (tool changer) → gripper/tool.
>
> **Flange frame vs. TCP:**
> - **Flange frame (tool0):** a coordinate frame fixed at the center of the flange face. It's defined by the robot's kinematics.
> - **TCP (Tool Center Point):** the point that actually does the work, like a gripper's fingertips or a welding torch tip. It's set as an offset from the flange frame. You program motions for the TCP, and the controller works out where the flange must go.

## RS-485

An electrical standard for serial communication over a twisted pair using differential signaling. It defines only the physical layer (voltages, wiring, drivers). The protocol on top is up to you, commonly Modbus RTU, Profibus DP or a vendor-specific one.

**Key traits:**
- **Differential pair (A/B):** the signal is the voltage difference between two wires, so it's highly resistant to electrical noise. That's why it's popular in factories.
- **Multi-drop:** up to 32 standard nodes on one bus (more with low-load transceivers), daisy-chained with 120 Ω termination at each end.
- **Speed vs. distance:** about 10 Mbit/s at short runs (~10 m), down to about 100 kbit/s at 1200 m.
- **Usually half-duplex:** one node talks at a time, typically a single controller polling devices. A 4-wire variant allows full-duplex.
- **Cheap and simple:** basically a UART plus a transceiver chip.

**Compared to CAN:**

| | RS-485 | CAN |
|---|---|---|
| Layer | Physical only | Physical + data link |
| Arbitration | None; the controller must schedule who talks | Built-in, collision-free, priority-based |
| Error handling | Whatever your protocol adds | CRC, ACK, auto-retransmit, fault confinement |
| Speed | Up to ~10+ Mbit/s | 1 Mbit/s (CAN FD up to ~8) |
| Effort | You build the protocol | Robust out of the box |

**In robotics:** RS-485 is common in servo drives such as Dynamixel servos and many Modbus-based drives. It can beat classic CAN on raw speed if you run a tight controller-polled scheme. You give up CAN's built-in arbitration and error handling, so reliability depends on your protocol design.

> [!NOTE]
> **What is a transceiver?**
>
> A **trans**mitter plus a re**ceiver** in one device. In bus communication (RS-485, CAN) it's a small chip that translates between a microcontroller's logic-level signals and the electrical signals on the bus wires. It handles only the physical signaling and has no knowledge of the protocol.
>
> **Where it sits:**
> ```
> MCU / controller ──TX/RX (3.3 V or 5 V logic, single-ended)──► Transceiver ──A/B or CANH/CANL (differential)──► Bus wires
> ```
> - **Transmit:** turns the MCU's TX logic signal into a differential voltage on the pair.
> - **Receive:** reads the voltage difference on the pair and outputs a clean logic signal on RX.
>
> **Protocol vs. physical layer:**
>
> | Layer | RS-485 system | CAN system |
> |---|---|---|
> | Protocol logic | UART in the MCU + your software (e.g. Modbus) | CAN controller (built into most MCUs): framing, arbitration, CRC, retransmit |
> | Physical signaling | RS-485 transceiver | CAN transceiver |
>
> **Example chips:**
> - **RS-485:** MAX485, TI SN65HVD7x, isolated ADM2587E
> - **CAN / CAN FD:** NXP TJA1051 / TJA1462, Microchip MCP2551 / MCP2562FD, TI TCAN1042
>
> **Features that matter:**
> - **Driver enable (RS-485):** DE/RE pins switch the chip between talking and listening on a half-duplex bus. Your firmware must toggle them at the right time. CAN transceivers don't need this.
> - **Bus protection:** ESD and surge protection and short-circuit tolerance for factory wiring.
> - **Galvanic isolation:** isolated transceivers break ground loops between devices on different power supplies. They're common in industrial and motor-drive systems, where ground noise is severe.
> - **Fail-safe receiver:** outputs a defined idle state when the bus is open, shorted or idle.
> - **Slew-rate control:** slower signal edges reduce electromagnetic noise on long cables.
> - **Speed rating:** must match the bus, e.g. a CAN FD transceiver for 5–8 Mbit/s.
>
> **Analogy:** in Ethernet the equivalent part is called a PHY. EtherCAT uses one per port too.

## Servo drive

The power electronics plus controller that runs a servo motor. It takes a command (position, velocity or torque) from a higher-level controller, reads the motor's feedback, and switches current into the motor windings so the motor does exactly what was commanded.

**What it does:**
- **Power stage:** an inverter of MOSFETs or IGBTs, PWM-switched at about 10–20 kHz, converts DC bus power into three-phase motor current.
- **Commutation:** uses the encoder or rotor angle to energize the right windings at the right time. Field-oriented control (FOC) is the usual method.
- **Cascaded control loops,** from inner to outer:
  - Current/torque loop at about 10–20 kHz
  - Velocity loop at about 2–10 kHz
  - Position loop at about 1–5 kHz
- **Feedback processing:** reads encoders or resolvers, phase currents and temperature.
- **Communication:** receives setpoints over a bus (EtherCAT, CANopen, RS-485/Modbus) or older interfaces (step/direction pulses, ±10 V analog).
- **Protection and safety:** overcurrent, overvoltage, overtemperature, following-error limits, brake control, and Safe Torque Off (STO).

**Where it sits:**
```
Motion controller ──bus──► Servo drive ──3-phase power──► Motor
  (trajectory, 1 kHz)       (loops, 10+ kHz)  ◄──encoder────┘
```

**Physical form:**
- **Traditional industrial:** separate boxes in a control cabinet, one per axis or multi-axis units.
- **Integrated:** a small PCB built into the motor or joint, as in cobot-style actuators.

The drive closes the fast loops locally, so the bus only has to carry slower setpoints. That's why CAN or RS-485 can be enough in many systems.

> [!NOTE]
> **What is STO (Safe Torque Off)?**
>
> A safety function built into servo drives, defined in **IEC 61800-5-2**, that guarantees the motor can't produce torque. It does this by blocking the switching signals to the drive's power transistors, so no current can reach the motor windings, even though the drive itself stays powered.
>
> **How it works**
> ```
> Safety controller / E-stop ──24 V──► STO1 input ─┐
>                            ──24 V──► STO2 input ─┴─► drive's gate driver for the power transistors
>                                                     (voltage removed → transistors can't switch → no motor current)
> ```
> - **Two independent channels (STO1, STO2):** if either one drops, torque is off. That redundancy is what lets it reach high safety ratings, up to **SIL 3 / PL e**.
> - **Fail-safe:** STO is active when the 24 V inputs are **removed**. A broken wire or power loss therefore makes the drive safe, the same principle as spring-applied brakes and contactors.
> - **Diagnostic feedback:** many drives have a status output that confirms STO is active. The safety controller monitors it.
>
> **What STO does and doesn't do**
>
> | Does | Doesn't |
> |---|---|
> | Removes motor torque within ms | Brake the motor: a moving axis **coasts** to a stop |
> | Prevents unexpected startup | Hold position: vertical axes **fall** without a brake |
> | Keeps the DC bus charged, so restart is fast | Electrically isolate anything: dangerous voltages remain inside the drive and motor cables |
>
> So STO is paired with:
> - **Spring-applied brakes**, for axes affected by gravity. The **Safe Brake Control (SBC)** function controls them safely.
> - **Lockout/tagout and a main disconnect** for electrical maintenance. STO is not a replacement for isolation.
>
> **STO vs. main contactors**
>
> | | STO | Main contactors |
> |---|---|---|
> | Method | Electronic: blocks transistor switching | Mechanical: opens power contacts |
> | Speed | ms | Tens of ms |
> | Wear | None | Contacts wear with each switching cycle |
> | Restart | Instant; the DC bus stays charged | Needs DC bus pre-charge |
> | Isolation | No | Yes, from mains (but the DC bus can stay charged for a while) |
>
> That's why modern robots use STO for frequent safety stops and keep contactors, where present, for full power removal (see "Main contactors" below).
>
> **Related drive safety functions (also IEC 61800-5-2)**
> - **STO (Safe Torque Off):** torque removed immediately. This is stop category 0.
> - **SS1 (Safe Stop 1):** controlled deceleration, then STO. This is stop category 1, the usual E-stop behavior for robots.
> - **SS2 (Safe Stop 2):** controlled deceleration, then the drive holds position with torque on, monitored by **SOS** (Safe Operating Stop). This is stop category 2.
> - **SLS (Safely Limited Speed):** monitors that speed stays below a limit, e.g. in collaborative or teach mode.
> - **SBC (Safe Brake Control):** safe control of the holding brake.

## Commutation

Switching current between a motor's windings in step with the rotor's position, so the magnetic field keeps pulling the rotor forward. Without it, the rotor would turn until it lined up with the field and then stop.

**Why it's needed:** a motor makes torque when the stator's magnetic field is out of line with the rotor's magnets, ideally by 90°. As the rotor turns, the current has to keep moving to different windings to keep that angle.

**Two ways to do it:**

| | Mechanical | Electronic |
|---|---|---|
| Used in | Brushed DC motors | BLDC / PMSM servo motors |
| How | Brushes rub on a segmented commutator that physically flips the current | The drive switches transistors based on the rotor angle it senses |
| Downsides | Brush wear, sparking, speed limits | Needs a rotor position sensor and a controller |

**Electronic commutation methods:**
- **Trapezoidal (six-step):** uses 3 Hall sensors and energizes 2 of the 3 phases at a time in 6 steps per electrical cycle. It's simple and cheap but causes torque ripple.
- **Sinusoidal:** uses an encoder to drive smooth sine-wave currents in all 3 phases. It's smoother.
- **Field-oriented control (FOC):** the standard in servo drives. It converts the phase currents into a frame that rotates with the rotor and controls the torque-producing current directly, giving smooth, efficient torque at all speeds.
- **Sensorless:** estimates the rotor angle from back-EMF instead of a sensor. It's common in drones and fans but weak at very low speed, so servos rarely use it.

**Robotics angle:** the motor-side encoder in a joint or servo motor exists mainly for commutation. A wrong angle offset ("commutation alignment") causes weak torque, heat or a runaway motor. That's why drives run a phasing routine at setup or startup.

## Rotor position sensors in industrial robotics

Industrial robot motors mostly use **absolute optical encoders**. **Resolvers** cover harsh or high-temperature uses, and **magnetic encoders** are common in cobots and lower-cost joints. Hall sensors alone are rare except in cheap drives.

| Sensor | How it works | Typical resolution | Pros | Cons | Where used |
|---|---|---|---|---|---|
| **Absolute optical encoder** | LED shines through a coded glass or metal disc onto a photodetector array | 17–24+ bit/rev | Very high accuracy and resolution, low noise | Sensitive to shock, contamination, heat; higher cost | Most industrial arm servo motors (FANUC, Yaskawa, ABB, KUKA motors) |
| **Resolver** | Rotary transformer: rotor angle modulates AC signals in two windings (sin/cos) | Effectively 12–16 bit via an R/D converter | Extremely rugged: tolerates heat, vibration, oil, radiation; no electronics on the motor | Lower accuracy, needs excitation and conversion electronics, bulkier | Harsh environments, automotive/EV, some older robots, aerospace |
| **Magnetic encoder** | Hall or magnetoresistive chip reads a magnet (on-axis) or a magnetized ring (off-axis) | 12–21 bit | Cheap, compact, tolerates dust and oil | Less accurate than optical; affected by stray fields and temperature | Cobots, integrated joint modules, output-side joint encoders |
| **Inductive / capacitive encoder** | PCB coils or electrodes sense a patterned target | 16–22 bit | Rugged like a resolver with digital output; immune to magnetic fields | Less common, fewer suppliers | Growing use in robot joints and cobots |
| **Hall sensors (3×)** | Detect magnet poles, 6 states per electrical cycle | Only about 60° electrical | Very cheap | Commutation only, no fine position | Low-end BLDC drives, sometimes as a startup backup |

**Multi-turn tracking** matters because the robot must know its position at power-up without re-homing. Encoders keep counting turns in one of two ways:
- **Battery-backed:** a small battery keeps the turn counter alive while power is off. This is classic on industrial arms, which is why robots lose calibration when the encoder battery dies.
- **Battery-less:** Wiegand-wire energy harvesting or mechanical gear stages. This approach is increasingly common.

**Digital interfaces:** modern encoders send position as serial data rather than analog pulses. Common protocols:
- **Open:** BiSS-C, EnDat 2.2 (Heidenhain), HIPERFACE DSL (SICK), SSI.
- **Vendor-proprietary:** Tamagawa, Nikon, and the robot makers' own serial encoder protocols. Industrial robot OEMs often use these.

**Typical joint setup:** a high-resolution encoder on the motor is used for commutation and the velocity loop. Cobots add a second absolute encoder on the gearbox output for true joint position and torque estimation. Traditional industrial arms usually rely on the motor encoder alone and calculate joint position through the gear ratio.

## Homing

The routine (also called referencing) that tells the controller where an axis actually is. It moves the axis to a known reference point and sets the position counter to a known value there. It's only needed when the position sensor can't report absolute position at power-up.

**When it's needed:**
- **Incremental encoders and stepper motors:** these only count changes in position, so at power-up the controller doesn't know where the axis is. It has to home every time it starts.
- **Absolute encoders:** these report position immediately, so no homing is needed at startup. That's why industrial arms use them. You only redo the reference ("mastering" or "calibration") after the encoder battery dies, a motor or encoder is replaced, or a crash shifts something.

**Common homing methods:**

| Method | How | Accuracy |
|---|---|---|
| **Home/limit switch** | Drive slowly until a switch triggers, often backing off and re-approaching at slower speed | Moderate, limited by switch repeatability |
| **Switch + encoder index pulse** | Find the switch, then latch on the encoder's once-per-rev index (Z) pulse | High, the standard on CNC machines |
| **Hard stop** | Drive into a mechanical stop and detect the current or torque spike | Moderate; simple and needs no sensor, common for grippers and cheap actuators |
| **Index pulse only** | Rotate to the next index pulse | Only works if it's already known which revolution the axis is in |
| **Manual / fixture mastering** | Align the joint to marks, a pin or a dial gauge, then set zero | Used for robot calibration after service |

**Robot-specific terms:**
- **Mastering / zero calibration:** recording the encoder value at each joint's mechanical zero. FANUC calls it mastering, KUKA uses a gauge called an EMD for it, and ABB calls it revolution counter update.
- **Home position:** often also means a safe "park" pose the robot moves to before starting work. That's a taught pose, not a reference calibration, though people use the same word for both.

## Power supply for industrial robots

Industrial robots run on three-phase AC mains, usually 200–480 VAC at 50/60 Hz. The control cabinet turns that into a high-voltage DC bus for the motors and 24 VDC for logic, I/O, brakes and safety. Cobots usually run from single-phase AC or a 48 VDC supply instead.

**Power chain in a typical industrial robot controller:**
```
3-phase AC mains (200–480 V)
  → main disconnect + fuses/breakers
  → (optional transformer, if mains voltage ≠ robot's rated voltage)
  → main contactors (opened by E-stop / safety circuit)
  → rectifier → DC bus (~300 VDC from 230 VAC, ~560–680 VDC from 400–480 VAC)
       → servo amplifiers (inverters) → motors
       → braking resistor or regenerative converter (dumps or returns energy when motors decelerate)
  → 24 VDC power supply → controller CPU, I/O, teach pendant, motor brakes, safety circuits
```

**Main parts:**

| Part | Purpose |
|---|---|
| **Three-phase input** | Main power for the motors. Common ratings: 200–230 V class (Japan, some FANUC/Yaskawa models) and 380–480 V class (EU/US). |
| **Transformer** | Matches local mains to the robot's rated voltage, e.g. a 200 V-class robot in a 480 V plant. |
| **Main contactors** | Cut motor power when E-stop is pressed or a safety fault occurs. Usually redundant (two in series) for safety ratings. |
| **DC bus + capacitors** | Shared energy reservoir for all axes. Energy from one decelerating axis can feed another axis that's accelerating. |
| **Braking resistor / regen unit** | Absorbs energy fed back when heavy axes slow down or lower a load. A regen unit sends it back to the grid to save energy. |
| **24 VDC supply** | Powers logic, sensors, I/O, safety relays, and the motor brakes, which are spring-applied and released by 24 V. |
| **Backup power** | Capacitors or a small UPS let the controller save data and stop cleanly during a power loss. A separate encoder battery keeps absolute position. |

**Power ratings (rough):**
- Small arm (≤10 kg payload): about 1–3 kVA
- Medium arm (20–70 kg): about 3–8 kVA
- Large arm (150–500+ kg): about 10–25+ kVA

Peak draw during fast multi-axis moves can be well above the average, so supply circuits and breakers are sized for those peaks.

**Cobots and mobile robots:**
- **Cobots:** usually 100–240 VAC single-phase into a control box that outputs about 48 VDC to the joints. The lower voltage is simpler and safer around people.
- **Mobile manipulators and AMRs:** battery-powered, typically 24–48 VDC, sometimes higher.

**Safety:** E-stop and protective stops typically use Safe Torque Off (STO) in the drives and/or open the main contactors. Brakes engage automatically when power is cut, since they're spring-applied.

## Spring-applied brake

The brake is held closed by springs, and electricity is needed to open it. It's also called a power-off, fail-safe or normally-engaged brake.

**How it works:**
```
Power OFF:  springs push the armature plate against the friction disc → disc clamped → shaft locked
Power ON:   24 V energizes an electromagnet coil → pulls the armature away against the springs → shaft free
```

**Why robots use it:** fail-safe behavior. If power is lost, a cable is cut or E-stop opens the circuit, the coil loses power and the springs clamp the brake automatically. The arm can't drop under gravity. A power-applied brake (needs power to engage) would release exactly when you need it most.

**Practical points:**
- **Holding brake, not a service brake:** it's meant to hold a stopped axis. Normal stopping is done by the servo drive. The brake is rated for a limited number of emergency stops while moving, and those wear the friction lining.
- **Release sequence:** the controller enables motor torque first, then releases the brake. On stop it engages the brake first, then removes torque. This prevents the arm from sagging.
- **Manual brake release:** robots have a button or tool to release brakes without servo power, for freeing a trapped person or moving the arm after a crash. Heavy axes will fall when released, so support the arm first.
- **Brake test:** many controllers periodically test that each brake can still hold torque, as part of safety functions.

## Main contactors

Heavy-duty, electrically controlled switches that connect or cut the high-voltage power feeding the robot's motor drives. A contactor is basically a large relay built to switch high current safely.

**How a contactor works:**
```
Coil energized (usually 24 VDC from the safety circuit) → electromagnet pulls the contacts closed → power flows to the drives
Coil de-energized → a return spring snaps the contacts open → motor power is cut
```
Like spring-applied brakes, it's fail-safe: losing control power opens it.

**Contactor vs. ordinary relay:**

| | Relay | Contactor |
|---|---|---|
| Switches | Small signals, low current | Motor or main power, tens to hundreds of amps |
| Poles | Usually 1–2 | Usually 3, one per phase, plus auxiliary contacts |
| Arc handling | Minimal | Arc chutes and spaced contacts to break high current safely |

**Role in a robot controller:**
- They sit between the mains or rectifier and the servo amplifiers, so the safety system can physically cut motor power.
- **Opened by the safety circuit:** an E-stop, opened safety fence door, enabling switch released on the teach pendant, or a safety fault drops the coil and the contacts open.
- **Redundant:** typically two contactors in series, so if one fails to open, the other still cuts power.
- **Monitored:** each has mirror contacts, auxiliary contacts mechanically linked to the main ones. The safety controller checks them to catch a welded (stuck-closed) contactor and refuses to restart if one is stuck.
- **Pre-charge:** a resistor or small contactor often charges the DC bus capacitors slowly at power-up before the main contactor closes, to avoid a big inrush current.

**Stop categories (IEC 60204-1):**
- **Stop 0:** cut power immediately by opening the contactors or STO. Brakes clamp and the motors coast.
- **Stop 1:** the drive brakes the motion in a controlled stop, then power is cut. This is gentler on the mechanics and the most common for E-stop on modern robots.
- **Stop 2:** controlled stop with power kept on. This is used for protective stops, where the robot holds position.

Modern drives with Safe Torque Off (STO) can remove motor torque electronically, so some designs rely on STO and use contactors less. Many robots still have both.

## Spring-applied brakes vs. main contactors

They are two halves of the same safe stop. The main contactors remove the motor's power, so the motor can no longer push. The spring-applied brakes hold the axis mechanically, so gravity can't move it once the motor stops holding. Both default to the safe state when their 24 V control power is lost, and the same safety system controls both.

**Why you need both:**

| Only... | Problem |
|---|---|
| Contactors open, no brake | Motor has no torque, so vertical axes and heavy loads fall under gravity |
| Brake engaged, contactors closed | The drive could still drive the motor against the brake if a fault occurs, and the brake isn't meant to fight motor torque |
| **Both** | No energy to the motor, and the axis is mechanically locked |

**They're on separate circuits:**
- Contactors switch the high-voltage motor power, 200–480 VAC or the DC bus.
- Brakes run on 24 VDC through their own relay or drive output.
- The safety controller and drive coordinate the two, so their timing can be controlled independently.

**Sequencing:**

Power-up / servo on:
```
1. Close main contactors (after DC bus pre-charge)
2. Drive enables → motor produces holding torque
3. Release brakes (energize coils)
4. Robot can move
```

Normal or Stop 1 stop:
```
1. Drive decelerates the robot to standstill
2. Engage brakes (cut coil power)
3. Short delay so the brake fully clamps
4. Remove torque / open main contactors
```

Stop 0 (immediate power cut, e.g. some E-stops or a power failure):
```
Contactors open and brake power drops at the same time → brakes clamp while the axis is still moving
```
This works, but the moving axis wears the brake lining. That's why brakes are rated for a limited number of these stops, and why Stop 1 is preferred when possible.

**Key timing rule:** the brakes must be engaged before motor torque goes away, and released only after torque is present. Otherwise the arm sags or drops for a moment. That gap is what the sequencing prevents.

## Adding a new sensor to a robotics system

Adding a sensor touches seven steps: requirements, mechanical, electrical, communication, software, calibration, and validation. How deep each one goes depends on the sensor and whether your system is industrial (PLC or robot controller) or ROS-based.

**1. Define requirements first**
- What it measures, with the range, resolution, accuracy and update rate you need.
- Latency budget: does a control loop use it (needs low and deterministic latency) or only monitoring or perception?
- Environment: temperature, vibration, IP rating, EMI near motor drives.
- **Safety role:** if the sensor protects people (light curtain, safety scanner, safety mat), it must be a safety-rated device wired into the safety controller. Never use a standard sensor for a safety function.

**2. Mechanical integration**
- A rigid, repeatable mount: dowel pins or locating features, so a replaced sensor lands in the same spot.
- Cable routing through the arm or along dress packs, with strain relief and enough bend radius for joint motion.
- Check that payload, inertia and reach still fit the robot's limits if it's mounted on the arm.

**3. Electrical integration**
- **Power:** usually 24 VDC in industrial systems. Check current draw and fusing, and whether it needs its own isolated supply.
- **Signal type:** digital I/O, analog (4–20 mA preferred over 0–10 V for noise immunity), or a digital bus.
- **Noise:** shielded cable, shield grounded at one end (or as the manufacturer says), separated from motor power cables, isolated interface if grounds differ.

**4. Communication**
Pick the interface that matches the data rate and timing:

| Interface | Typical use |
|---|---|
| Discrete I/O | Proximity switches, part-present sensors |
| IO-Link | Smart industrial sensors with config and diagnostics |
| RS-485 / Modbus RTU | Slow, simple devices |
| CAN / CANopen | Embedded and actuator-level sensors |
| EtherCAT | Control-loop sensors (F/T, encoders), deterministic timing |
| Ethernet / USB / GigE | Cameras, LiDAR, high-bandwidth perception |

Also plan **time sync**: hardware triggers, PTP (IEEE 1588), or EtherCAT distributed clocks. Sensor data is only useful if you know exactly when it was measured.

> [!NOTE]
> **Is Camera Link a type of IO-Link?**
>
> No. Camera Link and IO-Link are unrelated standards that happen to share the word "Link." They sit at opposite ends of the bandwidth range.
>
> | | Camera Link | IO-Link |
> |---|---|---|
> | Purpose | Stream images from machine-vision cameras | Connect and configure simple industrial sensors and actuators |
> | Standard | AIA/A3 (2000) | IEC 61131-9 |
> | Speed | About 2 Gbit/s (Base) to about 6.8 Gbit/s (Deca) | 4.8 / 38.4 / 230.4 kbit/s |
> | Wiring | Thick, dedicated multi-pair cables, up to about 10 m | Standard unshielded 3-wire M12 sensor cable, up to 20 m |
> | Connects to | A frame grabber card in a PC | An IO-Link master port, which connects up to a fieldbus or PLC |
> | Data | Raw image pixels, trigger lines, a serial channel for camera settings | Process values (distance, temperature, …), parameters, diagnostics |
> | Topology | Point-to-point | Point-to-point (not a bus) |
>
> **Camera Link today:** it's a legacy standard, still used where very low, fixed latency matters. New machine-vision systems mostly use:
> - **GigE Vision:** standard Ethernet, long cables, no frame grabber.
> - **USB3 Vision:** simple and cheap, short cables.
> - **CoaXPress:** very high speed over coax, needs a frame grabber.
> - **Camera Link HS:** Camera Link's higher-speed successor.

> [!NOTE]
> **Is MIPI a type of IO-Link?**
>
> No. MIPI is also unrelated to IO-Link. In robotics, "MIPI" usually means **MIPI CSI-2**, a board-level, chip-to-chip interface that connects a camera image sensor directly to a processor. It's the camera link used in phones, Raspberry Pi and NVIDIA Jetson boards.
>
> | | MIPI CSI-2 | IO-Link |
> |---|---|---|
> | Purpose | Stream raw images from an image sensor into a processor | Connect and configure simple industrial sensors and actuators |
> | Organization | MIPI Alliance (originally mobile phones) | IEC 61131-9 |
> | Speed | About 1.5–4.5 Gbit/s per lane (D-PHY), 1–4 lanes | 4.8 / 38.4 / 230.4 kbit/s |
> | Distance | Centimeters: a flex cable or PCB trace, typically under about 30 cm | Up to 20 m of standard sensor cable |
> | Connects to | The SoC's camera input (ISP) | An IO-Link master port |
> | Configuration | A separate I²C link sets the sensor's registers | Built into the protocol |
>
> **Key points for robotics:**
> - **Cheap, low-latency, low-power embedded vision:** camera modules plug straight into a Jetson or another SoC, with no frame grabber or network stack.
> - **Driver dependency:** each sensor (e.g. Sony IMX series) needs a kernel driver and ISP tuning for the specific board. Support varies a lot by board vendor.
> - **Short reach:** for cameras farther away on a robot or vehicle, the CSI-2 signal is carried over coax by serializer/deserializer chips:
>   - **GMSL** (Analog Devices / Maxim) and **FPD-Link III/IV** (TI): about 10–15 m, common on AMRs and autonomous vehicles.
>   - **MIPI A-PHY:** MIPI's own long-reach automotive standard.
>
> **Where it fits among camera interfaces:**
> - **MIPI CSI-2:** embedded, inside the robot.
> - **GigE Vision / USB3 Vision:** PC-based, off-the-shelf industrial cameras.
> - **CoaXPress / Camera Link:** high-end, frame-grabber-based machine vision.

**5. Software integration**
- **Driver:** a vendor SDK, an existing ROS 2 driver, or a fieldbus device description file (ESI for EtherCAT, EDS for CANopen, IODD for IO-Link).
- **Data model:** units, coordinate frame and timestamps.
- **ROS 2 specifics:**
  - Publish standard message types (`sensor_msgs/Imu`, `PointCloud2`, `geometry_msgs/WrenchStamped`, …).
  - Add the sensor's frame to the URDF / TF tree.
  - Pick a QoS profile that matches the data: sensor-data QoS for high-rate streams.
- **Industrial controller:** map the signals into the controller's or PLC's I/O and write the program logic.
- **Fault handling:** detect timeouts, out-of-range values and disconnects, and define what the robot does when they happen.

**6. Calibration**
- **Intrinsic:** the sensor's own parameters, e.g. camera lens distortion, IMU bias, F/T zero offset.
- **Extrinsic:** the sensor's exact pose relative to the robot (flange, base or TCP), e.g. hand-eye calibration for a camera.
- Plan how to redo both after a crash or a replacement.

**7. Validate**
- Bench-test the sensor alone, then test it on the robot at low speed.
- Check the real update rate, latency and noise against your requirements.
- Test failure cases: unplug it, block it, overload it. Confirm the system reacts safely.
- Document the wiring, configuration and calibration values.

## Syncing sensors

Sensor sync means solving two problems. You need **a shared clock**, so both sensors' timestamps mean the same thing. If the sensors must measure at the same instant, you also need **simultaneous capture**. Decide which you need early, because some methods require specific hardware features and wiring.

**1. Decide how tight the sync must be**

| Use case | Needed accuracy |
|---|---|
| Stereo cameras, multi-camera 3D | Under 1 ms, ideally µs; same exposure moment |
| Camera + IMU (visual-inertial odometry) | Under about 1 ms, with a known, constant offset |
| Camera + LiDAR fusion | About 1 ms, or ms-level plus motion compensation |
| F/T sensor + joint data in a control loop | µs-level, deterministic |
| Monitoring and logging | Tens of ms are fine |

**2. Pick the sync method (best to worst)**

| Method | How it works | Typical accuracy | Needs |
|---|---|---|---|
| **Hardware trigger** | One controller or one sensor sends a pulse that starts capture on both | µs | Trigger-in/out pins, extra wiring |
| **Fieldbus clock** (EtherCAT distributed clocks) | The bus keeps every device's clock aligned and timestamps samples | Under 1 µs | Both sensors on the same EtherCAT network |
| **PTP (IEEE 1588 / gPTP)** | Network protocol aligns device clocks over Ethernet | Under 1 µs with hardware timestamping | PTP-capable NICs, switches and sensors |
| **GPS + PPS** | GPS time plus a pulse-per-second signal disciplines each clock | Under 1 µs | GPS receiver; common for LiDAR on outdoor robots |
| **NTP** | Network time sync in software | About 1–10 ms | Nothing special |
| **Arrival-time stamping** | Host stamps data when it arrives | Poor: ms of jitter plus unknown latency | Last resort only |

**Practical combination:** use a hardware trigger for simultaneous capture plus PTP or PPS for a common clock. Example: LiDAR on PTP, with the camera triggered when the LiDAR beam sweeps through the camera's field of view.

**3. Timestamp correctly**
- Use the sensor's **hardware timestamp**, taken at the actual measurement, not the time the host received the data.
- **Cameras:** timestamp the middle of the exposure. Use global-shutter cameras for fast motion, because with rolling shutter each row is captured at a slightly different time.
- **LiDAR:** each point in one scan has its own time. Use per-point timestamps to correct for motion distortion ("deskewing").
- Measure and compensate any fixed latency, such as sensor processing or transfer delay.

**4. Align the data in software**
- **ROS 2:** `message_filters` with an **ExactTime** policy for triggered sensors (identical timestamps) or **ApproximateTime** for free-running ones (nearest match).
- **Different rates** (e.g. IMU at 400 Hz, camera at 30 Hz): interpolate the fast sensor's data to the slow sensor's timestamps.
- Avoid simply taking "the latest message" from each sensor. That causes hidden time errors.

**5. Calibrate and verify the time offset**
- **Estimate the remaining offset:** tools like **Kalibr** (camera + IMU) estimate it along with the spatial calibration.
- **Measure the real sync:** point the sensors at something time-varying, like a blinking LED, a fast-rotating target or a sharp tap, and compare when each sensor detects it.
- **Monitor in operation:** log the clock offset and trigger status, and flag drift or missing triggers.

**Rule of thumb:** if the sensors support triggering or PTP, use them. Software-only timestamps are rarely good enough for fusion on a moving robot.

### Hardware trigger in detail

With a hardware trigger, a single electrical pulse on a dedicated wire tells every connected sensor to capture at once. Because the signal is a direct electrical edge, with no network or software in between, the sensors start within microseconds of each other.

**1. Architectures**
```
(a) External trigger source (most flexible)
    MCU / FPGA / PLC / sync box ──pulse──┬──► Camera A trigger-in
                                         ├──► Camera B trigger-in
                                         └──► IMU sync-in

(b) One sensor leads, others follow
    Camera A (strobe/sync-out) ──pulse──► Camera B trigger-in

(c) Sensor-driven
    IMU or LiDAR sync-out ──pulse──► Camera trigger-in
```
- **(a)** gives you full control over rate, delays and timestamps. It's the best choice for 3 or more sensors.
- **(b)** needs no extra hardware, but the follower always lags slightly behind the leader's actual exposure. That lag is fixed and measurable.
- **(c)** fits sensors that can't be triggered themselves, like a spinning LiDAR or a free-running IMU.

**2. Electrical details**
- **Signal levels:** check what each input expects.
  - Logic-level: 3.3 V or 5 V TTL/CMOS, typical for embedded boards.
  - Industrial: 24 V PLC-level, often opto-isolated.
  - Mismatches need a level shifter or driver.
- **Opto-isolated inputs:** common on industrial cameras. They protect against ground loops but add some delay (µs), which is usually constant.
- **Edge and pulse width:** configure rising or falling edge, and keep the pulse longer than the minimum the sensor specifies.
- **Fan-out:** for several sensors, use a buffer or driver chip rather than one MCU pin driving everything.
- **Long cables or noisy environments:** use shielded twisted pair, or differential signaling (RS-422 drivers) for runs longer than a few meters or near motor drives. Noise can create false triggers.
- **Grounding:** share a ground reference, or use isolated inputs.

**3. Camera configuration (GenICam names, used by most industrial cameras)**

| Setting | Typical value | Meaning |
|---|---|---|
| `TriggerSelector` | `FrameStart` | What the trigger starts |
| `TriggerMode` | `On` | Wait for a trigger instead of free-running |
| `TriggerSource` | `Line0` | Which physical input pin |
| `TriggerActivation` | `RisingEdge` | Which edge fires |
| `TriggerDelay` | e.g. `0`–`x` µs | Extra delay after the edge, for alignment |
| `LineDebouncer` | a few µs | Ignore short glitches |
| `ExposureMode` | `Timed` | Exposure length set in software (alternative: `TriggerWidth` = pulse length sets exposure) |

Many cameras also have an **ExposureActive / strobe output**. Use it to flash lighting during exposure, trigger another sensor, or check timing with an oscilloscope.

**4. Timing details that matter**
- **Align exposure midpoints, not start times.** If camera A exposes for 10 ms and camera B for 4 ms, delay B by (10 − 4) / 2 = 3 ms so both are centered on the same instant. Better still, use equal or fixed exposures, since auto-exposure breaks alignment.
- **Global vs. rolling shutter:**
  - Global shutter: the whole image is exposed at once, so the trigger moment is the image moment.
  - Rolling shutter: rows are exposed one after another, so timing differs across the frame. Prefer global shutter for synced or moving setups.
- **Maximum trigger rate:** each frame needs exposure time plus readout time. Triggers that arrive too fast are ignored ("overtrigger") and frames are silently missed.
- **Latency vs. jitter:** a constant delay (opto-isolator, sensor internals) is fine because you can measure and subtract it. Varying delay (jitter) is what ruins sync. Hardware triggers keep jitter at the µs level.
- **A software trigger is not a hardware trigger.** A trigger command over USB or Ethernet has ms-level, variable latency.

**5. Timestamps: triggering alone isn't enough**
A trigger makes the captures simultaneous, but it doesn't tell you *when* they happened in system time. Add timestamps:
- Have the trigger source record a precise time for each pulse, ideally from a PTP- or GPS-synced clock. An MCU timer capture works well for this.
- Pair each pulse with a **frame counter** or sequence number from each sensor, then match frames by ID rather than by arrival time.
- Watch the counters: if one sensor drops a frame, IDs show the mismatch instead of silently pairing the wrong frames.

**6. Common sensor pairings**
- **Stereo or multi-camera:** an external source triggers all cameras with identical exposure settings.
- **Camera + IMU:** the IMU's sync-out triggers the camera every Nth IMU sample, or the trigger source drives both. The camera frames then line up exactly with known IMU samples.
- **Camera + spinning LiDAR:** a spinning LiDAR can't be triggered per frame. Instead:
  1. Phase-lock the LiDAR's rotation to a PPS signal. Many LiDARs support this.
  2. Trigger the camera at the moment the laser sweeps through the camera's field of view.
  3. LiDAR and image then see the same scene at the same moment.

**7. Pitfalls**
- Noise on the trigger line, causing false or double triggers.
- Mismatched or auto-adjusting exposure between sensors.
- Overtriggering, with silent frame drops.
- Sensors left in free-run mode after a reboot or config reset. Save the trigger settings to the sensor's startup configuration.
- Assuming triggered means timestamped.

**8. Verifying it works**
- **Oscilloscope:** probe the trigger line and each sensor's ExposureActive output to measure the actual delay and jitter.
- **Visual test:** film a fast LED counter or a rotating target. Synced cameras should show identical states.
- **Long runs:** compare trigger counts and frame counts over hours to catch drops.

> [!NOTE]
> **Using an MCU to trigger multiple sensors**
>
> An MCU (microcontroller) trigger board uses hardware timers to generate trigger pulses for several sensors with µs to sub-µs precision. It also records exactly when each trigger fired and sends those timestamps to the host. It's architecture **(a)** from "Architectures" above, and the most flexible way to sync cameras, IMUs and other triggerable sensors.
>
> **1. Why an MCU**
> - **Deterministic timing:** hardware timers produce edges with ns-level jitter, unaffected by OS scheduling. A Linux PC toggling a GPIO can't match that.
> - **Flexible:** different rates per sensor, phase offsets, pulse widths, all from one clock source, so they never drift relative to each other.
> - **Timestamps:** the MCU knows exactly when it fired each trigger.
> - **Cheap:** a $10–30 board.
>
> **2. Architecture**
> ```
>                  Time reference (optional): GPS PPS, or PPS out from a PTP-synced NIC
>                                   │ (input capture)
>                                   ▼
>                         ┌──────────────────┐ ──trigger CH1──► Camera A
>                         │       MCU        │ ──trigger CH2──► Camera B
>                         │ hardware timers  │ ──trigger CH3──► IMU sync-in
>                         └──────────────────┘ ◄─ExposureActive (optional, input capture)
>                                   │
>                        USB / UART / Ethernet: (sequence number, timestamp) per trigger
>                                   ▼
>                               Host PC (driver node matches frames to trigger times)
> ```
>
> **3. Choosing an MCU**
>
> Needs:
> - Hardware timers with several **output compare / PWM channels**, plus **input capture** for PPS and exposure feedback.
> - A **crystal or TCXO clock**, not the internal RC oscillator. RC oscillators can be off by about 1% and drift with temperature.
>
> | Option | Notes |
> |---|---|
> | **STM32** (F4/G4/H7) | Rich, linkable timers; H7/F7 have IEEE 1588 Ethernet |
> | **Teensy 4.x** (NXP i.MX RT1062) | Fast, many timers, Arduino-friendly; 4.1 has Ethernet |
> | **RP2040 / RP2350** | PIO blocks generate exact pulse patterns independent of the CPU |
> | **SAMD21** (Arduino Zero) | Used by the open-source **VersaVIS** sync board (ETH Zurich ASL) |
> | **ESP32** | Works with hardware timers, but Wi-Fi interrupts add jitter to anything done in software |
>
> **Rule:** edges must come from **timer hardware** (output compare, PWM, PIO), never from code toggling a pin in a loop or interrupt. Code-driven edges inherit interrupt latency jitter.
>
> **4. Generating the triggers**
> - **Same rate for all sensors:** one timer with several compare channels gives edges at the same instant. Alternatively, drive one output into a fan-out buffer.
> - **Different rates, still aligned:** derive every rate from one base timer. Example: IMU sync at 200 Hz on every tick, camera at 20 Hz on every 10th tick. Linked timers (one timer clocking another) or counting in the timer interrupt handles this. Since every rate comes from the same clock, they never drift apart.
> - **Phase offsets:** set a different compare value per channel:
>   - Align **exposure midpoints** by delaying the shorter-exposure camera (see "Timing details that matter" above).
>   - Trigger cameras at the moment a spinning LiDAR sweeps through their field of view.
> - **Pulse width:** set by the compare value in PWM mode. Some cameras use the pulse width as the exposure time (`TriggerWidth` mode).
>
> **5. Timestamping each trigger**
> - **Record the timer count at each trigger event** in the compare interrupt. Since the edge was produced by hardware at a known compare value, the timestamp is exact, even if the interrupt runs a few µs later.
> - **Extend the counter to 64 bits:** a 32-bit counter at 1 MHz wraps in about 71 minutes. Count overflows in software and combine.
> - **Send (sensor ID, sequence number, timestamp) to the host:** queue them in a ring buffer and send from the main loop. Never compute timestamps from when the host receives the message.
> - **Optional exposure feedback:** capture each camera's ExposureActive output on an input-capture pin. That gives the actual exposure time of every frame and catches missed triggers.
>
> Firmware sketch (structure only; register details depend on the MCU):
> ```c
> // Timer runs at 1 MHz from the crystal (1 tick = 1 µs).
> // Compare channels drive trigger pins in hardware; the ISR only records timestamps.
> volatile uint32_t overflow_count = 0;
>
> void TIMER_IRQHandler(void) {
>     if (overflow_flag())  { clear_overflow(); overflow_count++; }
>     if (cam_compare_flag()) {
>         clear_cam_compare();
>         uint64_t t = ((uint64_t)overflow_count << 32) | cam_compare_value();
>         ring_push(SENSOR_CAM, cam_seq++, t);
>         set_next_cam_compare(cam_compare_value() + CAM_PERIOD_TICKS);
>     }
>     // ... same for IMU channel ...
> }
>
> int main(void) {
>     setup_timer_and_outputs();
>     while (1) { send_queued_timestamps_to_host(); handle_host_commands(); }
> }
> ```
> (Handle the case where an overflow and a compare happen in the same interrupt, or use a timer that's already 64-bit or chained.)
>
> **6. Mapping MCU time to host time**
> MCU timestamps are in MCU ticks. To use them on the host, relate them to system time:
>
> | Method | How | Accuracy |
> |---|---|---|
> | **PPS discipline** | Input-capture a GPS PPS (or a PTP NIC's PPS output). Measure the MCU ticks per second to correct its frequency, and align the trigger train to the PPS edge. Label the second from NMEA or the host. | Tens of ns to ~1 µs |
> | **PPS from a PTP-synced NIC** | E.g. an i210 SDP pin outputs PPS from its PTP clock, so MCU time is tied to PTP time | Tens of ns |
> | **PTP on the MCU** | MCUs with IEEE 1588-capable Ethernet (e.g. STM32H7, NXP i.MX RT) run a PTP stack directly | Under 1 µs |
> | **Host round-trip sync over USB/serial** | NTP-like ping-pong exchange; filter offset and drift with a linear fit | About 10–100 µs, limited by USB jitter |
>
> Without any of these, MCU timestamps are still precise *relative to each other*, but their link to host time is only as good as your estimate.
>
> **7. Host-side software**
> - A driver node reads (seq, timestamp) messages from the MCU and matches them to sensor frames.
> - **Match by sequence number:** use the camera's frame counter if it has one. Arm all sensors in trigger mode **before** the MCU starts triggering, so trigger #0 ↔ frame #0.
> - **Without frame counters:** match each frame to the nearest trigger time within a window. That only works if the trigger period is much longer than the arrival-time jitter.
> - **Set message timestamps** to trigger time + fixed delay + half the exposure (the exposure midpoint).
> - **Detect drops:** compare trigger counts and frame counts. Re-sync or flag a mismatch instead of silently pairing wrong frames.
>
> **8. Electrical output stage**
> MCU pins are 3.3 V with low current, so you usually need a driver stage:
> - **Logic-level inputs (3.3/5 V):** a buffer or level-shifter chip per output, with a small series resistor (about 22–100 Ω) to damp ringing.
> - **Opto-isolated or 24 V industrial inputs:** these need current, often 5–15 mA, or a higher voltage. Use a transistor or MOSFET stage, or an industrial output driver.
> - **Long cables or noisy environments:** RS-422 differential line drivers and shielded twisted pair.
> - **Protection:** ESD diodes on connectors, and pull-down resistors so outputs stay low while the MCU is resetting. Without them, floating pins can fire spurious triggers.
> - **Grounding:** a common ground, or isolated outputs where grounds differ.
> - **Propagation delay:** each stage adds a fixed delay (ns to µs). Use identical stages per channel, or measure and compensate.
>
> **9. Pitfalls**
> - **Code-toggled pins:** jitter from interrupts and other tasks.
> - **Internal RC oscillator:** a large frequency error and temperature drift distort timestamps and rates.
> - **Counter overflow handling:** off-by-one-wrap bugs create timestamps that jump by about 71 minutes (32-bit at 1 MHz).
> - **Interrupt priority:** a low-priority timestamp interrupt can be delayed so long that it misses the next event. Give the timer interrupt high priority.
> - **Sensors not armed before triggering starts:** sequence numbers get offset by one or more.
> - **Overtriggering:** trigger rate faster than the sensor's exposure + readout. Frames silently drop.
> - **Changing camera settings while triggering:** many cameras drop or delay frames during configuration changes. Stop triggers, change settings, restart.
> - **Spurious edges at power-up or reset:** missing pull-downs.
>
> **10. Verifying it works**
> - **Oscilloscope:** probe every trigger output and ExposureActive line to check rates, phase offsets and jitter.
> - **Count check:** compare MCU trigger counts and sensor frame counts over a long run.
> - **PPS drift check:** log MCU ticks per PPS second over hours to measure oscillator drift and confirm disciplining works.
> - **End-to-end:** a blinking LED or fast-rotating target captured by all sensors, comparing the resulting timestamps.

> [!NOTE]
> **MCU trigger vs. host GPIO**
>
> If "GPIO" means toggling a host computer's GPIO pins (Jetson, Raspberry Pi or an industrial PC) to trigger sensors, the choice comes down to **timing precision vs. simplicity**. An MCU's hardware timers give ns-level, deterministic edges, but they need an extra board and a time mapping. Host GPIO needs no extra hardware and its timestamps are already in system time, but the OS adds µs-to-ms jitter.
>
> **Three options compared**
>
> | | Host GPIO, software toggle | Host hardware PWM (SoC peripheral) | MCU hardware timer |
> |---|---|---|---|
> | **How** | A program writes the pin high/low (e.g. `libgpiod`) | The SoC's PWM block generates pulses after it's configured | The MCU timer generates pulses in hardware |
> | **Edge jitter** | About 10 µs to over 1 ms on standard Linux; about 10–50 µs with PREEMPT_RT | ns | ns |
> | **Under CPU load** | Gets worse; edges can be late or skipped | Unaffected once running | Unaffected |
> | **Timestamps** | Taken around the pin write, already in system time; error ≈ the jitter | Edges not timestamped individually; you infer them from start time + period, and the PWM clock can drift from system time | Exact, but in MCU time; needs mapping to host time |
> | **Multiple rates / phase offsets** | Easy in code, but jittery | Limited channels; alignment between channels varies by SoC | Flexible and exact |
> | **If the host process crashes** | Triggers stop | Usually continue | Continue |
> | **Extra hardware** | None | None | MCU board + firmware + link to host |
> | **Electrical** | Fragile 3.3 V SoC pins, still needs a buffer | Same | Same, but you design the output stage |
>
> **What jitter actually affects**
>
> If **one pin fans out to all sensors**, they still capture at exactly the same instant, even with a jittery host GPIO, because they share one edge. Jitter affects two other things:
> 1. **Timestamp accuracy:** how well you know *when* the shared edge happened.
> 2. **Period regularity:** uneven frame intervals. That matters for some algorithms, like visual-inertial odometry assuming fixed rates.
>
> So host GPIO can be fine for **simultaneous capture** but poor for **precise absolute timing**.
>
> **Ways to improve host-side triggering**
> - **Hardware PWM + edge timestamping:** use the SoC's PWM to generate triggers, and timestamp the actual edges in hardware. On Jetson Orin, the Generic Timestamping Engine (GTE), exposed through Linux's HTE subsystem, can timestamp GPIO edges in system time. Elsewhere, wire the edge back to an input with kernel interrupt timestamps (µs-level).
> - **DMA-timed output:** on Raspberry Pi, `pigpio`'s DMA-timed waveforms give about µs-level timing without CPU involvement.
> - **PREEMPT_RT + high-priority thread:** brings software-toggle jitter down to tens of µs. That's still not ns-level.
>
> **MCU: timer output vs. code-toggled GPIO**
>
> The same rule applies inside an MCU:
> - **Timer output compare / PWM / PIO:** hardware edges, ns jitter. Use this.
> - **Code toggling a pin** in a loop or interrupt: jitter from interrupt latency and other code, about 0.1–several µs. That's better than Linux, but avoid it for triggers.
>
> **When to use which**
>
> | Situation | Recommendation |
> |---|---|
> | Prototype, low rate (≤ 30 Hz), ms-level timing is OK, one pin feeding all sensors | Host GPIO |
> | SBC with hardware PWM and edge timestamping (e.g. Jetson GTE) | Host hardware PWM, a good middle ground |
> | Visual-inertial odometry, stereo, camera-LiDAR fusion, several rates or phase offsets | MCU hardware timer |
> | Needs to keep running regardless of host load or crashes | MCU |
> | Needs absolute time across devices | MCU disciplined by PPS or PTP (see the MCU note above) |
>
> **Rule of thumb:** host GPIO is fine for getting simultaneous captures cheaply. Use an MCU when you also need exact, regular timing and precise timestamps.

> [!NOTE]
> **How to programmatically toggle a GPIO**
>
> On Linux, use **libgpiod**, the standard character-device interface (`/dev/gpiochipN`), or a board library built on it. On an MCU, write the pin through the SDK or HAL. For trigger signals, though, use the hardware PWM or timer peripheral rather than toggling in code (see the "MCU trigger vs. host GPIO" note above).
>
> **1. Linux: find the pin first**
> ```bash
> gpiodetect            # list GPIO chips
> gpioinfo gpiochip0    # list lines (offsets) on a chip, their names and usage
> ```
> - The **line offset** is not the header pin number. Check your board's pinout to map them.
> - On Jetson, the pin must also be set to GPIO mode in the pinmux (`sudo /opt/nvidia/jetson-io/jetson-io.py`).
> - **Permissions:** add your user to the `gpio` group, or add a udev rule. Otherwise you need `sudo`.
>
> **2. Linux: command line (libgpiod tools)**
> ```bash
> # libgpiod v2 (newer distros, e.g. Ubuntu 24.04+, Raspberry Pi OS Bookworm)
> gpioset -c gpiochip0 17=1        # set line 17 high (holds it while running; Ctrl+C to release)
> gpioset -c gpiochip0 17=0
>
> # libgpiod v1 (older distros)
> gpioset gpiochip0 17=1
> ```
> Check your version with `gpioset --version`; the syntax differs between v1 and v2.
>
> **3. Linux: Python (libgpiod v2 bindings, `pip install gpiod`)**
> ```python
> import time
> import gpiod
> from gpiod.line import Direction, Value
>
> LINE = 17
> with gpiod.request_lines(
>     "/dev/gpiochip0",
>     consumer="trigger",
>     config={LINE: gpiod.LineSettings(direction=Direction.OUTPUT,
>                                      output_value=Value.INACTIVE)},
> ) as req:
>     while True:
>         t = time.clock_gettime_ns(time.CLOCK_REALTIME)   # software timestamp of the edge
>         req.set_value(LINE, Value.ACTIVE)
>         time.sleep(0.001)                                # 1 ms pulse
>         req.set_value(LINE, Value.INACTIVE)
>         time.sleep(0.099)                                # ~10 Hz (drifts and jitters; see below)
> ```
>
> **Board-specific libraries:**
> - **Raspberry Pi:** `gpiozero` (simple), or `lgpio`. Note that `RPi.GPIO` doesn't work on the Pi 5.
>   ```python
>   from gpiozero import DigitalOutputDevice
>   pin = DigitalOutputDevice(17)      # BCM numbering
>   pin.on(); pin.off()
>   pin.blink(on_time=0.001, off_time=0.099)   # background blinking, still software-timed
>   ```
> - **Jetson:** `Jetson.GPIO`, which has an RPi.GPIO-style API with BOARD/BCM pin numbering.
> - **Avoid** the old sysfs interface (`/sys/class/gpio`). It's deprecated and disabled on many current kernels.
>
> **4. Linux: hardware PWM (much better for triggers)**
> The SoC's PWM block generates the pulses, so the CPU and OS don't affect their timing:
> ```bash
> cd /sys/class/pwm/pwmchip0
> echo 0 > export                              # creates pwm0/
> echo 100000000 > pwm0/period                 # period in ns: 100 ms = 10 Hz
> echo 1000000   > pwm0/duty_cycle             # pulse width in ns: 1 ms
> echo 1 > pwm0/enable
> ```
> - **Raspberry Pi:** enable it first with `dtoverlay=pwm-2chan` (or `pwm`) in `config.txt`.
> - **Jetson:** configure the pin for PWM in `jetson-io.py`.
> - The chip and channel numbers vary by board.
>
> **5. Linux: reducing software-toggle jitter**
> For timing-sensitive loops:
> - Write the loop in **C/C++**, not Python.
> - Sleep until **absolute deadlines** with `clock_nanosleep(..., TIMER_ABSTIME, ...)`, so the period doesn't drift.
> - Run with real-time priority (`chrt -f 80 ./trigger`) on a **PREEMPT_RT** kernel. Optionally pin to an isolated CPU core (`isolcpus`, `taskset`).
> - Call `mlockall()` to avoid page-fault delays.
>
> Even with all of this, expect tens of µs of jitter. Use hardware PWM or an MCU when you need better.
>
> **6. MCU: simple toggle**
> ```c
> // Arduino
> digitalWrite(2, HIGH);  digitalWrite(2, LOW);
>
> // STM32 HAL (or write the BSRR register directly for the fastest toggle)
> HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
> HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
>
> // Raspberry Pi Pico SDK
> gpio_init(2); gpio_set_dir(2, GPIO_OUT);
> gpio_put(2, 1);  gpio_put(2, 0);
> ```
>
> **7. MCU: hardware PWM trigger (preferred)**
> Example for a Raspberry Pi Pico (RP2040 at its default 125 MHz system clock), giving 10 Hz with a 1 ms pulse:
> ```c
> #include "pico/stdlib.h"
> #include "hardware/pwm.h"
>
> int main(void) {
>     const uint PIN = 2;
>     gpio_set_function(PIN, GPIO_FUNC_PWM);
>     uint slice = pwm_gpio_to_slice_num(PIN);
>     pwm_set_clkdiv(slice, 250.0f);          // 125 MHz / 250 = 500 kHz → 2 µs per tick
>     pwm_set_wrap(slice, 49999);             // 50,000 ticks = 100 ms period → 10 Hz
>     pwm_set_gpio_level(PIN, 500);           // 500 ticks = 1 ms high pulse
>     pwm_set_enabled(slice, true);
>     while (true) { tight_loop_contents(); } // CPU is free; the PWM runs in hardware
> }
> ```
> The RP2350 defaults to 150 MHz, so adjust the divider. On STM32, the equivalent is a timer in PWM mode (`HAL_TIM_PWM_Start`), configured in CubeMX.
>
> **8. Electrical cautions**
> - SoC and MCU GPIO pins are **3.3 V** and supply only a few mA. **Never apply 5 V** to them unless the datasheet says the pin tolerates it.
> - Drive sensors through a **buffer or transistor stage**, not directly, especially opto-isolated or 24 V inputs. See "Electrical output stage" in the MCU note above.

> [!NOTE]
> **Is trigger GPIO toggling usually done in the driver?**
>
> Usually not by toggling a pin in software. In a finished system the trigger pulses come from **hardware**: an MCU timer, SoC PWM, a camera's sync output, or a GMSL deserializer's frame-sync. The **drivers' job is to configure that hardware and handle the timestamps**. Code that directly toggles GPIOs for triggering is mostly found in prototypes.
>
> **Who does what, by layer**
>
> | Layer | Role in triggering |
> |---|---|
> | **Kernel GPIO/PWM controller driver** | Always present. Exposes pins as `/dev/gpiochipN` and PWM as `/sys/class/pwm`. You don't write this; it comes with the board support package. |
> | **Kernel sensor driver** (e.g. a MIPI camera driver on Jetson) | Uses GPIOs defined in the device tree for reset, power-down, and sometimes a frame-sync (FSYNC) pin. For multi-camera MIPI/GMSL setups, frame sync is usually set up here or in the deserializer, not in user code. |
> | **MCU firmware** (trigger board) | Generates trigger pulses with hardware timers and records their timestamps. |
> | **User-space sensor driver / ROS node** (e.g. a GenICam camera driver) | Puts the sensor in external-trigger mode through its SDK, then receives frames and sets their timestamps. |
> | **Trigger / sync node** | Configures hardware PWM or talks to the MCU, publishes trigger timestamps, and matches them to frames. Sometimes this logic is built into the camera driver node. |
> | **Application** | Consumes already-synced, correctly timestamped data. It shouldn't toggle pins. |
>
> **Typical patterns**
> - **MCU trigger board:** the firmware generates the pulses, and a ROS driver node reads the (sequence, timestamp) messages from the MCU and re-stamps camera frames with them. The open-source VersaVIS board (ETH Zurich ASL) works this way.
> - **Industrial cameras:** the camera driver (e.g. a Spinnaker or Pylon ROS node) only sets `TriggerMode` and `TriggerSource`. The pulses come from an MCU, a PLC, or another camera's strobe output.
> - **Embedded MIPI cameras:** sync comes from sensor FSYNC pins or a GMSL/FPD-Link deserializer, configured by the kernel driver and device tree.
> - **Prototype:** a small user-space script or node toggles a GPIO with libgpiod. That's fine for testing, but should be replaced by hardware PWM or an MCU before you rely on the timing.
>
> **Writing your own kernel driver** is rarely needed for triggering. Hardware PWM plus hardware edge timestamping (e.g. Jetson GTE) gives better timing than kernel-space toggling anyway. A kernel driver is only worth considering if you need interrupt-level handling that isn't otherwise available.
>
> **Recommendation:** put all trigger logic in **one dedicated component**, the MCU firmware or one trigger node. Have it publish trigger timestamps, and keep sensor drivers and applications unaware of the GPIO details.

### Fieldbus clock in detail

With a fieldbus clock, the industrial network itself keeps every device's internal clock aligned. Each device then samples or acts at a scheduled time on that shared clock, so the moment the data happens to travel over the bus doesn't matter. **EtherCAT distributed clocks (DC)** is the most common example, with typical sync accuracy well under 1 µs. Other fieldbuses have similar mechanisms, covered at the end.

**1. How EtherCAT distributed clocks work**

Each DC-capable device (SubDevice) has a 64-bit nanosecond clock inside its EtherCAT controller chip (ESC). The controller (MainDevice) aligns them in three steps:

```
MainDevice ──► SubDevice 1 (reference clock) ──► SubDevice 2 ──► SubDevice 3 ...
                    │                                │               │
               defines system time        adjusted to match   adjusted to match
```

1. **Propagation delay measurement (at startup):** the MainDevice sends a frame that every device timestamps as it passes through each port. From these timestamps it calculates the cable and processing delay to each device.
2. **Offset compensation (at startup):** the MainDevice writes each device an offset so all of them show the same **system time** (counted in ns since 1 Jan 2000).
3. **Drift compensation (continuous):** each cycle, the MainDevice reads the reference clock's time and writes it to all other devices in the same frame. Each chip's control loop speeds up or slows down its own clock slightly to stay locked.

Result: all device clocks agree to within tens of nanoseconds, typically under 100 ns.

**2. SYNC signals: acting at the same instant**

The shared clock is used to produce hardware pulses inside each device:
- **SYNC0 / SYNC1:** the chip fires a pulse at a programmed system time, then every cycle after that. The device samples inputs or applies outputs exactly on that pulse.
- **Result:** every sensor and drive samples at the same instant, even though the frame reaches device 1 and device 20 at slightly different times.

```
Cycle (e.g. 1 ms):
  |──frame passes through──|          SYNC0 ▼ (all devices, same instant)
  t0                        t0 + shift         → sample inputs / apply outputs
                                                → data read out in the next frame
```

- **Shift time:** SYNC0 is set slightly after the frame passes, so new outputs have arrived before they're applied. Inputs captured at SYNC0 are collected by the next frame, which adds one cycle of known, fixed latency.

**3. Device sync modes**

EtherCAT devices usually support three modes:

| Mode | When the device acts | Sync quality |
|---|---|---|
| **Free Run** | On its own internal timer | None; drifts relative to others |
| **SM-synchronous** | When the frame arrives at that device | Depends on frame jitter and position on the bus |
| **DC-synchronous (SYNC0)** | At the shared, scheduled time | Best: sub-µs across all devices |

Servo drives running cyclic synchronous position (CSP, CiA 402) and sensors used in control loops should use **DC mode**. The device's ESI (XML description) file lists which DC modes it supports, and you select one in the MainDevice configuration.

**4. Bringing non-EtherCAT sensors into the same clock**

Sensors without EtherCAT can still be tied to system time:
- **Timestamped outputs:** a DC-capable digital output terminal (e.g. Beckhoff EL2252) switches at a precise system time. Use it as a **hardware trigger** for a camera, so the trigger is scheduled on the same clock as the joints.
- **Timestamped inputs / latches:** a DC input terminal (e.g. Beckhoff EL1252) or the chip's **LATCH** inputs record the exact system time of an incoming edge, such as a camera's ExposureActive output.
- **Bridging to PTP or GPS:** a gateway (e.g. a PTP/IEEE 1588 terminal like Beckhoff EL6688) or the MainDevice can align EtherCAT system time with a PTP or GPS clock. EtherCAT data then shares a time base with cameras and LiDAR on a normal Ethernet network.

**5. Requirements on the controller side**
- **Real-time MainDevice:** frames must go out on time with low jitter, e.g. TwinCAT, or Linux with PREEMPT_RT plus IgH EtherCAT Master or SOEM. Late frames cause sync errors and drive faults.
- **Align the MainDevice with the bus:** either adjust the controller's cycle timer to the reference clock, or adjust the reference clock to the controller. Pick one; the configuration tool usually handles it.
- **Configuration:** cycle time (commonly 125 µs–1 ms), SYNC0 shift and DC activation per device, all set in the EtherCAT configurator.

**6. Pitfalls**
- **Not every device supports DC:** cheap I/O modules may only do Free Run or SM-synchronous. Check the datasheet or ESI file.
- **Controller jitter:** a non-real-time OS on the MainDevice causes missed cycles. Devices drop out of sync or trigger watchdog errors.
- **Wrong shift time:** if SYNC0 fires before the new data arrives, outputs apply one cycle late or old data gets used.
- **Topology changes:** hot-connecting or reordering devices requires re-running delay measurement.
- **Monitoring:** read each device's clock difference from the reference (available in a status register and in most configurators) to confirm sync is actually holding.

**7. Other fieldbus clock mechanisms**

| Fieldbus | Sync mechanism | Typical accuracy |
|---|---|---|
| **EtherCAT** | Distributed clocks + SYNC0/1 | Under 100 ns |
| **PROFINET IRT** | PTCP clock sync, or IEEE 802.1AS on TSN variants | Under 1 µs |
| **Sercos III** | Cyclic sync telegram from the controller | Under 1 µs |
| **POWERLINK** | Start-of-cycle (SoC) broadcast frame | About 1 µs or less |
| **CANopen** | SYNC broadcast message (plus TIME object for absolute time) | µs to ~100+ µs: if another frame is mid-transmission, SYNC must wait for it to finish |

**When to use it:** fieldbus clocks are ideal when sensors and actuators share one control network, e.g. F/T sensors, encoders and joint drives in a robot arm. Their data then lines up exactly with joint positions and torques. For cameras and LiDAR, combine the fieldbus clock with timestamped trigger outputs or a PTP bridge.

### PTP in detail

PTP (Precision Time Protocol, IEEE 1588) synchronizes device clocks over standard Ethernet. With hardware timestamping and PTP-aware switches it reaches **sub-microsecond accuracy**, often tens of nanoseconds. It's the usual way to give a robot PC, LiDARs and GigE cameras one shared time base.

**1. Roles**

| Role | What it does |
|---|---|
| **Grandmaster** | The single time source for the network. Often a robot PC, a dedicated PTP clock, or a GPS-disciplined clock on outdoor robots. |
| **Ordinary clock (time receiver)** | An end device that syncs to the grandmaster: sensor, PC, controller. |
| **Boundary clock** | A switch that syncs to the grandmaster upstream, then acts as a time source for devices downstream. |
| **Transparent clock** | A switch that measures how long each PTP message waited inside it and adds that to a correction field, so queuing delay doesn't cause errors. |

Recent versions of the standard (IEEE 1588g) use "timeTransmitter / timeReceiver" for the two ends of a sync link. **BMCA** (Best Clock Algorithm) automatically elects the grandmaster by comparing priority settings, clock quality and an ID. Set `priority1` lowest on the device you want as grandmaster.

**2. How a clock is synced: the message exchange**

```
Time transmitter                     Time receiver
     t1 ──── Sync ──────────────────►  t2
        ──── Follow_Up (contains t1) ►        (two-step mode)
     t4 ◄─── Delay_Req ──────────────  t3
        ──── Delay_Resp (contains t4) ►
```

With these four timestamps:
- **Path delay** = ((t2 − t1) + (t4 − t3)) / 2
- **Clock offset** = ((t2 − t1) − (t4 − t3)) / 2

The receiver corrects its clock by the offset. It repeats this continuously, typically 1–16 times per second, and a control loop adjusts its clock rate so it doesn't drift.

**Key assumption:** the delay is the same in both directions. Asymmetry, like different cable or PHY delays per direction, turns directly into offset error.

**Delay mechanisms:**
- **End-to-end (E2E):** the Delay_Req exchange above, across the whole path. Common for industrial sensors.
- **Peer-to-peer (P2P):** each link measures its own delay with Pdelay messages. Used by gPTP/TSN, and it handles topology changes faster.

**3. Hardware timestamping is what makes it precise**

| Setup | Typical accuracy |
|---|---|
| Hardware timestamping + PTP-aware switches | Under 1 µs, often tens of ns |
| Hardware timestamping + ordinary switch, light traffic | About 1–10 µs |
| Software timestamping | About 10–100 µs |
| NTP, for comparison | About 1–10 ms |

- **Hardware timestamping:** the network card (NIC) stamps each PTP packet at the moment it crosses the wire, using its own clock, the **PTP Hardware Clock (PHC)**. Software timestamps are taken later by the OS, with variable delay.
- **Check support on Linux:** `ethtool -T eth0` should list hardware transmit/receive timestamping and a PTP hardware clock (`/dev/ptpN`). Intel i210, i225 and i226 are common, well-supported choices.
- **Switches matter:** an ordinary switch adds variable queuing delay. Use PTP-aware switches (transparent or boundary clock), or connect sensors directly to PTP-capable NIC ports.

**4. Profiles: settings must match across devices**

| Profile | Used in | Notes |
|---|---|---|
| **Default (IEEE 1588v2)** | Most industrial sensors, LiDARs, GigE cameras | Usually E2E, UDP/IPv4 or layer-2 Ethernet |
| **gPTP (IEEE 802.1AS)** | TSN networks, automotive, some newer robots | P2P, layer 2 only, stricter rules |
| **Telecom / power profiles** | Telecom networks, electrical grids | Rare in robotics |

All devices must agree on **domain number, transport (UDPv4 vs L2), delay mechanism (E2E vs P2P)** and sometimes one-step vs two-step. A mismatch is the most common reason a sensor never locks.

**5. Linux setup (linuxptp)**

There are two clocks to sync on a Linux PC:
1. The **NIC's hardware clock (PHC)**, synced over the network by `ptp4l`.
2. The **system clock (CLOCK_REALTIME)**, which ROS and drivers use for timestamps. It's synced to the PHC by `phc2sys`.

```bash
# Robot PC as a time receiver (another device is grandmaster)
sudo ptp4l -i eth0 -s -m                          # -s = receiver only, -m = log to terminal
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m       # copy PHC time into the system clock

# Robot PC as grandmaster for its sensors
sudo ptp4l -i eth0 -m -f /etc/linuxptp/gm.conf     # config with low priority1
sudo phc2sys -s CLOCK_REALTIME -c eth0 -w -m       # copy system time (from GPS/NTP) into the PHC
```

Key `ptp4l.conf` settings:
```ini
[global]
domainNumber        0
priority1           100        # lower = preferred grandmaster (default 128)
network_transport   UDPv4      # or L2 — must match the sensors
delay_mechanism     E2E        # or P2P
time_stamping       hardware
```

Run both as systemd services so they start before the sensor drivers.

**6. Typical robot architecture**
```
GPS/PPS (optional) ──► Robot PC (grandmaster, i210 NIC)
                           │
                    PTP-aware switch (transparent clock)
                     ┌─────┼──────────┐
                  LiDAR   GigE camera  GigE camera
               (receiver)  (receiver)   (receiver)
```
- **LiDARs:** most current models (Ouster, Hesai, newer Velodyne) support PTP and stamp each point with PTP time.
- **GigE Vision cameras:** GigE Vision 2.0 added PTP. They also support **scheduled action commands**: the PC broadcasts "capture at time T," and every camera exposes at T on its PTP clock. That gives near-hardware-trigger sync with no trigger wires, typically at the µs level.
- **ROS:** configure drivers to use the sensor's own timestamp, not arrival time, in message headers. With `phc2sys` running, those timestamps match the PC's system clock.

**7. Pitfalls**
- **TAI vs. UTC:** PTP counts time in TAI, which is currently **37 s** ahead of UTC. Misconfigured devices end up exactly 37 s apart. Check the UTC offset setting on the grandmaster and in `phc2sys`.
- **Only half synced:** sensors are locked to PTP but `phc2sys` isn't running, so ROS timestamps come from an unsynced system clock.
- **Clock fights:** NTP/chrony and `phc2sys` both adjusting the system clock. Disable one, or let chrony use the PHC as its reference.
- **Ordinary switches or Wi-Fi:** variable delays destroy accuracy. PTP over Wi-Fi is generally not usable for precise sync.
- **Several grandmaster candidates:** BMCA can switch between them, causing time jumps. Set priorities explicitly.
- **Multicast filtered:** PTP uses multicast (224.0.1.129 for UDP, 01-1B-19-00-00-00 for L2). VLANs, firewalls or managed switches can block it.
- **Convergence time:** after startup, clocks can take seconds to a minute to settle. Wait for lock before recording or fusing data.
- **Asymmetric paths:** media converters, some PHYs and mixed link speeds add one-way delay that PTP can't see. Calibrate it or avoid it.

**8. Verifying it works**
- **ptp4l logs:** the offset value should settle to tens of ns with hardware timestamping. A steadily growing or jumping offset means trouble.
- **Sensor status:** most LiDARs and cameras report their PTP state ("locked" or "slave/receiver") in their web UI or driver diagnostics.
- **`pmc` tool:** query PTP status and grandmaster identity from the command line, e.g. `sudo pmc -u -b 0 'GET TIME_STATUS_NP'`.
- **Pulse output:** many NICs (e.g. i210 SDP pins) and sensors can output a PPS from their PTP clock. Compare two PPS signals on an oscilloscope to measure true sync error.
- **End-to-end test:** a blinking LED or fast-moving target seen by all sensors, compared in recorded timestamps.

**When to use it:** PTP is the standard choice for Ethernet sensors (LiDAR, GigE cameras) and for sharing one time base across several computers. Combine it with hardware triggers or scheduled action commands when you also need simultaneous capture.

### GPS + PPS in detail

GPS + PPS uses a GNSS receiver as a reference clock tied to the satellites' atomic clocks. The receiver outputs two signals. The **PPS (pulse-per-second)** line gives the exact start of each second, typically to within about 20–50 ns. The **NMEA serial message** says which second that pulse marked. Together they give absolute, globally consistent time without any network.

**1. Why you need both signals**

| Signal | What it gives | Limitation alone |
|---|---|---|
| **PPS** | A sharp electrical edge exactly at the start of each UTC second | No label: you don't know *which* second it is |
| **NMEA message** (e.g. `$GPRMC`, `$GNZDA`) | Date and time as text over a serial line | Arrives tens to hundreds of ms late, with variable delay |

The device uses NMEA to know which second it is, and the PPS edge to know exactly when that second started.

```
GNSS antenna (clear sky view)
      │
GNSS receiver ──PPS (TTL pulse, 1 Hz)──────────┬──► LiDAR sync-in
              └─NMEA (UART, once per second)───┤
                                               └──► Robot PC / trigger MCU / PTP grandmaster
```

**2. Ways to use it on a robot**

- **a. Feed sensors directly:** many LiDARs (Velodyne, Ouster, Hesai) accept PPS + NMEA inputs. They timestamp every point in GPS-based time, and many can phase-lock their rotation to the PPS.
- **b. Discipline the robot PC's system clock:** connect PPS to a GPIO or the serial port's DCD pin, which appears in Linux as `/dev/pps0`. Then use `gpsd` + `chrony`, which typically gets the system clock within about 1 µs.
  ```ini
  # /etc/chrony/chrony.conf
  refclock SHM 0 refid NMEA offset 0.2 noselect   # coarse time from gpsd (labels the second)
  refclock PPS /dev/pps0 lock NMEA refid PPS prefer   # precise edge
  ```
- **c. Discipline the NIC's PTP clock directly:** feed PPS into a NIC timing pin (e.g. Intel i210 SDP) and use linuxptp's `ts2phc`. This gives tens-of-ns accuracy. The PC then acts as **PTP grandmaster** for everything on the network (see "PTP in detail").
  ```ini
  # ts2phc.conf (pin index depends on your NIC)
  [global]
  ts2phc.nmea_serialport  /dev/ttyS0
  [eth0]
  ts2phc.pin_index        0
  ts2phc.extts_polarity   rising
  ```
  ```bash
  sudo ts2phc -f ts2phc.conf -s nmea -m
  ```
- **d. Lock a trigger generator to it:** an MCU or FPGA locks its timer to the PPS and produces camera triggers at exact fractions of each second, e.g. 10 Hz aligned to the second. It also timestamps each trigger.
- **e. Dedicated GNSS grandmaster or GNSS/INS unit:** commercial PTP grandmasters with GNSS input, or GNSS/INS units (u-blox, Septentrio, NovAtel, SBG). INS units also timestamp their IMU data in GPS time.

**Common outdoor setup:** GNSS → PC NIC (`ts2phc`) → PTP grandmaster → LiDAR and cameras over PTP, plus PPS wired directly to the LiDAR if it supports it.

**3. Electrical details**
- **Signal level:** PPS is usually 3.3 V or 5 V TTL. Some sensors expect RS-232 levels for NMEA or even for PPS. Check every input.
- **Edge and width:** the rising edge is usually the on-time edge (polarity is often configurable). Pulse width is typically 1–100 ms.
- **Fan-out:** use a buffer or driver chip to feed several sensors. Many loads on one line degrade the edge.
- **Cables:** shielded, kept short, away from motor wiring. The delay is about 5 ns/m, negligible for most robots but correctable.
- **NMEA format:** sensors often require a specific sentence (e.g. `$GPRMC`) and baud rate (often 9600). A mismatch means no lock. Some also require the NMEA message to arrive within a set window after the PPS.
- **Antenna:** needs a clear sky view. Its cable delay is also fixed and correctable.

**4. Time scales: a common source of offset bugs**

| Time scale | Relation | Used by |
|---|---|---|
| **UTC** | Civil time, with leap seconds | NMEA messages, most system clocks |
| **GPS time** | UTC + **18 s** (no leap seconds) | GNSS internals, some sensors |
| **TAI** | UTC + **37 s** = GPS + 19 s | PTP |

Devices using different scales end up exactly 18, 19 or 37 s apart. Know which scale each device reports and convert explicitly. Leap-second handling should be planned too.

**5. When GPS is lost: holdover**
Indoors, in tunnels, under bridges or between tall buildings, the receiver loses fix. The clock then free-runs on its local oscillator:
- **TCXO** (common in receivers and PCs): drifts by µs within seconds to minutes.
- **OCXO** (in dedicated timing gear): holds µs-level accuracy for hours.

**Key design rule:** for sync *between sensors on one robot*, it matters most that every sensor follows **one** local clock. If GPS drops, let the grandmaster or PC continue in holdover and keep distributing its time. Don't let each sensor fall back to its own free-running clock. Absolute accuracy degrades slowly, but relative sync stays tight.

**6. Accuracy (rough)**

| Path | Typical accuracy |
|---|---|
| Receiver PPS output | About 20–50 ns (timing receivers about 5–15 ns with quantization correction) |
| PPS → NIC hardware clock (`ts2phc`) | Tens of ns |
| PPS → sensor directly (LiDAR) | Under 1 µs |
| PPS → PC system clock (chrony) | About 1 µs |
| NMEA only, no PPS | Tens to hundreds of ms; not usable for sync |

**7. Where it fits**
- **Outdoor robots, autonomous vehicles, agricultural and survey robots:** a free, absolute time reference.
- **Multi-robot fleets:** every robot is synced to the same global time **without a network link** between them, which makes merging data from several robots easy.
- **Mapping:** sensor data lines up directly with GNSS/INS position data.
- **Not for indoor robots:** with no satellite signal, use a local PTP grandmaster instead.

**8. Pitfalls**
- **No fix, no or bad PPS:** some receivers stop PPS without a fix. Others output an unsynced PPS. Configure "PPS only when locked" or monitor the fix status.
- **Time-scale mix-ups:** offsets of exactly 18, 19 or 37 s.
- **Wrong NMEA sentence, baud or polarity:** the sensor never locks.
- **Cold start:** the first fix can take 30 s or more. Wait for lock before recording.
- **Multipath and urban canyons:** reflected signals degrade timing accuracy.
- **Jamming or spoofing:** a fake or blocked signal can make clocks jump. Safety-relevant systems should sanity-check time jumps.
- **Unbuffered fan-out and ground loops:** degraded edges, false pulses.

**9. Verifying it works**
- **`ppstest /dev/pps0`:** shows each PPS edge arriving with its kernel timestamp.
- **`chronyc sources -v`:** the PPS source should be selected, with an offset around a µs.
- **`ts2phc` logs:** the offset should settle to tens of ns.
- **Sensor status:** LiDAR web UIs and diagnostics show "PPS locked" and "NMEA valid."
- **Oscilloscope:** compare the receiver's PPS with each device's PPS or sync output (e.g. the NIC's PPS out) to measure the actual error.

### NTP in detail

NTP (Network Time Protocol) synchronizes computer clocks over ordinary IP networks in software. It's robust, needs no special hardware and works across the internet, but its accuracy is roughly **0.1–10 ms**. That's fine for aligning logs and computers, but usually **not good enough for sensor fusion** on a moving robot.

**1. Stratum hierarchy**
```
Stratum 0:  reference clocks (GPS receivers, atomic clocks), not on the network
Stratum 1:  servers directly connected to a stratum-0 source
Stratum 2:  servers syncing to stratum 1
   ...      (each level adds a little error)
```
Clients usually poll several servers and automatically pick the most reliable ones, discarding outliers.

**2. How the sync works**
It uses the same four-timestamp exchange as PTP:
```
Client                         Server
  t1 ──── request ──────────►  t2
  t4 ◄─── response (t2, t3) ── t3
```
- **Clock offset** = ((t2 − t1) + (t3 − t4)) / 2
- **Round-trip delay** = (t4 − t1) − (t3 − t2)

The client filters many samples, selects the best servers, then **slews** its clock: it speeds it up or slows it down slightly rather than jumping. Only large errors, usually at startup, are corrected by **stepping** (a sudden jump).

**3. Why it's less accurate than PTP**

| Factor | NTP | PTP |
|---|---|---|
| Timestamps | Usually software, taken by the OS with scheduling and interrupt delay | Hardware, taken by the NIC at the wire |
| Network support | None; switch and router queuing adds jitter | PTP-aware switches correct for queuing |
| Message rate | Every 16–1024 s by default (configurable) | Several times per second |
| Designed for | Robustness across the internet | Precision on a local network |

**Exception:** `chrony` supports **NIC hardware timestamping** for NTP (`hwtimestamp *`). On a quiet wired LAN that can reach µs-level or better, approaching PTP, if both ends support it.

**4. Implementations**

| Software | Notes |
|---|---|
| **chrony** | Recommended for robots: fast convergence, handles intermittent networks and offline operation, can act as client and server, supports hardware timestamping, PPS and PHC reference clocks |
| **ntpd** | The classic reference implementation; slower to converge, largely replaced by chrony |
| **systemd-timesyncd** | Default on many Linux distros. Client-only and simple (SNTP); coarse accuracy and can't serve time to other machines |

**5. Uses in robotics**
- **Multiple onboard computers:** e.g. main PC + Jetson + safety controller. ROS 2 messages and TF lookups compare timestamps across machines. Unsynced clocks cause errors like TF "extrapolation into the future" and dropped messages.
- **Logs and recordings:** aligning rosbags and logs from several machines and the fleet server.
- **Fleet and cloud:** keeping robots' clocks consistent with servers for logging, certificates (TLS fails with wrong time) and task scheduling.
- **Backup time source:** when PTP or GPS isn't available.

It's **not** suitable for camera-IMU, camera-LiDAR or stereo sync. For those, use hardware triggers, PTP or PPS.

**6. Typical robot setup with chrony**

One onboard computer is the local NTP server; the others are clients. The robot keeps a consistent internal time even with no internet.

```
Internet / GPS (when available) ──► Main PC (chrony server) ──► Jetson, other PCs (chrony clients)
```

Server (main PC), `/etc/chrony/chrony.conf`:
```ini
server time.example.com iburst      # upstream source, used when online
local stratum 10                    # keep serving time even when offline
allow 192.168.10.0/24               # let onboard computers sync to this one
makestep 1.0 3                      # allow stepping only during the first 3 updates
rtcsync                             # keep the hardware clock (RTC) updated
```

Client (other onboard computers):
```ini
server 192.168.10.1 iburst minpoll 2 maxpoll 4 prefer   # poll every 4–16 s on the LAN
makestep 1.0 3
hwtimestamp *                       # use NIC hardware timestamps if supported
```

Make robot software wait for time sync at boot: enable `chrony-wait.service` (or run `chronyc waitsync`) and order your nodes after `time-sync.target`.

**7. Accuracy (rough)**

| Setup | Typical accuracy |
|---|---|
| Internet servers | About 1–50 ms |
| Wi-Fi LAN | About 1–10 ms, with jitter |
| Wired LAN, chrony, software timestamps | About 50 µs–1 ms |
| Wired LAN, chrony, hardware timestamps | About 1 µs or better |
| systemd-timesyncd | ms to tens of ms |

**8. Pitfalls**
- **Wrong time at boot:** boards like Jetson or Raspberry Pi often have no battery-backed RTC (the Pi 5 has one if you add a battery). They boot with an old date, then jump forward when NTP syncs. Nodes started before sync get confusing timestamps. Wait for sync, or add an RTC.
- **Time jumps during operation:** a step backward or forward can break ROS timers, controllers, timeouts and recordings. Allow steps only at startup (`makestep 1.0 3`) and let chrony slew afterwards.
- **Offline robots:** with no upstream server, clients can't sync unless a local server has `local stratum` set.
- **Competing time daemons:** systemd-timesyncd and chrony both running, or chrony and `phc2sys` both adjusting the system clock. Run only one per clock.
- **Wi-Fi:** variable, asymmetric delays. A robot moving between access points sees jumps in delay and offset.
- **Firewall:** NTP uses **UDP port 123**, which may be blocked.
- **Leap-second smearing:** some public servers (e.g. Google's) spread a leap second over many hours instead of inserting it. Mixing smeared and non-smeared servers creates offsets of up to ~0.5 s around the event. Don't mix them.
- **Using arrival time as measurement time:** NTP syncs the PC clock, but sensor data stamped on arrival still carries transfer delay and jitter.

**9. Verifying it works**
- **`chronyc tracking`:** shows the current offset, RMS offset, stratum and reference source.
- **`chronyc sources -v`:** lists servers with their state (`*` = selected) and offsets.
- **`chronyc sourcestats`:** jitter and drift per source.
- **Across machines:** compare `chronyc tracking` on each, or check message header stamps against local time in ROS.

**10. Combining with other methods**
chrony can use precise references directly:
```ini
refclock PHC /dev/ptp0 poll 0 dpoll -2 offset 0 tai  # NIC clock already synced by PTP (PTP time is TAI)
refclock PPS /dev/pps0 lock NMEA refid PPS             # GPS PPS (see "GPS + PPS in detail")
```
A common split: **PTP or PPS for precise sensor time on the robot, NTP (chrony) to serve that time to devices and computers that don't support PTP**, and to sync with fleet servers.

## Driver vs. firmware

**Firmware** runs *on the device itself*: the microcontroller inside a sensor, servo drive, camera or MCU board. It controls the hardware directly. A **driver** runs *on the host computer's OS* and lets the OS and applications talk to that device. The two communicate over an interface like USB, Ethernet, CAN or a serial link.

```
Host computer (Linux)                          Device
┌────────────────────────────────┐             ┌──────────────────────────┐
│ Application / ROS nodes        │             │ Firmware (on device MCU) │
│              ↕                 │  USB / Eth  │  - reads sensor hardware │
│ Driver (user-space or kernel)  │ ◄─────────► │  - runs control loops    │
│  - talks the device's protocol │  CAN / ...  │  - generates timing      │
│  - exposes data to the OS/apps │             │  - implements protocol   │
└────────────────────────────────┘             └──────────────────────────┘
```

**Comparison**

| | Firmware | Driver |
|---|---|---|
| **Runs on** | The device's own MCU, FPGA or SoC | The host CPU, inside an OS |
| **Stored in** | The device's flash memory | The host's disk (kernel module, library, ROS package) |
| **Talks to** | Hardware registers, ADCs, timers, motor power stages | The OS, applications, and the device through its interface |
| **Timing** | Deterministic: µs/ns, bare-metal or an RTOS | Subject to OS scheduling: µs–ms jitter unless real-time |
| **Typical language** | C/C++ (sometimes Rust), vendor SDK/HAL | C (kernel), C++/Python (user space, ROS) |
| **Who writes it** | Usually the device maker; you, for custom boards | The OS, the device maker or the community; you, for custom devices |
| **Updated by** | Flashing over a bootloader, DFU or JTAG/SWD | Installing a package or kernel module |
| **If it crashes** | The device stops working; a watchdog may reset it | Host software loses the device; the device may keep running |

**Two kinds of driver**
- **Kernel driver:** part of the OS, e.g. USB, network, GPIO and MIPI camera drivers. It exposes devices as `/dev/...` files or network interfaces.
- **User-space driver:** a library or program using standard OS interfaces (sockets, serial, libusb, an SDK). Most ROS "drivers" are user-space nodes.

**Robotics examples**

| Device | Firmware does | Driver does |
|---|---|---|
| **Servo drive** | Runs the current/velocity/position loops at kHz rates, commutation, protection, STO | The EtherCAT/CANopen MainDevice stack plus a CiA 402 layer sends setpoints and reads status (e.g. `ros2_control` hardware interface) |
| **Industrial camera** | Exposure, trigger handling, readout, timestamping, GenICam feature set | The vendor SDK + ROS node configures settings, receives frames, publishes images |
| **LiDAR** | Spins the motor, fires lasers, measures range, PTP/PPS sync | The ROS node parses UDP packets into point clouds |
| **MCU trigger board** | Generates trigger pulses with hardware timers, timestamps them | The ROS node reads timestamps and re-stamps camera frames |
| **IMU** | Samples the sensor, filters, outputs data at a fixed rate | Parses the serial/SPI stream and publishes `sensor_msgs/Imu` |

**Gray areas**
- **Firmware loaded by a driver:** some devices (Wi-Fi chips, GPUs, some cameras) have no stored firmware. At boot, the Linux driver uploads a firmware file from `/lib/firmware`.
- **"Firmware" on PCs:** BIOS/UEFI is also called firmware. It runs on the motherboard before the OS starts.
- **FPGA logic:** often counted as firmware ("gateware"), even though it's hardware configuration rather than code.

**Where to put logic**
- **Firmware:** anything timing-critical or safety-critical, or that must keep working if the host crashes. Examples: control loops, trigger generation, hardware timestamping, fault protection.
- **Driver:** configuration, data conversion (raw bytes → messages with units and frames), integration with the OS and ROS, and anything that's easier to update and debug on the host.
- **Define the interface between them clearly:** message formats, units, timestamps, sequence numbers, error codes, and a protocol version. Then each side can be updated without breaking the other.

## Robot system schema

"Schema" means different things depending on who's talking. Usually it's the **system schematic**, a block diagram of how a robot's parts connect. In software it can also mean a robot **description file** or a **data/message schema**.

**1. System schematic (architecture diagram)**

This shows the main subsystems and the power, signal and data links between them:

```
                    ┌──────────────────────────────┐
   User / HMI ────► │   High-level computer (PC)   │ ◄── Cameras, LiDAR (USB/GigE/Ethernet)
   (teach pendant,  │  planning, perception, ROS 2 │
    network)        └──────────────┬───────────────┘
                                   │ EtherCAT / CAN / Ethernet
                    ┌──────────────▼───────────────┐
                    │  Motion controller (real-time)│ ◄── Safety controller (E-stop, STO,
                    │  trajectory, kinematics, 1 kHz│      light curtains, safe I/O)
                    └──────────────┬───────────────┘
                                   │ fieldbus (EtherCAT)
          ┌────────────┬───────────┼───────────┬────────────┐
     ┌────▼───┐   ┌────▼───┐  ┌────▼───┐  ┌────▼───┐   ┌────▼────┐
     │Drive J1│   │Drive J2│  │  ...   │  │Drive J6│   │ Gripper/│
     └────┬───┘   └────┬───┘  └────────┘  └────┬───┘   │ tool I/O│
       motor+encoder+brake per joint           ...    └─────────┘

   Power: AC mains → main contactor → DC bus (48 V / 300–600 V) → drives
          + separate 24 V logic supply for controllers, sensors, brakes
```

It typically has these layers:

| Layer | Contents |
|---|---|
| **Power** | Mains, power supplies, contactors, DC bus, brakes, E-stop chain |
| **Actuation** | Motors, gearboxes, servo drives, encoders |
| **Sensing** | Joint encoders, force/torque sensors, IMU, cameras, LiDAR |
| **Control** | Real-time motion controller and safety controller |
| **Compute / software** | Perception, planning, task logic, UI |
| **Communication** | Fieldbuses (EtherCAT, CAN, RS-485), Ethernet, time sync (PTP) |

Engineers usually draw several views of the same system: power wiring, signal/communication, safety circuit, and software nodes.

**2. Robot description schema (the robot's model)**

These are machine-readable files that describe the robot's links, joints, limits, mass and inertia, and geometry:
- **URDF** (ROS): XML with `<link>` and `<joint>` elements
- **SDF** (Gazebo), **MJCF** (MuJoCo), **USD** (NVIDIA Isaac)

```xml
<joint name="shoulder" type="revolute">
  <parent link="base"/><child link="upper_arm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="150" velocity="2.0"/>
</joint>
```

**3. Data / interface schema**

These define the structure of the messages that components exchange, for example ROS 2 `.msg` files (`sensor_msgs/JointState`: name[], position[], velocity[], effort[]), Protobuf, or JSON Schema for APIs. For an LLM agent that controls a robot, this is the **tool schema**: a JSON Schema for each robot action the model can call.

### Hardware schematics in detail

On the hardware side, a robot is documented in several standard drawing types:

| Drawing | What it shows | Used for |
|---|---|---|
| **Block diagram** | Major hardware units and how they connect, without wiring detail | Overview, design reviews |
| **Single-line (power) diagram** | Power path: mains → breaker → contactor → PSU → DC bus → drives, plus fuses and ratings | Sizing, power design |
| **Electrical schematic** (circuit diagram) | Every component and connection using standard symbols, grouped by function, not physical position | Building, troubleshooting |
| **Wiring / interconnect diagram** | Physical cables between devices: connector pins, wire colours and gauges, terminal numbers | Assembly, field service |
| **Cable harness drawing** | Cable lengths, connectors, shielding, routing through the arm | Manufacturing cables |
| **Safety circuit diagram** | E-stop chain, safety relays/PLC, STO lines, interlocks, dual-channel wiring | Safety certification (ISO 13849) |
| **Cabinet layout** | Physical placement of DIN-rail parts, drives and ducts in the control cabinet | Panel building, heat and EMC planning |
| **Fluid (pneumatic/hydraulic) schematic** | Valves, cylinders, vacuum grippers | Grippers and tooling |
| **PCB schematic + layout** | Circuits on custom boards (drive, sensor, I/O boards) | Electronics design |
| **Mechanical drawings / CAD** | Parts, assemblies, tolerances, bill of materials (BOM) | Machining, assembly |

**Example: wiring/interconnect diagram for one joint**

```
 Control cabinet                                         Joint J2
┌───────────────────────┐   Motor cable (shielded)   ┌──────────────────┐
│ Servo drive J2        │                            │                  │
│  U ───────────────────┼────────────────────────────┼─ U  ┐            │
│  V ───────────────────┼────────────────────────────┼─ V  ├ Motor      │
│  W ───────────────────┼────────────────────────────┼─ W  ┘            │
│  PE ──────────────────┼──── shield + PE ───────────┼─ PE (housing)    │
│  BR+ / BR- ───────────┼──── brake pair ────────────┼─ Brake 24 V      │
│                       │   Encoder cable (twisted)  │                  │
│  ENC  ────────────────┼──── data+/- , 5 V, GND ────┼─ Encoder         │
│  STO1 / STO2 ◄────────┼── from safety relay        │                  │
│  EtherCAT IN/OUT ◄───►┼── to drives J1 / J3        │                  │
└───────────────────────┘                            └──────────────────┘
```

**Standards and conventions**
- **IEC 60617:** graphical symbols for electrical schematics.
- **IEC 81346:** reference designations, such as `-K1` for a contactor or `-F3` for a fuse.
- **IEC 60204-1 / NFPA 79:** electrical safety of machinery, including wire colours, protective earth and E-stop.
- **ISO 1219:** fluid-power symbols.
- **ISO 13849 / IEC 62061:** functional safety, which determines how the safety circuit is drawn and validated.

**Tools:** EPLAN or AutoCAD Electrical for cabinets and wiring; KiCad or Altium for PCBs; SolidWorks, Fusion or Onshape for mechanical CAD; WireViz (free, text-based) for harness diagrams.

In practice the **electrical schematic + wiring diagram + BOM** set is what you need to build, certify and service a robot.

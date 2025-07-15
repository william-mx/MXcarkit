# Programming the DFRobot Gravity: URM09 Ultrasonic Distance Sensors

This repository contains tooling for programming the **10 ultrasonic sensors installed in the MXcarkit**.  
In the directory `uss_programming/`, there are 10 precompiled hex files:

```

set_<hex_id>_<id>.hex

````

These hex files configure the individual ultrasonic sensors with unique IDs.

---

## Default Sensor Mapping

Before starting, plan in which order you want to assign the IDs.  
The default mapping is as follows:

| ID  | Frame      | Position Description    |
|-----|-----------|--------------------------|
| 0   | USS_SRF   | Side Right Front        |
| 1   | USS_SRB   | Side Right Back         |
| 2   | USS_BR    | Back Right              |
| 3   | USS_BC    | Back Center             |
| 4   | USS_BL    | Back Left               |
| 5   | USS_SLB   | Side Left Back          |
| 6   | USS_SLF   | Side Left Front         |
| 7   | USS_FL    | Front Left              |
| 8   | USS_FC    | Front Center            |
| 9   | USS_FR    | Front Right             |

We recommend writing the corresponding ID numbers directly on the sensors for easy identification.

---

## Programming Procedure

### 1️⃣ Preparation
- **Unplug all ultrasonic sensors** from the system.
- **Decide and document the ID assignment order** (see table above).

---

### 2️⃣ Connect J-Link to the Microcontroller
Run the following to start J-Link Commander:

```bash
JLinkExe -device STM32L432KC -if SWD -speed 4000
````

In the J-Link prompt:

```
connect
```

---

### 3️⃣ Erase flash memory:

```
erase
```

---

### 🔄 4️⃣–7️⃣ Repeat for each sensor:

For **each sensor (IDs 0–9)**:

#### 4️⃣ Load the corresponding HEX file:

Example for sensor with ID `0`:

```bash
loadfile /home/mxck/MXcarkit/uss_programming/set_10_0.hex
```

#### 5️⃣ Reset the MCU:

```
reset
```

#### 6️⃣ Plug in the sensor:

* Plug in the **sensor to be programmed with the current ID**.

#### 7️⃣ Start program execution:

```
go
```

---

### ℹ️ Status LED behavior:

> If successful, the LED stays ON permanently.
> If the sensor did not have the default address, the LED blinks quickly (250 ms) and does nothing.
> If the sensor could not be reprogrammed, the LED blinks slowly (500 ms).

---

## Final step:

After all sensors are programmed, **ensure the Micro-ROS software is flashed again** for normal operation.

---

✅ **Important notes:**

* Only one sensor should be connected during programming.
* The microcontroller can remain connected the entire time — **only steps 4–7 need to be repeated for each sensor**.
* Document the ID assignments for future maintenance.


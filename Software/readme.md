## MATLAB Plotting
Produces figures from the recorded flight data.

---

## UWB Flight Control
Software that runs on the flight controller.

### Key source files
| Purpose | File(s) |
|---------|---------|
| Main application entry point | `core/src/main.c`, `core/src/main.h` |
| RTOS scheduler | `core/src/freertos.c` |
| UWB multilateration functions | `tasks/tasks_uwb.c`, `tasks/tasks_uwb.h` |

**Note:** LittleFS is used to update the flash memory.

### Drivers
- **`Radio/`** – driver for the onboard **LoRa** module.  
- **`CRSF_LIB/`** – driver for the **CRSF** protocol, enabling control via an **ELRS** transmitter (connected through a 4‑pin Picoblade connector and **not** to be confused with the onboard LoRa module).

---

## UWB_CAN_REVB
Firmware for the UWB‑enabled PCB.

| Mode | Description |
|------|-------------|
| **Static mode** | PCB is mounted on a fixed surface (e.g., a wall). |
| **Mobile mode** | PCB is mounted on the UAV; distance data to nodes A–F is sent to the flight controller via CAN bus as soon as new measurements are available. |

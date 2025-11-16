# Lab: CAN — Hardware Filters & Node Interaction

> **Title:** Lab: CAN  
> **Developers:** Mariia Ivanchenko,  Oleksandr Ivaniuk, Anastasiia Pelekh

This lab demonstrates how CAN hardware filters work by using two STM32-based nodes:

- **Node C (CTRL)** — acts as a **guard / watchdog** (receiver)
- **Node A (TESTER)** — acts as a **traffic generator** (sender)

Only specific CAN message IDs should trigger reactions on the receiver. All other messages are ignored on the hardware level.

---

## Node C — CTRL (Receiver / Guard)

**Role:** Listens on the CAN bus and reacts **only** to selected message IDs.

### Hardware Filter Configuration

Node C configures its CAN peripheral to accept **only** these two IDs:

- `0x040` — **Command**
- `0x081` — **Heartbeat**

All other IDs (for example `0x120`, `0x180`) are **rejected by hardware filters** and never reach the application layer.

### LED Behaviour on Allowed IDs

When Node C receives a frame that passes the hardware filters:

- **ID `0x040` (Command)**
  - The LED **blinks quickly twice**
  - Indicates: *"Command received"*

- **ID `0x081` (Heartbeat)**
  - The LED **toggles its state** (ON → OFF or OFF → ON)
  - Indicates: *"Heartbeat received"*

---

## Node A — TESTER (Transmitter)

**Role:** Sends a sequence of CAN frames to test Node C’s filtering.

Node A **does not receive** anything. It only **transmits** CAN frames in an infinite loop.

### Transmission Pattern

Every **2 seconds**, Node A sends the following sequence of CAN IDs:

1. **ID `0x120`**  
   - Should be **ignored** by Node C (filtered out by hardware)

2. **ID `0x040`**  
   - Should be **accepted** by Node C  
   - Triggers: **double fast blink** on Node C’s LED

3. **ID `0x180`**  
   - Should be **ignored** by Node C

4. **ID `0x081`**  
   - Should be **accepted** by Node C  
   - Triggers: **LED toggle** on Node C

---

## Summary of Expected Behaviour

| CAN ID | Meaning   | Should CTRL Accept? | CTRL LED Reaction               |
|--------|-----------|---------------------|---------------------------------|
| 0x040  | Command   | Yes                 | Fast double blink               |
| 0x081  | Heartbeat | Yes                 | Toggle LED state (ON/OFF)       |
| 0x120  | Noise     | No                  | No LED change (ignored)         |
| 0x180  | Noise     | No                  | No LED change (ignored)         |

---

## Purpose of the Lab

Together, Node A and Node C validate **Part B** of the CAN lab:

- Node A (TESTER) sends a mix of **“trash”** and **“valid commands”**.
- Node C (CTRL) reacts **only** to the allowed IDs.
- The visible LED behaviour proves that **hardware-level CAN filters** are correctly configured and working.
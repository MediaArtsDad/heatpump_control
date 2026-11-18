# Heatpump Controller — TODO & Change Log

## Open TODOs

### Hardware / wiring (Jeff)
- [ ] **Confirm the relay board has onboard pull-ups on its inputs** — pins float from MCU reset until setup() drives them HIGH; pull-ups keep relays off during that gap (most opto-isolated boards have them).
- [ ] **Connect K3 (upper tank element) to pin 5** — firmware now drives it as `aux_upper` (stage-2 AUX logic + EXTEMERG backup call).
- [ ] **Future: second element (lower tank) on pin 6** (`aux_lower`) — already driven by EXTEMERG-only logic; just wire it when/if more resistive capacity is needed.
- [ ] **Fill in DS18B20 ROM addresses** — set `SENSOR_DISCOVERY_MODE true`, flash, copy addresses from Serial Monitor into the `*_ADDR` arrays, set back to `false`. Until then all four sensors read as disconnected and aux heat stays locked out.
- [ ] **(#6) Add electronic filters** — RC filter on each opto output / input pin (HEAT1, HEAT2, EXTCALL, EXTEMERG) so the hardware presents a clean DC level; the software filter then becomes a second layer instead of the only one.
- [ ] **(#7) Verify input pull-downs** — each input needs a pull-down so it can't float when its source contact is open. If sources are dry contacts to ground, switch code to `INPUT_PULLUP` + inverted reads instead.
- [ ] **Verify LCD degree symbol** — code uses `(char)176`; some HD44780 ROMs use `0xDF` (223).
- [ ] **Verify bootloader is optiboot** (standard Uno R3) — very old bootloaders boot-loop after a watchdog reset.

### Firmware / project (either)
- [ ] **Replace extpump 60 s cooldown timer with real pump/flow feedback** from the heat pump when available.
- [ ] **Confirm stage-2 assumptions** — HEAT2 HIGH = stage-2 call; thresholds 120 °F on / 130 °F off.
- [ ] **Confirm 5-min compressor delay after power-up/reset is acceptable** (side effect of min-off timer initialization; also applies after a watchdog reset).
- [ ] Optional RAM trims (review #9): shrink `tArray[16]` to 4 (or drop it), make pin numbers `const uint8_t`, drop the unused Fahrenheit floats (only `TANK_TOP_F_t4` is used).

## Change Log
<!-- Newest entries on top. Keep this section to a maximum of 100 lines; delete oldest entries beyond that. -->

### 2026-07-12 — Switched to sekdiy/Relay library (LOW-trigger board confirmed)
- Jeff identified the intended library was **sekdiy/Relay**, not Erutan409's (same filename, different library). Compared both; sekdiy wins: explicit polarity parameter, no Avail dependency, boolean `isOn()`. Board confirmed LOW-trigger; encoded as `RELAY_ACTIVE_MODE = LOW`.
- **sekdiy quirk found & worked around:** `setState()` skips the pin write when state doesn't change, and state initializes OFF — so a LOW-active relay constructed OFF never gets its pin driven HIGH, leaving it *energized* from boot. setup() now force-writes all five relay pins HIGH first thing.
- **Latent trap documented:** the original sketch's old `Relay(2, true)` under sekdiy's signature = initial state ON (every relay energized at power-up); constructors now pass explicit `Relay::OFF` + mode. Relay pins moved to named `PIN_*` constants.
- Sketch converted: `relayOn()` helper dropped in favor of native `isOn()`; project `lib/` now holds sekdiy Relay.h (Jeff's upload) + Relay.cpp; RelayConfig.h removed (not part of this library). Erutan409 TODOs (Avail, trigger-type) closed.

### 2026-07-12 — Relay library adopted (Erutan409/Arduino-Relay-Library) — review + integration
- Library files added to project under `lib/` (Relay.h from Jeff's upload; Relay.cpp + RelayConfig.h from the GitHub repo). TODO #15 closed.
- **Review findings:** LOW-trigger drive (on() = pin LOW — board type must be confirmed); constructor initializes pin OUTPUT/HIGH at global construction so relays power up OFF, no `begin()` exists; `RELAY_SAFETY true` (default) requires the Avail library and silently drops state changes within 250 ms of the last one (benign — loop() re-commands every pass, so it acts as an anti-chatter rate limit); `getState()` returns a *pointer* — comparing it to true/false compiles but is always-true.
- **Sketch adapted to the real API:** single-arg constructors, `begin()` calls removed, `turnOn()/turnOff()` → `on()/off()`, all state reads through new `relayOn()` helper (`*getState() == RELAY_ON`). Verified by compiling against a stub that mirrors the library's exact signatures.
- Timing note: min-off/min-run timers key off *accepted* state changes, and commands dropped by the 250 ms throttle are retried every pass — timers stay truthful.

### 2026-07-12 — High-limit values confirmed
- `TANK_MAX_C` 90 → **85 °C (185 °F)**, `TANK_MAX_RESUME_C` 85 → **75 °C (167 °F)** per Jeff. TODO closed.

### 2026-07-12 — Tank element architecture (K3 + future lower element)
- **Relays renamed to physical meaning:** `emergency` → `aux_upper` (pin 5 = K3, upper tank element), `ext_emergency` → `aux_lower` (pin 6, reserved for future lower element).
- **Element logic unified:** shared 120/130 °F hysteresis latch on tank top. `aux_upper` = stage-2 demand OR EXTEMERG; `aux_lower` = EXTEMERG only ("all hands" backup — forces both elements, so future lower element needs no code change).
- **EXTEMERG now temperature-gated** (Jeff's pick): compressor's backup call only energizes elements when the tank band says heat is needed — no element heat into an already-hot tank (e.g. defrost with tank at 140 °F).
- **Sensor-loss asymmetry (Jeff's pick):** stage-2 AUX fails OFF (comfort top-up, don't heat blind); EXTEMERG fails toward HEAT (freeze-protection path; mechanical high-limit is the backstop).
- High-limit override now forces `aux_upper` + `aux_lower` off by name; LCD Mode legend updated (AUX = upper element, EXTEM = compressor backup with both elements).

### 2026-07-11 — Decisions from Jeff + status display rework
- **EXTEMERG joins the high-limit override** (Jeff confirmed backup heat can feed the tank): a tank over-temp trip now forces off compressor, stage-2 aux, AND ext_emergency, overriding the compressor's call. Gated at both the follow logic and the override so the relay can't pulse within one pass.
- **Decision recorded: compressor keeps running on tank-top sensor loss.** Jeff confirmed a physical (mechanical) temp limit exists on the tank as the backstop when the software limit is blind. (Removed the open TODO.)
- **LCD Stat field reworked** — old field only mirrored the ext-pump state machine. New statuses computed from live conditions each pass: HILIM (high-limit tripped), HEAT (a heat source actually running, incl. min-run hold), PRE (heat call active, compressor waiting on min-off timer), EXT (EXTCALL-only, e.g. defrost), COOL (cooldown), IDLE.

### 2026-07-11 — Full logic-flow review fixes
- **Interior pump control unified into one block** evaluated every pass: previously its hysteresis only ran during a HEAT1 call, so during EXTCALL-only operation (e.g. long defrost) or cooldown it froze in its last state — could run indefinitely with a cooling tank. Now: off when ext pump off, else hysteresis (50/35 °C), sensor loss = off.
- **HEAT2 filter read every pass**: previously only read during a heat call, so a call resuming within 100 ms of dropping could see a phantom stage-2 call from the stale filter latch and pulse the aux relay.
- **Software tank high-limit added**: tank top ≥ 90 °C forces compressor + stage-2 aux off (overrides the min-run hold; also pulses-safe — the heat-call branch is gated so the relay can't flick on/off within one pass), locks out until < 85 °C, keeps pumps running to shed heat, starts the compressor min-off timer, prints trip/clear to Serial. Latch holds through sensor loss.
- **LCD shows "---" instead of -127** for disconnected/unset sensors.
- Verified OK in the same review: min-off/min-run interplay (incl. call returning during a min-run hold), boot with all-call inputs active (temps init as disconnected block aux heat; min-off blocks compressor), millis() rollover on all timers, cooldown re-trigger on renewed demand, watchdog margin vs. worst-case loop pass, EXTCALL-during-cooldown restart.
- Known cosmetic quirk (not fixed): LCD Stat shows HEAT during EXTCALL-only operation even though only the glycol pump is running.

### 2026-07-11 — Review fixes (review items 1, 2, 4, 5, 6, 8, 10–14)
- **#1 Boot safety:** all temperature globals now initialize to `DEVICE_DISCONNECTED_C` instead of 0.0, so the first ~1.75 s after reset can't fire aux heat off a garbage "0 °F" reading. Fahrenheit values propagate the disconnected sentinel instead of converting it.
- **#2 Interior pump hysteresis:** on above 50 °C, off below 35 °C, hold in between (previously never turned off during a heat call).
- **#4 Fail-safe:** stage-2 aux heat now turns OFF on tank-top sensor loss instead of holding last state; intpump also fails OFF.
- **#5 Serial warnings:** sensor lost/recovered messages print once per state change instead of 4 lines/second.
- **#6 Software input filters:** all four inputs go through a retriggerable one-shot (HIGH latches instantly, LOW only after 100 ms continuous LOW) — rides through 60 Hz opto ripple and debounces contacts.
- **#8 LCD efficiency:** static labels drawn once in setup; per-second refresh only rewrites value fields, fixed-width padded (also eliminates ghost digits, review #3).
- **#10:** `sensorcount` moved local to `discoverSensors()`.
- **#11 Watchdog:** WDT enabled at 2 s; fed every loop pass; disabled first thing in setup to avoid reset loops.
- **#12 Compressor min-run:** 5 minutes minimum run time once started; glycol pump held on during the min-run hold.
- **#13:** interior pump now runs until the exterior pump stops (covers the cooldown wind-down).
- **#14 LCD status:** rows 2–3 now show INT/EXT pump ON/OFF, Stat (HEAT/COOL/IDLE), Mode (OFF/HP/AUX/HP+AX/EXTEM).
- TODOs added in code comments for #6 (RC filters), #7 (pull-downs), #15 (Relay.h).

### 2026-07-11 — Non-blocking temps + EXTCALL/EXTEMERG
- DS18B20 reads made non-blocking: `setWaitForConversion(false)`, request/read split in `loop()`; removed the ~750 ms per-second busy-wait.
- EXTCALL input (compressor call for glycol pump) added: ext pump runs on thermostat call OR EXTCALL; 60 s cooldown starts only after all demand drops.
- EXTEMERG input (compressor call for emergency backup heat) added: `ext_emergency` relay (pin 6) follows it directly.

### Earlier (pre-existing baseline)
- Sensor identification by ROM address instead of bus-scan order; discovery mode helper.
- Blocking while-loops replaced with state machine; compressor 5-min min-off (short-cycle protection); LCD print bugs fixed.

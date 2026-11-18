#include "Relay.h"
#include <OneWire.h> // https://lastminuteengineers.com/multiple-ds18b20-arduino-tutorial/#:~:text=Finally%2C%20we%20use%20a%20simple%20for%20loop%20to,to%20all%20the%20sensors%20for%20temperature%20conversion%20sensors.requestTemperatures%28%29%3B
#include <DallasTemperature.h> // https://arduinogetstarted.com/tutorials/arduino-temperature-sensor
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <avr/wdt.h> // watchdog timer

// Relay library (sekdiy/Relay), reviewed 2026-07-12:
//  - Relay(pin, initialState, mode): polarity is EXPLICIT -- mode LOW means
//    LOW-active (relay energizes when the pin is driven LOW), matching this
//    system's LOW-trigger relay board (confirmed by Jeff 2026-07-12).
//  - isOn()/isOff() return real booleans, reflecting the commanded state --
//    exactly what the min-off/min-run timers need. No begin(), no external
//    dependencies.
//  - KNOWN QUIRK (worked around at the top of setup()): the internal state
//    starts as OFF and setState() skips the pin write when the state doesn't
//    change -- so constructing a LOW-active relay as OFF never actually
//    drives the pin HIGH, leaving the relay ENERGIZED from boot until its
//    first on()/off() cycle. setup() forces every relay pin HIGH explicitly.
const uint8_t PIN_INTPUMP   = 2;
const uint8_t PIN_EXTPUMP   = 3;
const uint8_t PIN_HEATPUMP  = 4;
const uint8_t PIN_AUX_UPPER = 5; // K3, upper tank element
const uint8_t PIN_AUX_LOWER = 6; // reserved: future lower tank element
const unsigned int RELAY_ACTIVE_MODE = LOW; // LOW-trigger board: pin LOW = relay energized
Relay intpump(PIN_INTPUMP, Relay::OFF, RELAY_ACTIVE_MODE),
      extpump(PIN_EXTPUMP, Relay::OFF, RELAY_ACTIVE_MODE),
      heatpump(PIN_HEATPUMP, Relay::OFF, RELAY_ACTIVE_MODE),
      aux_upper(PIN_AUX_UPPER, Relay::OFF, RELAY_ACTIVE_MODE),
      aux_lower(PIN_AUX_LOWER, Relay::OFF, RELAY_ACTIVE_MODE);
// aux_upper (pin 5): K3, resistive element in the upper buffer tank. Driven
//   by stage-2 AUX logic (thermostat) and by EXTEMERG (compressor backup call).
// aux_lower (pin 6): reserved for a future second element (lower tank), not
//   yet installed/connected. Driven by EXTEMERG only -- "all hands" backup.
int HEAT1 = 11,HEAT2 = 12,EXTCALL = 8,EXTEMERG = 9;  //name input pins
// TODO (#7): verify every input has a pull-down resistor so it can't float
// when its source contact is open. If the sources are dry contacts switching
// to ground instead, change to INPUT_PULLUP and invert the reads.
const int32_t tArraySize = 16; //array size for temperatures
int tArray[tArraySize]; // legacy display array, kept in sync from the named floats below -- no longer used to decide which sensor is which
// Initialize as DISCONNECTED, not 0: globals default to 0.0, which reads as a
// valid "0 degrees" for the first ~1.75 s after power-up (until the first
// conversion completes) and previously let stage-2 aux heat fire off garbage
// data during a reset mid-heat-call. DEVICE_DISCONNECTED_C makes the existing
// guards treat "no reading yet" the same as "sensor missing": hold safe.
float TANK_TOP_C_t0    = DEVICE_DISCONNECTED_C,
      TANK_BOTTOM_C_t1 = DEVICE_DISCONNECTED_C,
      EXT_IN_C_t2      = DEVICE_DISCONNECTED_C,
      INT_OUT_C_t3     = DEVICE_DISCONNECTED_C; // temperatures in Celsius
float TANK_TOP_F_t4    = DEVICE_DISCONNECTED_C,
      TANK_BOTTOM_F_t5 = DEVICE_DISCONNECTED_C,
      EXT_IN_F_t6      = DEVICE_DISCONNECTED_C,
      INT_OUT_F_t7     = DEVICE_DISCONNECTED_C; // temperatures in Fahrenheit

//Setup Onewire instance
#define T_BUS 10 // One wire bus
OneWire tempbus(T_BUS);
DallasTemperature sensors(&tempbus); // share onewire with Dallas Temp

//initialize the liquid crystal library - https://projecthub.arduino.cc/arduino_uno_guy/i2c-liquid-crystal-displays-5eb615
//the first parameter is  the I2C address
//the second parameter is how many rows are on your screen
//the  third parameter is how many columns are on your screen
LiquidCrystal_I2C lcd(0x27,  20, 4);

// ---- DS18B20 sensor identification by ROM address ----
// Reading sensors by bus-scan order (old behavior) isn't reliable -- discovery
// order can change if a sensor is replaced or the bus is rescanned, and this
// firmware makes heating decisions off those readings. Identify sensors by
// their unique 64-bit address instead:
//   1. Set SENSOR_DISCOVERY_MODE to true below, flash, open Serial Monitor @ 9600.
//   2. Copy each printed address into the matching *_ADDR array below (warm/cool
//      one sensor at a time to tell which physical sensor is which).
//   3. Set SENSOR_DISCOVERY_MODE back to false and reflash.
// Until an address is filled in, that sensor reads as "disconnected"
// (DEVICE_DISCONNECTED_C, -127) rather than silently reading the wrong
// physical sensor -- watch the Serial monitor for WARNING lines, and note
// that loop() holds safe (aux heat OFF, see stage-2 logic) instead of acting
// on a disconnected reading.
#define SENSOR_DISCOVERY_MODE false

DeviceAddress TANK_TOP_ADDR    = {0,0,0,0,0,0,0,0}; // TODO: fill in from discovery output
DeviceAddress TANK_BOTTOM_ADDR = {0,0,0,0,0,0,0,0}; // TODO: fill in from discovery output
DeviceAddress EXT_IN_ADDR      = {0,0,0,0,0,0,0,0}; // TODO: fill in from discovery output
DeviceAddress INT_OUT_ADDR     = {0,0,0,0,0,0,0,0}; // TODO: fill in from discovery output

// ---- Timing / thresholds ----
// Minimum time the heat pump compressor must stay off before it's allowed to
// restart, to protect it from short-cycling damage. Side effect: since the
// timer initializes as if the compressor just turned off, this also enforces
// a 5-minute delay before the compressor can start after power-up or a
// reset. Confirm that's acceptable (e.g. after a brownout during an active
// heat call, you'll wait up to 5 minutes before the compressor resumes).
const unsigned long COMPRESSOR_MIN_OFF_MS = 5UL * 60UL * 1000UL; // 5 minutes

// Minimum time the compressor must RUN once started, so a flapping thermostat
// call can't cycle it with arbitrarily short run times. While the min-run
// timer is holding the compressor on after the heat call drops, the glycol
// pump is held on too (see glycolDemand in loop()).
const unsigned long COMPRESSOR_MIN_RUN_MS = 5UL * 60UL * 1000UL; // 5 minutes

// How long to keep the exterior pump running after all demand stops (thermostat
// heat call, EXTCALL, and compressor min-run all dropped), so it doesn't cut
// water flow while the compressor(s) are still winding down. TODO: replace
// with an actual pump/flow sensor from the heat pump when available.
const unsigned long EXTPUMP_COOLDOWN_MS = 60UL * 1000UL; // 60 seconds

// How often a new temperature conversion is started / display refreshed.
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 1000UL; // 1 second

// Software input filter (see readFilteredInput below): a HIGH is recognized
// immediately and latched; the input only reads LOW after it has been
// continuously LOW for this long. This rides through the 50/60 Hz off
// half-cycles of an optocoupler driven from a 24 VAC signal (~8 ms gaps) and
// also debounces contact bounce on the falling edge.
// TODO (#6): add electronic filters (RC on each opto output / input pin) so
// the hardware presents a clean DC level too; this software filter then
// becomes a second layer of protection instead of the only one.
const unsigned long INPUT_OFF_DELAY_MS = 100UL;

// Tank element thresholds (shared by stage-2 AUX and EXTEMERG demand), tank
// top temp in Fahrenheit: elements engage below ELEMENT_ON_F, disengage at/
// above ELEMENT_OFF_F, hold in between (hysteresis -- no chatter at a single
// setpoint).
const float ELEMENT_ON_F  = 120.0;
const float ELEMENT_OFF_F = 130.0;

// Interior pump hysteresis, tank top temp in Celsius: on above INTPUMP_ON_C,
// off below INTPUMP_OFF_C, hold in between. Sensor loss = pump OFF (fail safe).
const float INTPUMP_ON_C  = 50.0;
const float INTPUMP_OFF_C = 35.0;

// Software tank high-limit: if the tank top reaches TANK_MAX_C, the
// compressor and stage-2 aux heat are forced off (this overrides the min-run
// hold -- safety beats short-cycle protection) and stay locked out until the
// tank falls back below TANK_MAX_RESUME_C. Pumps deliberately keep running
// to move heat out of the tank. This is a firmware backstop only -- a
// mechanical high-limit on the tank is still required (see TODO doc).
// Values confirmed by Jeff 2026-07-12.
const float TANK_MAX_C        = 85.0; // 185 F
const float TANK_MAX_RESUME_C = 75.0; // 167 F

// ---- State machine (replaces the old blocking while-loops) ----
// Note: currentState drives the ext pump cooldown timing only. What the LCD
// shows is displayStatus below, computed at the end of loop() from the live
// conditions -- the control state alone can't distinguish "waiting on the
// compressor min-off timer" from "actually delivering heat".
enum SystemState { STATE_HEATING, STATE_COOLDOWN, STATE_IDLE };
SystemState currentState = STATE_IDLE;

// LCD Stat field (5 chars max):
//   HILIM -- tank high-limit tripped (all heat sources forced off)
//   PRE   -- heat call active, compressor locked out by the min-off timer
//            and no aux running yet (glycol pump pre-circulating)
//   HEAT  -- heat actually being delivered (compressor or aux heat running;
//            includes the min-run hold after the call drops)
//   EXT   -- EXTCALL-only: compressor servicing itself (e.g. defrost), no
//            thermostat call
//   COOL  -- cooldown wind-down (pumps only)
//   IDLE  -- everything off
const char* displayStatus = "IDLE";

unsigned long lastHeatpumpOffTime = 0; // millis() timestamp; see COMPRESSOR_MIN_OFF_MS note above
unsigned long lastHeatpumpOnTime  = 0; // set on each OFF->ON transition; see COMPRESSOR_MIN_RUN_MS
unsigned long extpumpCooldownStart = 0;
bool tankOverTemp = false; // high-limit hysteresis latch; holds last value on sensor loss (safe: a tripped latch can't clear without a valid cool reading)
bool tankNeedsElementHeat = false; // element-band hysteresis latch (ELEMENT_ON_F/ELEMENT_OFF_F); freezes on sensor loss -- the two demand paths handle the unknown differently

// ---- Non-blocking DS18B20 conversion tracking ----
// With setWaitForConversion(false), requestTemperatures() just kicks off a
// conversion on every sensor and returns immediately instead of sitting in a
// busy-wait for ~750 ms (12-bit resolution). We note when the request was
// issued and only read the scratchpads once the conversion time has elapsed.
bool conversionInProgress = false;
unsigned long conversionStart = 0;
unsigned long lastConversionRequest = 0;
uint16_t conversionTimeMs = 750; // recalculated in setup() from actual resolution

// ---- Software input filter state (one lastHigh timestamp per input) ----
unsigned long heat1LastHigh, heat2LastHigh, extcallLastHigh, extemergLastHigh;

// Retriggerable one-shot input filter: HIGH is latched the instant it's seen;
// LOW is only reported after INPUT_OFF_DELAY_MS of continuous LOW. Turn-on is
// therefore immediate; turn-off is delayed ~100 ms. That asymmetry is
// deliberate -- an opto on 24 VAC never holds HIGH continuously, so a
// "stable both ways" debounce would never settle.
bool readFilteredInput(int pin, unsigned long &lastHigh, unsigned long now) {
  if (digitalRead(pin) == HIGH) {
    lastHigh = now;
    return true;
  }
  return (now - lastHigh) < INPUT_OFF_DELAY_MS;
}

void setup() {
  // If we got here via a watchdog reset, the WDT is still running with a
  // short timeout -- turn it off first so a slow setup can't reset-loop.
  // (Requires an optiboot-style bootloader, standard on Uno R3. Very old
  // bootloaders, e.g. original Duemilanove, don't clear the WDT flag and can
  // boot-loop after the first watchdog reset -- verify before relying on it.)
  wdt_disable();

  // Boot-state fix for the sekdiy Relay quirk (see library notes at top):
  // explicitly drive every relay pin HIGH (= OFF on this LOW-active board).
  // The constructors already ran pinMode(OUTPUT) but skipped the initial pin
  // write, which would otherwise leave all relays ENERGIZED right now. Done
  // first thing in setup() to minimize that window; the gap from MCU reset
  // to here still exists, so the relay board's onboard pull-ups matter
  // (verify the board has them -- most opto-isolated boards do).
  digitalWrite(PIN_INTPUMP,   HIGH);
  digitalWrite(PIN_EXTPUMP,   HIGH);
  digitalWrite(PIN_HEATPUMP,  HIGH);
  digitalWrite(PIN_AUX_UPPER, HIGH);
  digitalWrite(PIN_AUX_LOWER, HIGH);

  Serial.begin(9600);

  // Each pin needs its own pinMode() call. The previous comma-operator
  // version only actually set HEAT1 -- HEAT2, EXTCALL, and EXTEMERG were
  // silently left unconfigured (harmless only because INPUT is the AVR
  // power-on default).
  pinMode(HEAT1, INPUT);
  pinMode(HEAT2, INPUT);
  pinMode(EXTCALL, INPUT);
  pinMode(EXTEMERG, INPUT);

  // Start the input filters "expired" so an input that's LOW at boot reads
  // LOW immediately instead of reading HIGH for the first 100 ms (unsigned
  // math makes the subtraction wrap correctly even when millis() is near 0).
  unsigned long bootNow = millis() - INPUT_OFF_DELAY_MS;
  heat1LastHigh = heat2LastHigh = extcallLastHigh = extemergLastHigh = bootNow;

  // Initialize OneWire bus
  sensors.begin();  // initialize temp sensers

  // Non-blocking mode: requestTemperatures() returns immediately instead of
  // blocking loop() for the whole conversion (~750 ms at 12-bit). loop()
  // tracks the conversion time itself and reads results when they're ready.
  sensors.setWaitForConversion(false);
  conversionTimeMs = sensors.millisToWaitForConversion(sensors.getResolution());

#if SENSOR_DISCOVERY_MODE
  discoverSensors();
#endif

  //initialize lcd screen
  lcd.init();
  lcd.clear();
  // turn on the backlight
  lcd.backlight();
  drawStaticLabels(); // labels drawn once here; display() only rewrites values

  // Watchdog: if loop() ever hangs for 2 s (classic culprit: I2C lockup from
  // a flaky LCD), reset instead of leaving relays latched in their last
  // state. Safe to enable only because the boot path above initializes all
  // temps as DISCONNECTED -- a watchdog reset can't fire aux heat off
  // garbage readings. Note the reset also restarts the 5-minute compressor
  // power-up delay (see COMPRESSOR_MIN_OFF_MS).
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset(); // feed the watchdog every pass
  unsigned long now = millis();

  // ---- Non-blocking temperature updates ----
  // Phase 1: kick off a conversion on an interval. Returns immediately.
  if (!conversionInProgress && (now - lastConversionRequest >= SENSOR_UPDATE_INTERVAL_MS)) {
    sensors.requestTemperatures(); // non-blocking (setWaitForConversion(false))
    conversionStart = now;
    lastConversionRequest = now;
    conversionInProgress = true;
  }
  // Phase 2: once the conversion time has elapsed, read the results.
  // (Reading 4 scratchpads still costs a few tens of ms of OneWire traffic
  // once per second -- fine; the ~750 ms busy-wait is what's gone.)
  if (conversionInProgress && (now - conversionStart >= conversionTimeMs)) {
    conversionInProgress = false;
    readtemps();
    display();
  }

  // ---- Inputs (software-filtered; see readFilteredInput) ----
  bool callingForHeat = readFilteredInput(HEAT1, heat1LastHigh, now);
  // EXTCALL: the heat pump compressor's own call for the external water pump
  // (glycol loop). The glycol pump must run whenever the compressor demands
  // flow, even if the thermostat heat call has already dropped.
  bool extPumpCall = readFilteredInput(EXTCALL, extcallLastHigh, now);
  // EXTEMERG: the heat pump compressor's call for Emergency Backup Heat.
  bool extEmergCall = readFilteredInput(EXTEMERG, extemergLastHigh, now);
  // HEAT2 is read every pass (not just during a heat call) so its filter
  // timestamp can't go stale -- previously a heat call resuming within
  // 100 ms of dropping could see a phantom stage-2 call from the old latch.
  bool callingForStage2 = readFilteredInput(HEAT2, heat2LastHigh, now);

  // ---- Software tank high-limit (see TANK_MAX_C) ----
  if (TANK_TOP_C_t0 != DEVICE_DISCONNECTED_C) {
    if (!tankOverTemp && TANK_TOP_C_t0 >= TANK_MAX_C) {
      tankOverTemp = true;
      Serial.println(F("HIGH-LIMIT: tank top over max temp -- compressor and aux heat forced off"));
    } else if (tankOverTemp && TANK_TOP_C_t0 < TANK_MAX_RESUME_C) {
      tankOverTemp = false;
      Serial.println(F("HIGH-LIMIT cleared: tank back below resume threshold"));
    }
  }
  // On sensor loss the latch holds: a tripped high-limit can only clear on a
  // valid reading below the resume threshold.

  // ---- Tank element heat (aux_upper / aux_lower) ----
  // Shared tank-temp hysteresis latch: needs-heat below ELEMENT_ON_F,
  // satisfied at/above ELEMENT_OFF_F, holds in between. On sensor loss the
  // latch freezes; the two demand paths below treat the unknown differently.
  bool tankTempKnown = (TANK_TOP_C_t0 != DEVICE_DISCONNECTED_C);
  if (tankTempKnown) {
    if (TANK_TOP_F_t4 < ELEMENT_ON_F)        tankNeedsElementHeat = true;
    else if (TANK_TOP_F_t4 >= ELEMENT_OFF_F) tankNeedsElementHeat = false;
  }

  // Stage-2 AUX demand (thermostat-driven, firmware-supervised): heat call +
  // stage-2 call + tank verifiably cold. Fails OFF on sensor loss -- a
  // comfort top-up isn't worth heating blind. ASSUMPTION: "calling for
  // stage II" = HEAT2 input HIGH -- confirm this matches your wiring.
  bool stage2Demand = callingForHeat && callingForStage2 && tankTempKnown && tankNeedsElementHeat;

  // EXTEMERG demand (compressor-driven): the compressor's emergency backup
  // call. Not gated on the thermostat (the heat pump knows when it needs
  // backup -- defrost, low-ambient lockout), but gated by the same tank band
  // so it can't burn element heat into an already-hot tank. Deliberately
  // fails toward HEAT on sensor loss (follows the call): this is the
  // freeze-protection path, and the tank's mechanical high-limit is the
  // backstop. Forces BOTH elements -- upper now, lower when installed.
  bool extEmergDemand = extEmergCall && (tankNeedsElementHeat || !tankTempKnown);

  // !tankOverTemp gate here as well as in the override below so the relays
  // can't pulse on-then-off within one pass.
  if ((stage2Demand || extEmergDemand) && !tankOverTemp) {
    aux_upper.on();
  } else {
    aux_upper.off();
  }
  if (extEmergDemand && !tankOverTemp) {
    aux_lower.on();
  } else {
    aux_lower.off();
  }

  // ---- Heat call: compressor ----
  // (Tank elements are handled above; interior pump at the end of loop().)
  if (callingForHeat) {
    bool compressorReady = (now - lastHeatpumpOffTime >= COMPRESSOR_MIN_OFF_MS);
    // !tankOverTemp gate here (not just in the override below) so an
    // over-temp pass can't pulse the relay on-then-off within one loop.
    if (compressorReady && !tankOverTemp) {
      if (!heatpump.isOn()) {
        lastHeatpumpOnTime = now; // OFF->ON transition: start min-run timer
      }
      heatpump.on();
    }
    // else: still inside the mandatory off-time (or high-limit lockout) --
    // extpump runs alone until it's safe to start the compressor again.

  } else {
    // Not calling for heat.
    if (heatpump.isOn()) {
      bool minRunMet = (now - lastHeatpumpOnTime >= COMPRESSOR_MIN_RUN_MS);
      if (minRunMet) {
        // Start the mandatory-off timer from the moment the compressor is
        // commanded off, not from whenever we happen to notice later.
        lastHeatpumpOffTime = now;
        heatpump.off();
      }
      // else: min-run hold -- keep the compressor running until it has
      // accumulated COMPRESSOR_MIN_RUN_MS, even though the call dropped.
      // glycolDemand below keeps the ext pump running during this hold.
    }
    // Note: tank elements and intpump are handled by their own blocks -- the
    // elements above (stage2Demand requires callingForHeat, so they drop out
    // of a stage-2 call naturally), intpump at the end of loop().
  }

  // ---- High-limit override ----
  // Runs AFTER the branches above so it also wins over the min-run hold
  // (which otherwise keeps the compressor running in the not-calling branch).
  if (tankOverTemp) {
    if (heatpump.isOn()) {
      lastHeatpumpOffTime = now; // forced off starts the min-off timer too
      heatpump.off();
    }
    aux_upper.off();
    aux_lower.off(); // element heat feeds the tank -- override the compressor's call too
    // Pumps deliberately keep running (below) to move heat out of the tank.
    // On tank-top sensor loss the compressor deliberately keeps running on a
    // heat call: the tank's physical (mechanical) temp limit is the backstop
    // when this software limit is blind.
  }

  // ---- External (glycol) pump ----
  // Runs whenever the thermostat calls for heat, the compressor asserts
  // EXTCALL, or the compressor is still running (min-run hold). The cooldown
  // timer only starts once ALL demand has dropped.
  bool glycolDemand = callingForHeat || extPumpCall || heatpump.isOn();

  if (glycolDemand) {
    extpump.on();
    currentState = STATE_HEATING;
  } else if (extpump.isOn()) {
    if (currentState != STATE_COOLDOWN) {
      // Just transitioned out of active demand -- start the ext pump
      // cooldown timer instead of blocking here with delay(60000) like the
      // original version (a renewed heat call or EXTCALL during the minute
      // is still caught, since demand is re-checked every pass).
      extpumpCooldownStart = now;
      currentState = STATE_COOLDOWN;
    } else if (now - extpumpCooldownStart >= EXTPUMP_COOLDOWN_MS) {
      extpump.off();
      currentState = STATE_IDLE;
    }
  } else {
    currentState = STATE_IDLE;
  }

  // ---- Interior pump ----
  // Evaluated every pass, not just during a heat call, so it can't hold a
  // stale state during EXTCALL-only operation (e.g. a long defrost) or the
  // cooldown wind-down. Runs while the system is active (ext pump on) and
  // the tank is hot enough to be worth circulating: on above INTPUMP_ON_C,
  // off below INTPUMP_OFF_C, hold in between. Sensor loss = OFF (fail safe),
  // and it always stops when the ext pump stops (i.e. it runs through the
  // whole wind-down, per the "run until extpump stops" behavior).
  if (!extpump.isOn()) {
    intpump.off();
  } else if (TANK_TOP_C_t0 == DEVICE_DISCONNECTED_C) {
    intpump.off();
  } else if (TANK_TOP_C_t0 > INTPUMP_ON_C) {
    intpump.on();
  } else if (TANK_TOP_C_t0 < INTPUMP_OFF_C) {
    intpump.off();
  }
  // else: between thresholds -- hold current state (hysteresis)

  // ---- Display status (see the legend at displayStatus's declaration) ----
  // Computed last, from the relay states the passes above just decided.
  if (tankOverTemp) {
    displayStatus = "HILIM";
  } else if (heatpump.isOn() || aux_upper.isOn() || aux_lower.isOn()) {
    displayStatus = "HEAT";  // some heat source is actually running
  } else if (callingForHeat) {
    displayStatus = "PRE";   // call active, compressor waiting on min-off
  } else if (extPumpCall) {
    displayStatus = "EXT";   // compressor's own glycol demand, no heat call
  } else if (currentState == STATE_COOLDOWN) {
    displayStatus = "COOL";
  } else {
    displayStatus = "IDLE";
  }
}

void readtemps() { // reads results of an already-completed conversion; the
                   // request itself is issued non-blocking from loop().
  TANK_TOP_C_t0    = sensors.getTempC(TANK_TOP_ADDR);
  TANK_BOTTOM_C_t1 = sensors.getTempC(TANK_BOTTOM_ADDR);
  EXT_IN_C_t2      = sensors.getTempC(EXT_IN_ADDR);
  INT_OUT_C_t3     = sensors.getTempC(INT_OUT_ADDR);

  // Propagate DISCONNECTED instead of converting it (toFahrenheit(-127)
  // would otherwise yield -196.6 and hide the sentinel from any F-side check).
  TANK_TOP_F_t4    = (TANK_TOP_C_t0    == DEVICE_DISCONNECTED_C) ? DEVICE_DISCONNECTED_C : DallasTemperature::toFahrenheit(TANK_TOP_C_t0);
  TANK_BOTTOM_F_t5 = (TANK_BOTTOM_C_t1 == DEVICE_DISCONNECTED_C) ? DEVICE_DISCONNECTED_C : DallasTemperature::toFahrenheit(TANK_BOTTOM_C_t1);
  EXT_IN_F_t6      = (EXT_IN_C_t2      == DEVICE_DISCONNECTED_C) ? DEVICE_DISCONNECTED_C : DallasTemperature::toFahrenheit(EXT_IN_C_t2);
  INT_OUT_F_t7     = (INT_OUT_C_t3     == DEVICE_DISCONNECTED_C) ? DEVICE_DISCONNECTED_C : DallasTemperature::toFahrenheit(INT_OUT_C_t3);

  // Kept in sync (rounded) purely so display() keeps working; no longer used
  // to decide which sensor is which -- that's done by ROM address above.
  tArray[0] = round(TANK_TOP_C_t0);
  tArray[1] = round(TANK_BOTTOM_C_t1);
  tArray[2] = round(EXT_IN_C_t2);
  tArray[3] = round(INT_OUT_C_t3);

  // DEVICE_DISCONNECTED_C (-127) means that named address hasn't been filled
  // in yet (see the *_ADDR TODOs near the top) or the sensor isn't
  // responding. Warnings print once per state CHANGE (lost/recovered), not
  // every second, so the serial log stays readable.
  checkSensor(TANK_TOP_C_t0,    0, F("TANK_TOP"));
  checkSensor(TANK_BOTTOM_C_t1, 1, F("TANK_BOTTOM"));
  checkSensor(EXT_IN_C_t2,      2, F("EXT_IN"));
  checkSensor(INT_OUT_C_t3,     3, F("INT_OUT"));
}

void checkSensor(float tempC, uint8_t idx, const __FlashStringHelper* name) {
  static bool wasDisconnected[4] = {false, false, false, false};
  bool disconnected = (tempC == DEVICE_DISCONNECTED_C);
  if (disconnected && !wasDisconnected[idx]) {
    Serial.print(F("WARNING: "));
    Serial.print(name);
    Serial.println(F(" sensor not responding -- check its *_ADDR entry and wiring"));
  } else if (!disconnected && wasDisconnected[idx]) {
    Serial.print(name);
    Serial.println(F(" sensor recovered"));
  }
  wasDisconnected[idx] = disconnected;
}

// Print an integer left-justified, space-padded to a fixed field width, so a
// shorter value fully overwrites a longer previous one (no ghost digits when
// e.g. -127 is replaced by 23).
void lcdPrintPadded(int value, uint8_t width) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", value);
  uint8_t len = 0;
  while (buf[len] != '\0' && len < width) {
    lcd.print(buf[len]);
    len++;
  }
  for (; len < width; len++) lcd.print(' ');
}

// Temperature field: shows "---" when the sensor is disconnected/unset
// instead of a confusing -127.
void lcdPrintTemp(int valueC) {
  if (valueC == DEVICE_DISCONNECTED_C) lcdPrintPaddedStr("---", 4);
  else lcdPrintPadded(valueC, 4);
}

void lcdPrintPaddedStr(const char* s, uint8_t width) {
  uint8_t len = 0;
  while (s[len] != '\0' && len < width) {
    lcd.print(s[len]);
    len++;
  }
  for (; len < width; len++) lcd.print(' ');
}

// Static labels are drawn once at startup; display() below only rewrites the
// value fields. A full 50-character redraw through the I2C backpack costs
// tens of ms of blocking time every second -- this cuts that by more than half.
// 20x4 layout (columns 0-19):
//   row 0: "TT nnnn"+deg+"C  IN nnnn"+deg+"C"
//   row 1: "TB nnnn"+deg+"C  OUT nnnn"+deg+"C"
//   row 2: "INT:xxx   Stat:xxxxx"
//   row 3: "EXT:xxx   Mode:xxxxx"
void drawStaticLabels() {
  lcd.setCursor(0,0);  lcd.print(F("TT "));
  lcd.setCursor(7,0);  lcd.print((char)176); lcd.print(F("C")); // degree symbol -- verify against your LCD's character ROM; some HD44780 variants use 0xDF (223) instead
  lcd.setCursor(0,1);  lcd.print(F("TB "));
  lcd.setCursor(7,1);  lcd.print((char)176); lcd.print(F("C"));
  lcd.setCursor(10,0); lcd.print(F("IN "));
  lcd.setCursor(17,0); lcd.print((char)176); lcd.print(F("C"));
  lcd.setCursor(10,1); lcd.print(F("OUT "));
  lcd.setCursor(18,1); lcd.print((char)176); lcd.print(F("C"));
  lcd.setCursor(0,2);  lcd.print(F("INT:"));
  lcd.setCursor(0,3);  lcd.print(F("EXT:"));
  lcd.setCursor(10,2); lcd.print(F("Stat:"));
  lcd.setCursor(10,3); lcd.print(F("Mode:"));
}

void display(){
  // Temperature values (fixed-width fields; labels/degree symbols are static)
  lcd.setCursor(3,0);  lcdPrintTemp(tArray[0]); // TT
  lcd.setCursor(3,1);  lcdPrintTemp(tArray[1]); // TB
  lcd.setCursor(13,0); lcdPrintTemp(tArray[2]); // IN
  lcd.setCursor(14,1); lcdPrintTemp(tArray[3]); // OUT

  // Pump status
  lcd.setCursor(4,2); lcdPrintPaddedStr(intpump.isOn() ? "ON" : "OFF", 3);
  lcd.setCursor(4,3); lcdPrintPaddedStr(extpump.isOn() ? "ON" : "OFF", 3);

  // System status (computed at the end of loop(); legend at its declaration)
  lcd.setCursor(15,2); lcdPrintPaddedStr(displayStatus, 5);

  // Active heat source(s) -- 5-char field:
  //   EXTEM = compressor emergency backup (both elements forced),
  //   HP+AX = compressor + upper element (stage-2), HP, AUX (upper element
  //   alone), OFF
  const char* mode;
  if (aux_lower.isOn())                             mode = "EXTEM";
  else if (heatpump.isOn() && aux_upper.isOn()) mode = "HP+AX";
  else if (heatpump.isOn())                         mode = "HP";
  else if (aux_upper.isOn())                        mode = "AUX";
  else                                                  mode = "OFF";
  lcd.setCursor(15,3); lcdPrintPaddedStr(mode, 5);
}

void printAddress(DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print('0');
    Serial.print(addr[i], HEX);
    if (i < 7) Serial.print(' ');
  }
}

void discoverSensors() {
  int sensorcount = sensors.getDeviceCount(); // Get number of sensors on bus
  Serial.print(F("Found "));
  Serial.print(sensorcount);
  Serial.println(F(" DS18B20 sensor(s) on the bus:"));
  DeviceAddress addr;
  for (int i = 0; i < sensorcount; i++) {
    if (sensors.getAddress(addr, i)) {
      Serial.print(F("  Sensor "));
      Serial.print(i);
      Serial.print(F(": "));
      printAddress(addr);
      Serial.println();
    }
  }
  Serial.println(F("Copy the address(es) above into the *_ADDR arrays near the top of this file. Warm/cool one sensor at a time (e.g. hold it in your hand) and re-run discovery to tell which physical sensor is which."));
}

# heatpump_control

Arduino Uno R3 controller for an air-to-water heat pump + buffer tank system: it reads DS18B20 tank/loop temperatures and thermostat call inputs, then drives a LOW-trigger relay board (interior pump, exterior pump, compressor enable, tank elements), with min-run/min-off timers, a software tank high-limit, a watchdog, and a 20x4 I2C LCD status display.

## Layout

| Path | What |
|---|---|
| `firmware/heatpump/` | The sketch (`heatpump.ino`) + bundled sekdiy `Relay` library. Open this folder in the Arduino IDE. |
| `firmware/heatpump/archive/` | Older sketch revisions, kept for reference (`.txt` so the IDE ignores them). |
| `hardware/kicad/` | KiCad schematic, project and symbol library. |
| `hardware/Heatpump_Schematic.pdf` | Exported schematic. |
| `hardware/Pinout.ods` | Arduino pin / relay channel map. |
| `docs/TODO.md` | Open TODOs and a dated decision log. **Read before changing behavior**: it explains the fail-safe directions, high-limit values, relay polarity, etc. |

## Building

Libraries (Arduino Library Manager):

- OneWire 2.3.8
- DallasTemperature 4.0.6
- LiquidCrystal I2C 1.1.2

```
arduino-cli compile --fqbn arduino:avr:uno firmware/heatpump
```

## Workflow: develop here, release to the running copy

- Make changes **in this repo** on a branch (`git switch -c feature/xyz`), and merge to `main` once they're tested.
- When a build is flashed to the controller, tag it (`git tag -a vX.Y -m "..."`, then `git push --tags`). The latest tag is the record of what's running.
- The deployed "running version" copy kept outside this repo is updated **only from a tagged release** (`git archive vX.Y firmware/heatpump`) and is never edited in place.

## Datasheets

The vendor manuals aren't redistributed here. The relay board is the Handson Technology 8-channel 5V opto-isolated LOW-level relay module.

## License

CC0 1.0. See `LICENSE`. `Relay.h`/`Relay.cpp` are from [sekdiy/Relay](https://github.com/sekdiy/Relay) under their original license.

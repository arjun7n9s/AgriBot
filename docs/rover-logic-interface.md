# Rover logic interface: HH-8130K-RXT2 7-pin header

This note covers how our own microcontroller can operate the stock rover board through its 7-pin header. Every command in it comes from the bench results in [“Confirmed on the bench”](rover-control-board.md#confirmed-on-the-bench). The photos are used only for the header's orientation and pin pitch. Anything the bench did not test is marked unknown.

The header behaves as contact closures, not a data bus. Tejesh got two behaviours by shorting two pins together. Our microcontroller does the same with an electronic switch that closes the same two pins. It sends no data and does not change the board's firmware.

## What the bench established

- **Pin order**, as Tejesh read it: `+` `V` `ROCK` `−` `B` `F` `S`.
  - The older photo `IMG_20261004_010740_824.jpg` shows labels in the same order along the board edge beside the header, partly cut off at the edge. `+` is at the pin nearest the yellow F1 disc, and `S` is at the end nearest the `HH-8130K-RXT2` text. The names come from Tejesh's reading; the photo is used only for which end is which. This note counts positions 1 to 7 from the `+` end.
  - The pin pitch measured from the photos is 2.5 mm.
- **`−` to `B`:** shorting them makes the motors run backwards.
- **`+` to `V`:** shorting them enables wireless control from the handset. No other header pin has to be connected for that.
- **`ROCK`, `F`, `S`:** no tested function.

**Not established:**

- the voltage on any header pin, and the battery voltage
- which pin of each pair is the more positive, and how much current flows through each short
- what `V` is electrically
- what happens when a short is removed: whether the motors stop, and whether wireless control ends
- whether `+` to `V` was closed during the `−` to `B` test, and what both shorts do together
- which wins when the handset and a header short ask for different things

## The interface: two switched channels

`HANDSET_EN` and `REVERSE` are names used in this note, not silkscreen.

| Channel | Closes | Bench result |
| --- | --- | --- |
| `HANDSET_EN` | positions 1 and 2, `+` to `V` | Wireless control from the handset works. |
| `REVERSE` | positions 4 and 5, `−` to `B` | Motors run backwards. |
| none | positions 3, 6, 7: `ROCK`, `F`, `S` | No tested function. Left unconnected. |

## Rules for every circuit below

- Never connect a GPIO, 3.3 V, 5 V, or the microcontroller's ground to a header pin. The header voltage is unknown, so is the battery voltage, and the board's reference may not be our ground.
- Each channel gets its own switch. The header side of that switch touches its own two pins and nothing else.
- `ROCK`, `F`, and `S` stay unconnected.
- During bring-up, power the microcontroller from its own supply, such as USB.
- Any test that can move the motors runs with the drive wheels off the ground and a battery disconnect within reach.

## Measure before choosing parts

Use a multimeter, with the board powered as it was on the bench and nothing else connected to the header.

1. **Open-circuit voltage across each pair.** DC volts. Put the red probe on the first pin named (`+`, then `−`) and the black probe on the second (`V`, then `B`). A negative reading means the second pin is the more positive one. If the reading wanders or sits near 0 V, note that too.
   - Decides: the voltage rating of the switch, and which way round an optocoupler output goes.
2. **Current through each short.** Fused DC mA range, probes across the pair. This is the short itself, so `−` to `B` will run the motors backwards: wheels off the ground. Check that the board still responds with the meter in the path.
   - Decides: whether a phototransistor optocoupler is enough, or a PhotoMOS-type relay or a relay is needed.
3. **Each header pin against battery negative.** DC volts, black probe on the battery's negative terminal, red probe on each of the seven pins in turn. A voltmeter shorts nothing, so this is safe on every pin. The header's `−` is only a name until this shows whether it sits at battery negative.
   - Decides: whether a plain transistor sharing our ground is possible for either pair.

## How the microcontroller replaces a short

### Default: one optocoupler per channel

The GPIO drives the optocoupler's LED. The optocoupler's output transistor sits across the two header pins and closes them while the LED is on. Nothing on the microcontroller side touches the board, so the unknown header voltage never reaches the GPIO.

```
 microcontroller side                          header side (one pair only)
                        +----------------------+
 GPIO --+---[ R1 ]----->| LED anode  collector |----- more positive pin of the pair
        |               |                      |
     [ R2 ]             |     optocoupler      |
        |               |                      |
 GND ---+-------------->| LED cathode  emitter |----- other pin of the pair
                        +----------------------+
```

- **R1** sets the LED current to about 5 mA: about 470 Ω from a 3.3 V GPIO, or about 820 Ω from a 5 V GPIO. Check that this is within the GPIO's rated current.
- **R2**, 10 kΩ, keeps the LED off while the microcontroller is resetting or the pin is not yet set up.
- **Polarity:** the collector goes to the pin that measured more positive. A phototransistor conducts one way only and tolerates very little reverse voltage. If measurement 1 was unclear, use a polarity-free switch instead.
- **Voltage:** pick a part whose collector-emitter rating is well above the measured voltage. PC817-class parts are a common choice.
- **Current:** at this LED current, a phototransistor output suits a short that carries about 1 mA or less. If measurement 2 is higher, use a polarity-free switch rated for it.
- **Bench check:** a closed phototransistor still has a few tenths of a volt across it. Confirm that the board responds through the optocoupler exactly as it did to the direct short.

### Polarity-free alternatives

Use one of these if a pair's polarity is unclear, its reading wanders, the current is too high for a phototransistor, or the board does not respond through the optocoupler.

- **MOSFET-output solid-state relay (PhotoMOS-type)**, for example Toshiba TLP222A or a Panasonic AQY21x-series part. Its LED is driven like the optocoupler's; check the datasheet's LED current. The output conducts in both directions with low on-resistance. Check its load voltage and current ratings against measurements 1 and 2.
- **Small relay with a dry contact.** Wire the pair to COM and NO only. A metal contact is the closest match to a direct short and works either way round. It needs a transistor driver and a flyback diode across the coil, unless a ready-made module includes them. It is slower and audible. If you use a module, confirm the contact stays open when the module's input is low, floating, or unpowered.

### Plain transistor: only after measurement 3

A transistor can replace a short only if one pin of the pair sits at battery negative and the other sits positive while open. The microcontroller's ground then joins battery negative at that pin.

```
 microcontroller side                    header side (one pair only)
 GPIO --[ R1 2.2k ]--+------ base        collector ----- pin that sits positive when open
                     |             NPN
                  [ R2 10k ]             emitter ---+--- pin that sits at battery negative
                     |                              |
 GND ----------------+------------------------------+
```

- Use a small NPN (2N3904-class) or a logic-level N-MOSFET, rated well above the measured voltage and current.
- The shared ground lets motor noise and fault currents reach our electronics.
- If measurement 3 shows that neither pin of a pair sits at battery negative, this circuit cannot be used for that pair. If in doubt, use the optocoupler.

### Connecting to the header

- Plug onto the existing header instead of soldering to the board. Confirm the mating connector against the part itself.
- Fit wires only at positions 1 (`+`), 2 (`V`), 4 (`−`), and 5 (`B`). Leave positions 3, 6, and 7 empty.
- Before power-up, check each wire against the silkscreen beside the header.

## Firmware rules

- Both channels start open. Drive both GPIOs low before anything else runs, avoid pins that are pulled high or toggled during boot, and keep R2 fitted.
- `REVERSE` is a hold with a time limit. It opens after a set time unless the control loop renews it. A crash, a watchdog reset, or a lost link to the main controller must leave both channels open.
- Do not pulse, PWM, or rapidly toggle a channel. How the board responds to that is untested.
- Do not close both channels together until that has been tried on the bench.
- Opening a channel only returns the header to its unconnected state. Do not treat it as a stop until the release test (item 2 below) shows that it is one.

## What our logic can command today

These are the only commands the bench supports.

| Command | Our logic does | Bench result | Still unverified |
| --- | --- | --- | --- |
| Enable the handset | Close `HANDSET_EN` and keep it closed | Wireless control from the handset works | Whether opening it ends wireless control |
| Reverse | Close `REVERSE` while reverse is wanted | Motors run backwards | Whether opening it stops the motors at once; the speed; whether `HANDSET_EN` must also be closed |

That is the whole command set. Our logic can start reverse, and it can enable the handset so a person can drive. Whether it can end reverse by opening `REVERSE` waits on the release test.

## What we still cannot command

- **Forward.** No tested short produces forward.
- **Stop.** No short is known to stop the motors. Until the release test is done, the only known stop is cutting battery power.
- **Steering.** The board drives the steering motor, but no tested short steers.
- **Speed.** Shorts are on or off. No speed input has been tested.
- **`ROCK`, `F`, `S`.** No tested function, and none is assigned here.
- **Turning the handset off or overriding it.** Untested.
- **Any feedback.** The header gives nothing back. Our logic cannot read speed, position, or whether a command took effect.

Driving rows on our own needs at least forward, steering, and a dependable stop. None of those is available through this header today. The handset already runs all the motors. If its controls turn out to be simple switches, closing them the same way is another possible route. That needs the handset's circuit side photographed first, and it is not covered here.

## Need from Tejesh

1. **The three readings in “Measure before choosing parts”.**
   - Why it matters: they choose the switch for each channel and show whether a plain transistor is possible.
   - Settles it: the readings, with the meter range used for each.
2. **What removing each short does.**
   - Why it matters: if opening `−` to `B` stops the motors at once, our logic has a stop for reverse. If opening `+` to `V` ends wireless control, our logic can take control back from the handset.
   - Settles it: close and open each pair by hand with the wheels off the ground, and say what the motors and the handset do.
3. **Whether `REVERSE` needs `HANDSET_EN`, and what both together do.**
   - Why it matters: it decides whether our logic closes one channel or both to reverse, and whether both at once is safe.
   - Settles it: short `−` to `B` with `+` to `V` open, then again with it closed, and note any difference.
4. **Handset versus header.**
   - Why it matters: it decides who is in control when both act. That matters for every test where a person holds the handset.
   - Settles it: with both shorts closed and the wheels off the ground, press each handset control and say what happens.
5. **`ROCK`, `F`, and `S`.**
   - Why it matters: forward, steering, and stop are not among the tested shorts. If this header can give them at all, it has to be through an untested pin or pairing.
   - Settles it: first, with a voltmeter only, read each of these pins against `+` and against `−`, and watch whether any reading changes while the handset drives. Then, with the wheels off the ground, try one pin at a time to `−` and then to `+` through a 1 kΩ resistor, only for a moment, and say what happens. Do not short `+` to `−`. Nobody has measured them, and if they are supply connections that is a short circuit.
6. **Battery voltage.**
   - Why it matters: it is a first guide to the ratings the switch parts need, before the header itself is measured.
   - Settles it: the battery label, or a meter reading across the board's `+/−` blade tabs.

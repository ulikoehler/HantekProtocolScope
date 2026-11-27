# UliCAN quick guide

UliCAN turns a two-channel Hantek DSO-6022BL into a live Classic CAN
oscilloscope and decoder. It displays CANH and CANL, calculates the differential
signal (`CANH - CANL`), marks trigger positions and decoded frames, and lists
the received CAN messages.

## Start a live capture

Connect channel 1 to CANH, channel 2 to CANL, and the scope ground to the bus
ground. Then run:

```bash
./build/ulican live --window-ms 0.6 --trigger sof
```

The trigger is placed 10% from the left by default. Set another position as a
fraction of the graph width with, for example, `--trigger-position 0.25`.

The upper plot shows CANH and CANL. The lower plot shows their differential
voltage and decoded frame annotations. The table at the bottom contains CAN
IDs, payloads, CRC/ACK status, and measured voltage levels.

## Graph colors

- **Gold/yellow** is CANH and **teal/cyan** is CANL.
- **Dark and light blue** show the minimum and maximum differential voltage
  (`CANH - CANL`) within each displayed time interval.
- The dark/light shades of each signal are its minimum/maximum envelope when
  many samples must be combined into one screen column.
- **Orange** triangles and the orange dotted line mark trigger positions.
- The **red dashed** horizontal line is the adjustable trigger threshold;
  **gray dashed** lines are CAN voltage-reference guides.
- **Green** frame edges and labels mark valid decoded frames; **red** marks a
  frame with an ACK, delimiter, EOF, or dominant-state error.

## Main controls

- **Trigger V** sets the differential-voltage trigger threshold.
- **Holdoff** sets the minimum time between scope-display triggers. A longer
  holdoff makes the picture steadier; decoding and logging continue normally.
- **Bitrate** selects the CAN bitrate.
- **Trigger** selects start-of-frame (`sof`), error, or dominant triggering.
- **Trig auto/normal** chooses whether the display free-runs when no trigger is
  available or waits for a trigger.
- **Follow** keeps the newest accepted trigger at the configured horizontal
  position (10% from the left by default).
- **Y-Auto** automatically adjusts the voltage ranges.
- **Clear** clears the message list and rearms the view.
- **Eye** enables a persistence display. Each accepted trigger adds another
  trigger-aligned CANH, CANL, and differential sweep. Common paths become
  brighter while unusual voltage levels, noise, and timing excursions remain
  visible. Up to 256 sweeps are retained; **Clear** starts a new eye.
- **Graphs** cycles between both graphs, only the CANH/CANL signals, and only
  the differential graph.

Use the mouse wheel over a plot to change the time zoom. Press **Space** to
freeze/resume, **E** to toggle the eye display, **R** to reset the view, **F**
for fullscreen, and **Q** to quit. Changing the timebase or trigger settings
clears the accumulated eye because the old sweeps no longer share the same
display reference.

## Recording and replay

Record raw samples and decoded CAN events with:

```bash
./build/ulican live --log-raw capture.bin --log-can frames.jsonl
```

Replay a raw recording without the oscilloscope connected:

```bash
./build/ulican view capture.bin
```

Run `./build/ulican live --help` for all capture options.

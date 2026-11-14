# marsemu

Runs a Marstek battery's own firmware in an emulator, with models of the
hardware around it, and shows the inside of the device in the **marsemu**
panel in the sidebar: power flowing between the parts, the traffic on every
bus, the cells, and the registers the firmware writes.

The emulated battery is on your network like a real one: it connects to Home
Assistant's MQTT broker with its real topics (hm2mqtt finds it there), and it
steers against a CT002 meter, built in or AstraMeter.

## Firmware

marsemu ships no firmware. Use the firmware of a device you own: open the
panel and drop the image file into it, or copy it into the add-on's folder
`/addon_configs/<id>_marsemu/firmware` (the Samba or File editor add-on reach
it). The B2500 and the Venus E 3.0 (control and BMS image) are supported for now;
the other devices follow.

## Options

- **Firmware image**: the file to start. Empty: the only supported image in
  the folder, or pick one in the panel.
- **MQTT**: `home_assistant` (needs the Mosquitto broker add-on) or `offline`.
- **House load sensor**: a power sensor with your household consumption. The
  emulated house then follows it, so you can see what the battery would do in
  your house.
- **House load**, **PV 1**, **PV 2**, **State of charge**, **Extension packs**:
  the starting point when no sensor is set.
- **Speed limit**: 1 is real time. A Raspberry Pi may run slower than that.
- **CT meter**: `local`, or `HOST:PORT` of a CT002 emulator such as AstraMeter.

The emulated device uses its own identity (MCU ID, MAC). It does not take
over your real battery's topics, and it never connects to Marstek's cloud.

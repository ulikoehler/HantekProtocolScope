# Hantek Protocol Scope

`ulican` is a C++23 application for working with Hantek scope devices. It uses
CMake and the Vulkan-based [VolcanoPlot](https://github.com/ulikoehler/VolcanoPlot)
submodule for its UI.

## Build (Debian/Ubuntu)

Install the compiler, CMake, and required development packages:

```bash
sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev \
    zlib1g-dev libvulkan-dev libglfw3-dev libshaderc-dev glslang-dev \
    spirv-tools libpng-dev libwebp-dev libfreetype-dev libharfbuzz-dev
```

Clone the repository including submodules (or initialise them in an existing
checkout), then configure and build:

```bash
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The executable is written to `build/ulican`. Its DSO-6022BL firmware is copied
from the `Hantek6022API` submodule into `firmware/` during the build, so start
a live capture with:

```bash
./build/ulican
```

Use `./build/ulican --help` to list commands and `./build/ulican live --help`
to see capture options. To replay a raw capture created with `--log-raw`, use
`./build/ulican view capture.bin`.

## USB access (Linux)

Allow non-root access in both the firmware-loader and running-device modes:

```bash
sudo install -Dm644 udev/60-ulican.rules /etc/udev/rules.d/60-ulican.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb \
    --attr-match=idVendor=04b5 --attr-match=idProduct=602a
sudo udevadm settle
```

Alternatively, unplug and reconnect the scope after installing the rule. The
user running `ulican` must be a member of the `plugdev` group.

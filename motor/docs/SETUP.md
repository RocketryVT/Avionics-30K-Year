# Setup

Once per machine. The Pico program is built in WSL (Ubuntu); the laptop
program runs on Windows.

## 1. WSL: the Pico build tools

```bash
sudo apt install -y git cmake ninja-build build-essential python3 libusb-1.0-0-dev pkg-config

# ARM compiler
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-aarch64-arm-none-eabi.tar.xz
sudo mkdir -p /opt/arm
sudo tar -xJf arm-gnu-toolchain-15.2.rel1-aarch64-arm-none-eabi.tar.xz -C /opt/arm

# Pico SDK
git clone --branch 2.3.1 --depth 1 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
git -C ~/pico/pico-sdk submodule update --init --depth 1

echo 'export PICO_SDK_PATH="$HOME/pico/pico-sdk"' >> ~/.bashrc
echo 'export PATH="/opt/arm/arm-gnu-toolchain-15.2.rel1-aarch64-arm-none-eabi/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

On an Intel or AMD PC, change `aarch64` to `x86_64` everywhere above.

## 2. Windows: the laptop tools

From the repo folder:

```powershell
winget install Python.Python.3.13
py -m pip install -e laptop
```

## 3. Build, flash and run

Follow the Quick start in the README. In WSL the repo folder is under
`/mnt/c/`, for example `/mnt/c/Users/<you>/OneDrive/Desktop/<repo>`.

## If something goes wrong

| What you see | Fix |
| --- | --- |
| `wsl` opens Docker instead of Ubuntu | Use `wsl -d Ubuntu-24.04` |
| `py` is not recognised | Close Windows Terminal completely and reopen it |
| `No serial ports at all` | Use a data USB cable, not a charge-only one, and check the `.uf2` copied (the drive disappears when it has) |
| `Port busy` | Close any other program using the port, such as a VS Code serial monitor |
| `More than one Pico found` | Say which with `--port COM5` |

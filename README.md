# AirBeam Mini Firmware

Firmware for the AirBeam Mini, built using Rust and `esp-idf-svc`.
Target MCU: **ESP32-C3** (`riscv32imc-esp-espidf`)

## Prerequisites

Before building, ensure you have the necessary tools installed:

1.  **Rust Toolchain (Nightly)**
    The project uses the nightly toolchain with the `rust-src` component (configured in `rust-toolchain.toml`).
    ```bash
    rustup install nightly
    rustup component add rust-src --toolchain nightly
    rustup target add riscv32imc-unknown-none-elf --toolchain nightly
    ```

2.  **Build Tools (`ldproxy`)**
    This is required for linking.
    ```bash
    cargo install ldproxy
    ```

3.  **Flashing Tool (`espflash`)**
    ```bash
    cargo install espflash
    # Or using cargo-binstall if you have it:
    # cargo binstall espflash
    ```

4.  **ESP-IDF Prerequisites**
    The build process compiles ESP-IDF from source. You will need:
    - **Python 3.7+**
    - **CMake**
    - **Ninja**
    - **Git**

    *macOS:*
    ```bash
    brew install cmake ninja dfu-util python3
    ```

    *Linux (Ubuntu/Debian):*
    ```bash
    sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
    ```

## Setup

If you haven't already, creating the `esp-idf` local environment is often handled automatically by the build script, but using `espup` is a reliable way to ensure the environment is valid.

```bash
cargo install espup
espup install
# Source the exports file (add to your shell profile to make permanent)
. $HOME/export-esp.sh
```

## Compilation

To build the project:

```bash
cargo build
```

*Note: The first build will take significantly longer as it compiles the entire ESP-IDF framework.*

## Flashing & Monitoring

The project is configured to use `espflash` as the runner in `.cargo/config.toml`.

To build, flash, and monitor the serial output in one go:

```bash
cargo run
```

Or manually with `espflash`:

```bash
cargo espflash flash --monitor
```

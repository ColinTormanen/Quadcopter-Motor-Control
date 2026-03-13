# STM32F411 Quadcopter Motor Control

A bare-metal C implementation for quadcopter motor control using DShot and Bi-directional DShot protocols on the STM32F411 microcontroller.

## Features

- **DShot Protocol Implementation**: Full DShot600 support for high-speed motor control
- **Bi-directional DShot**: Telemetry support for reading motor data (RPM, voltage, current, temperature)
- **Quad Motor Control**: Simultaneous control of 4 motors using DMA for efficient PWM generation
- **Command Mode**: ESC programming and configuration commands (e.g., enable extended telemetry)
- **Bare-metal C**: No HAL libraries, direct register manipulation for optimal performance
- **DMA-based PWM**: Hardware-accelerated motor signal generation
- **Interrupt-driven**: Efficient handling of motor updates and telemetry reception

## Prerequisites

### Required Software
- **ARM GCC Toolchain**: `arm-none-eabi-gcc`
- **CMake**: Version 3.16 or higher
- **OpenOCD**: For debugging and flashing
- **GDB**: For debugging (regular `gdb` or `gdb-multiarch`)
- **Make**: Build system

### Installation (Arch Linux)
```bash
sudo pacman -S arm-none-eabi-gcc cmake openocd gdb make
```

### Installation (Ubuntu/Debian)
```bash
sudo apt install gcc-arm-none-eabi cmake openocd gdb-multiarch make
```

## Hardware Requirements
- STM32F411 development board (e.g., STM32F411CE "Black Pill")
- ST-Link V2 programmer/debugger
- 4x Brushless DC motors with ESCs supporting DShot
- Power supply appropriate for your motors (typically 11.1V-25.2V LiPo)
- USB cables for power and programming

## Hardware Connections

### ST-Link to STM32F411
| ST-Link Pin | STM32F411 Pin | Function |
|-------------|---------------|----------|
| SWDIO       | PA13          | SWD Data |
| SWCLK       | PA14          | SWD Clock|
| GND         | GND           | Ground   |
| 3.3V        | 3.3V          | Power    |

### Motor Connections
| Motor | STM32F411 Pin | Timer Channel | DMA Stream |
|-------|---------------|---------------|------------|
| Motor 1 | PB4          | TIM3 CH1     | DMA1 Stream 4 |
| Motor 2 | PB5          | TIM3 CH2     | DMA1 Stream 5 |
| Motor 3 | PB0          | TIM3 CH3     | DMA1 Stream 7 |
| Motor 4 | PB1          | TIM3 CH4     | DMA1 Stream 2 |

**Note**: Ensure ESCs are properly calibrated and armed before connecting motors. Always test with props removed first.

## Building the Project

### 1. Create Build Directory
```bash
mkdir build && cd build
```

### 2. Configure CMake
```bash
cmake ..
```

### 3. Build
```bash
make
```

This creates:
- `QuadcopterMotorControl` - ELF file with debug symbols (used for flashing and debugging)

## Flashing the Firmware

### Method 1: Using CMake Target (Recommended)
```bash
make flash
```

### Method 2: Manual OpenOCD Command
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program QuadcopterMotorControl verify reset exit"
```

## Debugging

### Start Debug Session
```bash
make debug
```

This will:
1. Start OpenOCD debug server on port 3333
2. Launch GDB connected to the target
3. Load the program and halt at reset

### Manual Debug Setup

**Terminal 1 - Start OpenOCD:**
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

**Terminal 2 - Connect GDB:**
```bash
gdb QuadcopterMotorControl
(gdb) target extended-remote localhost:3333
(gdb) load
(gdb) monitor reset halt
```

### Essential GDB Commands

#### Code Navigation
```bash
(gdb) layout next            # Show source code in TUI mode
(gdb) break main             # Set breakpoint at main function
(gdb) break main.c:25        # Set breakpoint at specific line
(gdb) continue               # Continue execution
(gdb) step                   # Step into functions
(gdb) next                   # Step over functions
```

#### Target Control
```bash
(gdb) monitor reset halt     # Reset and halt the MCU
(gdb) monitor reset          # Reset and run
(gdb) load                   # Load program to target
(gdb) continue               # Continue execution
```

#### Variable and Memory Inspection
```bash
(gdb) info locals            # Show local variables
(gdb) print variable_name    # Print specific variable
(gdb) print *pointer_name    # Dereference pointer
(gdb) print/x my_int         # Print in hexadecimal
(gdb) x/4x 0x20000000       # Examine 4 hex words at address
(gdb) info registers         # Show all registers
(gdb) print $r0              # Print specific register
```

#### Memory Addresses (STM32F411)
```bash
(gdb) x/8x 0x40020000        # GPIOA registers
(gdb) x/8x 0x40020800        # GPIOC registers (onboard LED)
(gdb) x/8x 0x20000000        # RAM start
(gdb) x/8x 0x08000000        # Flash start (vector table)
```

#### Exit Debug Session
```bash
(gdb) quit                   # Exit GDB
# Ctrl+C in OpenOCD terminal to stop server
```

## Usage

### Basic Motor Control

```c
#include "motor_control.h"

// Initialize motors
InitMotors();

// Start motor control (enables DMA and timers)
StartMotors();

// Set motor throttle (0-2047 range)
SetMotorThrottle(motor1, 500);  // 500/2047 throttle on motor 1
SetMotorThrottle(motor2, 750);  // 750/2047 throttle on motor 2

// Stop motors
StopMotors();
```

### Command Mode (ESC Programming)

```c
#include "dshot.h"

// Example: Enable extended telemetry on motor 1
uint16_t commands[] = {13};  // Command 13: Enable extended telemetry
uint8_t repeats[] = {6};     // Repeat 6 times for ESC to accept
ConstructCommandSequence(motor1, commands, repeats, 1);
```

### Telemetry Reading

The implementation includes structures for reading motor telemetry:
- RPM
- Voltage
- Current
- Temperature

*(Telemetry decoding implementation in progress)*

## Project Structure

```
├── CMakeLists.txt          # Build configuration
├── README.md              # This file
├── build/                 # Build artifacts (generated)
├── cmake/
│   └── stm32f411.cmake    # STM32F411 toolchain configuration
├── inc/                   # Header files
│   ├── core_cm4.h
│   ├── stm32f411xe.h
│   └── ...
├── linker/
│   └── STM32F411CEUx_FLASH.ld  # Linker script
└── src/                   # Source files
    ├── main.c             # Main application
    ├── motor_control.c    # Motor control functions
    ├── motor_control.h
    ├── dshot.c            # DShot protocol implementation
    ├── dshot.h
    ├── spi.c              # SPI for telemetry (future)
    ├── spi.h
    ├── system_stm32f4xx.c # System initialization
    ├── system.c           # Clock and peripheral setup
    ├── system.h
    └── startup_stm32f411xe.s  # Startup assembly
```

## Troubleshooting

### Build Issues
- **Missing toolchain**: Install ARM GCC toolchain
- **CMake errors**: Check CMake version (requires 3.16+)
- **Missing headers**: Ensure `inc/` directory exists with required headers

### Flashing Issues
- **ST-Link not detected**: Check USB connection and drivers
- **Permission denied**: Add user to `dialout` group or use `sudo`
- **Target not found**: Verify ST-Link connection to STM32

### Motor Control Issues
- **Motors not spinning**: Check ESC calibration and arming sequence
- **Noisy motors**: Verify DShot timing and DMA configuration
- **Telemetry not working**: Ensure ESCs support bi-directional DShot
- **Overheating**: Check power supply voltage and current limits

### Debug Issues
- **Cannot connect to target**: Check ST-Link connection and OpenOCD output
- **No debugging symbols**: Ensure build includes `-g3` flag
- **GDB crashes**: Try `gdb-multiarch` instead of regular `gdb`

## Safety Notes

- **Always remove propellers** when testing motor control code
- **Verify ESC calibration** before connecting motors
- **Use appropriate power supplies** to avoid damaging ESCs or motors
- **Monitor temperatures** during operation
- **Test incrementally** - start with low throttle values

## Expanding the Project

To add more source files:
1. Add `.c` files to `src/` directory
2. Add corresponding `.h` files to `inc/` directory  
3. Update `SOURCES` list in `CMakeLists.txt`:
   ```cmake
   set(SOURCES
       src/main.c
       src/motor_control.c
       src/dshot.c
       src/new_file.c
       src/startup_stm32f411xe.s
   )
   ```

## License

This project is provided as-is for educational and development purposes.
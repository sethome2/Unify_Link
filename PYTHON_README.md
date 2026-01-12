# Unify Link - Unified Communication Protocol Library

Python bindings for the Unify Link embedded communication protocol library.

## Features

- **Header-Only C++ Library**: Fast, zero-overhead communication protocol
- **Python Bindings**: Full Python 3.6+ support via pybind11
- **Multiple Components**:
  - Motor Control (with various control modes)
  - Encoder Feedback
  - Firmware Update
- **Protocol Features**:
  - CRC16 error checking
  - Sequence number tracking
  - Multi-component architecture
  - Ring buffer for efficient I/O

## Installation

### From PyPI (once published)
```bash
pip install unify-link
```

### From Source
```bash
git clone https://github.com/yourusername/unify-link.git
cd unify-link
pip install .
```

## Quick Start

```python
import unify_link as ul

# Create a communication instance
link = ul.UnifyLinkBase()

# Create motor and encoder components
motor = ul.MotorLink(link)
encoder = ul.EncoderLink(link)

# Send motor basic data
motor.send_motor_basic_data()

# Get transmitted bytes
tx_data = link.pop_send_buffer()

# Receive and parse data (simulating loop-back)
link.rev_data_push(tx_data)
link.parse_data_task()

print(f"Success frames: {link.success_count}")
```

## API Documentation

### Core Classes

#### `UnifyLinkBase`
Main communication handler managing send/receive buffers.

**Methods:**
- `parse_data_task()` - Parse received buffer and dispatch frames
- `rev_data_push(data: bytes) -> bool` - Push raw bytes into receive buffer
- `build_send_data(component_id: int, data_id: int, payload: bytes) -> int` - Build packet
- `pop_send_buffer() -> bytes` - Pop all buffered outbound data

**Properties:**
- `send_buff_used` - Used bytes in send buffer
- `send_buff_remain` - Available bytes in send buffer
- `last_seq_id` - Last received sequence number
- `com_error_count` - Communication error count
- `decode_error_count` - Decode error count
- `success_count` - Successful frames received

#### `MotorLink`
Motor control component.

**Data Structures:**
- `MotorBasic` - Position, speed, current, temperature, error code
- `MotorInfo` - Motor specifications and calibration
- `MotorSettings` - Feedback interval, mode, etc.
- `MotorSet` - Motor control commands (array of 8 values)

**Enumerations:**
- `MotorMode` - CURRENT_CONTROL, SPEED_CONTROL, POSITION_CONTROL, MIT_CONTROL
- `MotorErrorCode` - OK, OVER_HEAT_ERR, INTERNAL_ERR

**Methods:**
- `send_motor_basic_data()` - Transmit motor state
- `send_motor_info_data()` - Transmit motor info
- `send_motor_setting_data()` - Transmit settings
- `send_motor_set_current_data()` - Transmit control commands

#### `EncoderLink`
Encoder feedback component.

**Data Structures:**
- `EncoderBasic` - Position, velocity, error code (array of 8)
- `EncoderInfo` - Encoder specifications
- `EncoderSetting` - Feedback interval, etc.

**Enumerations:**
- `EncoderErrorCode` - OK, OVERFLOW_ERR, MAGNET_TOO_STRONG, MAGNET_TOO_WEAK, INTERNAL_ERR

**Methods:**
- `send_encoder_basic_data()` - Transmit encoder state
- `send_encoder_info_data()` - Transmit encoder info
- `send_encoder_setting_data()` - Transmit settings

#### `UpdateLink`
Firmware update component.

**Methods:**
- `send_firmware_info()` - Transmit firmware data
- `send_firmware_crc()` - Transmit CRC checksum

### Constants

- `COMPONENT_ID_SYSTEM` - System component ID
- `COMPONENT_ID_MOTORS` - Motor component ID
- `COMPONENT_ID_ENCODERS` - Encoder component ID
- `COMPONENT_ID_UPDATE` - Update component ID
- `FRAME_HEADER` - Frame header magic byte (0xA0)
- `MAX_FRAME_DATA_LENGTH` - Maximum payload length (512 bytes)

## Building from Source

Requirements:
- Python 3.6+
- CMake 3.16+
- A C++20 compatible compiler (MSVC, GCC, Clang)

### Development Build
```bash
# Clone repository
git clone https://github.com/yourusername/unify-link.git
cd unify-link

# Build and install in editable mode
pip install -e .

# Or with build isolation
pip install --no-build-isolation -e .
```

### Building the Wheel
```bash
# Install build tools
pip install build

# Build wheel
python -m build
```

## Testing

```bash
# Run C++ tests (requires CMake build)
cd build
ctest -C Debug

# Run Python examples
python -m unify_link.example
```

## License

MIT License - see LICENSE file for details

## Contributing

Contributions welcome! Please fork and submit pull requests.

## Support

For issues, feature requests, and discussions, please visit:
https://github.com/yourusername/unify-link/issues

## Acknowledgments

Built with:
- [pybind11](https://github.com/pybind/pybind11) - C++ to Python bindings
- [scikit-build-core](https://scikit-build-core.readthedocs.io/) - Modern Python build system

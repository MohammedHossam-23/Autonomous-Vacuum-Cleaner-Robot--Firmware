# Test strategy and validity

Passing tests only show that the code agrees with the assertions. This page
records *why those assertions are trustworthy*: where the expected values come
from, what the tests refuse to accept, and how the suite behaves when the code is
deliberately broken.

## Oracles, not mirrors

Round-trip tests (`encode` then `decode`) are necessary but weak: an encoder and
decoder that share the same misunderstanding still round-trip perfectly. Each
suite therefore also checks its output against a source outside the code under
test.

| Suite | Independent oracle | Negative cases |
| --- | --- | --- |
| `test_cobs` | Byte-exact vectors from the COBS specification (Cheshire & Baker) and its worked examples: `00` → `01 01`, `11 22 00 33` → `03 11 22 02 33`, and an empty input → `01`. Decoding is run against those literal vectors, not our own output. | Buffer overflow, zero byte inside an encoded body, truncated length code |
| `test_frame` | CRC-32/MPEG-2 catalogue check value `0x0376E6E7` for `"123456789"`. STM32 word order is pinned by constructing the equivalent byte stream by hand. A full frame is compared byte-for-byte with the frame produced by the Python bridge's algorithm (`crccheck` + explicit 32-bit word reversal). | Single-bit corruption at every frame position, truncation, missing delimiter, undersized output buffers, mismatched CRC |
| `test_pid` | Closed-form controller equations and a first-order plant simulated over 5,000 steps; convergence is checked against the requested setpoint. | Output saturation, integrator reverse-windup, derivative kick on a measurement step |
| `test_odometry` | Physical derivation from `WHEEL_radius_M` and `WHEELBASE`: a quarter wheel revolution must move `2·π·r/4`, and equal opposite wheel motion must rotate by `2·d/WHEELBASE`. | Heading stays within `[-π, π]`; RPM sign and EMA smoothing |
| `test_proto` | `TxMsg`/`RxMsg` are encoded by nanopb (the same library the device links against), pushed through the real frame layer, and decoded back field by field. | A flipped frame bit must fail before protobuf decoding |

The COBS and frame tests deliberately do **not** assert a CRC value produced by
the host implementation; hard-coded values came from the standard catalogue and
the ROS 2 bridge.

## Mutation check

`host/tests/mutation_check.py` injects one realistic defect at a time into the
production sources, rebuilds only the affected suite, and requires it to fail.
Sources are restored in a `finally` block, and a final clean `ctest` run verifies
the tree is back to normal.

```bash
python3 host/tests/mutation_check.py          # summary
python3 host/tests/mutation_check.py --verbose
```

Current result: **11/11 mutants killed**.

| Mutation | Killed by |
| --- | --- |
| COBS decoder consumes one byte too few | `test_cobs` |
| COBS encoder writes a wrong final length code | `test_cobs` |
| CRC polynomial changed from MPEG-2 | `test_frame` |
| CRC no longer reverses bytes within each 32-bit word | `test_frame` |
| Frame CRC written big-endian instead of little-endian | `test_frame` (known wire vector) |
| Frame decoder accepts a mismatched CRC | `test_frame` |
| PID integrator clamp disabled | `test_pid` |
| PID feed-forward term dropped | `test_pid` |
| Odometry RPM EMA smoothing removed | `test_odometry` |
| Odometry heading delta sign flipped | `test_odometry` |
| Odometry wheel distance scaled incorrectly | `test_odometry` |

One entry is worth highlighting: writing the CRC in the wrong byte order still
produces frames that decode correctly *with our own decoder*, so every
round-trip test still passes. Only the byte-exact vector from the bridge catches
it. That is the failure mode mutation testing exists to expose.

A surviving mutant means the suite does not constrain that behavior and should be
treated as a test gap, not ignored.

## Reproducing

```bash
cmake --preset host
cmake --build --preset host
ctest --preset host --output-on-failure
python3 host/tests/mutation_check.py
```

CI (`.github/workflows/host-tests.yml`) runs the same build, tests and mutation
check on every push and pull request.

## What is not covered

Be explicit about the boundary when reviewing:

- The firmware-only modules (HAL init, UART/ISR frame reassembly, FreeRTOS tasks,
  IMU/ToF drivers) are cross-compiled but not executed on the host.
- Timing assumptions such as the 1 MHz `TIM2` tick and the control-loop period
  are not verified.
- Odometry tests exercise the kinematics, not wheel calibration or encoder
  polarity on the real robot.
- `pid-step` is a simulation aid, not a hardware validation.

Hardware-in-the-loop tests remain the only way to close those gaps.

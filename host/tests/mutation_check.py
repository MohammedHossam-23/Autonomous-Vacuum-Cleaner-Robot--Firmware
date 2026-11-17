#!/usr/bin/env python3
"""Mutation check for the host test suite.

Each entry deliberately breaks one behavior in the production sources, rebuilds
only the affected test binary and asserts that the test fails. A mutant that
survives means the tests do not actually constrain that behavior.

The sources are restored in a finally block, and a clean build/test run is
performed at the end. Exit status is non-zero if any mutant survives.

Usage: python3 host/tests/mutation_check.py [--verbose]
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "host"
TEST_DIR = BUILD / "host" / "tests"
VERBOSE = "--verbose" in sys.argv[1:]

# (description, file, old snippet, replacement, failing test, occurrence)
# `occurrence` is zero-based and disambiguates snippets that appear more than
# once in a file (e.g. the two PID update functions).
MUTATIONS = [
    (
        "COBS decode consumes one byte too few",
        "avcrLowLevelCotrol/Core/Src/cobs.c",
        "            len_code--;",
        "            len_code -= 2;",
        "test_cobs",
        0,
    ),
    (
        "COBS encoder writes a wrong final length code",
        "avcrLowLevelCotrol/Core/Src/cobs.c",
        "        *dst_code_write_ptr = search_len;\n    }",
        "        *dst_code_write_ptr = (uint8_t)(search_len + 1u);\n    }",
        "test_cobs",
        0,
    ),
    (
        "CRC polynomial changed from MPEG-2",
        "host/src/crc32_mpeg2.c",
        "#define AVCR_CRC32_POLY 0x04C11DB7u",
        "#define AVCR_CRC32_POLY 0x04C11DB6u",
        "test_frame",
        0,
    ),
    (
        "CRC no longer reverses bytes within each 32-bit word",
        "host/src/crc32_mpeg2.c",
        "        crc = crc_byte(crc, data[i + 3]);",
        "        crc = crc_byte(crc, data[i + 0]);",
        "test_frame",
        0,
    ),
    (
        "frame CRC written big-endian instead of little-endian",
        "host/src/frame.c",
        "    scratch[len + 0] = (uint8_t)(crc & 0xFFu);",
        "    scratch[len + 0] = (uint8_t)((crc >> 24) & 0xFFu);",
        "test_frame",
        0,
    ),
    (
        "frame decoder accepts a mismatched CRC",
        "host/src/frame.c",
        "    if (received != calculated)",
        "    if (0 && received != calculated)",
        "test_frame",
        0,
    ),
    (
        "PID integrator clamp disabled",
        "avcrLowLevelCotrol/Core/Src/PID.c",
        "    if (pid->integrator > pid->limMaxInt) {",
        "    if (0 && pid->integrator > pid->limMaxInt) {",
        "test_pid",
        0,
    ),
    (
        "PID feed-forward term dropped",
        "avcrLowLevelCotrol/Core/Src/PID.c",
        "    float feedforward = pid->kff * setpoint;",
        "    float feedforward = 0.0f;",
        "test_pid",
        0,
    ),
    (
        "odometry RPM EMA smoothing removed",
        "avcrLowLevelCotrol/Core/Src/timerEncoderR.c",
        "enc->measured_RPM = (RPM_EMA_ALPHA * raw_RPM) + "
        "((1.0f - RPM_EMA_ALPHA) * enc->measured_RPM);",
        "enc->measured_RPM = raw_RPM;",
        "test_odometry",
        0,
    ),
    (
        "odometry heading delta sign flipped",
        "avcrLowLevelCotrol/Core/Src/timerEncoderR.c",
        "odom->delta_theta = (dr-dl)/WHEELBASE;",
        "odom->delta_theta = (dl-dr)/WHEELBASE;",
        "test_odometry",
        0,
    ),
    (
        "odometry wheel distance scaled incorrectly",
        "avcrLowLevelCotrol/Core/Src/timerEncoderR.c",
        "enc->wheelDis = ((float)enc->last_delta_ticks/(float)TICKS_PER_REV)* circumference;",
        "enc->wheelDis = ((float)enc->last_delta_ticks/(float)(TICKS_PER_REV * 2))* circumference;",
        "test_odometry",
        0,
    ),
]


def run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)


def build(target: str) -> subprocess.CompletedProcess[str]:
    return run(["cmake", "--build", "--preset", "host", "--target", target])


def main() -> int:
    print("Configuring and building the host target...")
    configure = run(["cmake", "--preset", "host"])
    if configure.returncode != 0:
        print(configure.stderr)
        return 2
    baseline = build("all")
    if baseline.returncode != 0:
        print(baseline.stdout)
        print(baseline.stderr)
        return 2

    baseline_tests = run(["ctest", "--preset", "host"])
    if baseline_tests.returncode != 0:
        print("Baseline tests already fail; fix them before running mutations.")
        return 2

    killed = 0
    survived: list[str] = []
    errored: list[str] = []

    print(f"Running {len(MUTATIONS)} mutations...\n")
    for description, rel_path, old, new, test_name, occurrence in MUTATIONS:
        path = ROOT / rel_path
        # Work on bytes: the firmware sources use CRLF, and a text-mode
        # rewrite would silently normalise every line ending in the file.
        original = path.read_bytes()
        old_bytes = old.encode()
        new_bytes = new.encode()
        if b"\r\n" in original:
            old_bytes = old_bytes.replace(b"\n", b"\r\n")
            new_bytes = new_bytes.replace(b"\n", b"\r\n")

        # Verify the snippet is unique enough to mutate deliberately.
        positions = []
        start = 0
        while True:
            idx = original.find(old_bytes, start)
            if idx == -1:
                break
            positions.append(idx)
            start = idx + 1
        if len(positions) <= occurrence:
            print(f"ERROR  {description}: snippet not found in {rel_path}")
            errored.append(description)
            continue

        idx = positions[occurrence]
        mutated = original[:idx] + new_bytes + original[idx + len(old_bytes):]
        try:
            path.write_bytes(mutated)
            built = build(test_name)
            if built.returncode != 0:
                print(f"ERROR  {description}: mutant did not compile")
                if VERBOSE:
                    print(built.stdout)
                    print(built.stderr)
                errored.append(description)
                continue

            result = run([str(TEST_DIR / test_name)])
            if result.returncode != 0:
                killed += 1
                print(f"KILLED {description}  ({test_name})")
            else:
                survived.append(description)
                print(f"SURVIVED {description}  ({test_name})")
                if VERBOSE:
                    print(result.stdout)
        finally:
            path.write_bytes(original)

    # Leave the tree and build directory in a known-good state.
    build("all")
    final_tests = run(["ctest", "--preset", "host"])

    print("\nSummary")
    print(f"  killed:   {killed}/{len(MUTATIONS)}")
    print(f"  survived: {len(survived)}")
    print(f"  errored:  {len(errored)}")

    if survived:
        print("\nSurviving mutants (tests do not constrain these):")
        for description in survived:
            print(f"  - {description}")
    if errored:
        print("\nMutations that could not be applied/compiled:")
        for description in errored:
            print(f"  - {description}")

    if final_tests.returncode != 0:
        print("\nERROR: restored tree does not pass its own tests.")
        return 2

    return 0 if not survived and not errored else 1


if __name__ == "__main__":
    sys.exit(main())

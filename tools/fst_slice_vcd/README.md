# FST Slice VCD

Tiny utility to extract a time window from an `.fst` trace and write it as a
small `.vcd` containing all signals in that window.
To be used with large Verilator/GTKWave traces where a full
`fst2vcd` conversion would be too slow or too large to process (e.g. in
Gtkwave).

## What It Does

- Opens an `.fst` trace with the native FST reader API.
- Limits iteration to a requested `[start, end]` time range.
- Emits a plain VCD with:
  - the full hierarchy
  - all variables/signals
  - only the value changes needed for the selected time window

## Dependencies

Required:

- CMake 3.16+
- A C compiler
- `zlib`
- A Verilator installation that ships the GTKWave FST API sources:
  - `fstapi.h`
  - `fstapi.c`
  - `fastlz.c`
  - `lz4.c`

It finds the FST API in this order:

1. `-DFSTAPI_DIR=/path/to/include/gtkwave`
2. `FSTAPI_DIR` environment variable
3. `VERILATOR_ROOT/include/gtkwave`
4. `verilator -V` and the reported `VERILATOR_ROOT`

## Compatibility Note

This should work with Verilator installations that include the GTKWave FST API
sources under `include/gtkwave`.

So:

- It should work with a typical distro Verilator install if that layout exists.
- It should work with a local Verilator source/build tree if `VERILATOR_ROOT`
  is set or `FSTAPI_DIR` is provided.
- It will **not** work with an arbitrary Verilator package that omits those FST
  API source files.

## Build

Auto-detect from the active `verilator` in `PATH`:

```bash
cmake -S . -B build
cmake --build build -j
```

Or specify the FST API directory explicitly:

```bash
cmake -S . -B build -DFSTAPI_DIR=/path/to/verilator/include/gtkwave
cmake --build build -j
```

## Usage

```bash
./build/fst_slice_vcd \
  --input /path/to/trace.fst \
  --output /path/to/window.vcd \
  --start 2705319200000 \
  --end 2705319400000
```

Optional:

```bash
--vcd-extensions
```

## Notes

- The output is a VCD slice, not an FST slice.
- The tool preserves all signals, which is useful for GTKWave inspection.
- The time range is enforced by the native FST reader, so it avoids a
  full-file text conversion path.

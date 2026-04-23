# Bridge Trace Analyzer

Streaming analyzer for AHB-to-AXI bridge traces.

This tool is to proccess large Verilator captures where the relevant
signals for the bridge are traced from the top-level testbench.

## Goals

- Accept `.vcd` directly.
- Accept `.fst` by streaming it through `fst2vcd`.
- Discover bridge probes by signal name rather than hardcoded VCD ids.
- Reconstruct:
  - accepted AHB-side transfers
  - AXI `AW/W/B`
  - AXI `AR/R`
- Check for:
  - write-side mismatches between AHB intent and AXI traffic
  - read-side sequencing mismatches
  - optional shadow-memory inconsistencies

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Usage

```bash
./build/bridge_trace_analyzer --input /path/to/dump.fst
./build/bridge_trace_analyzer --input /path/to/dump.vcd
```

Useful options:

```bash
--input <file>
--max-mismatches <n>
--no-shadow-memory
--verbose
--progress-interval
--show-notes
```

## Notes

- `.fst` input currently depends on `fst2vcd` being available in `PATH`.
- The analyzer is designed to work in a streaming way; it does not need
  a converted VCD on disk.
- Signal discovery expects the bridge-facing top-level signals to be present in
  the trace. Extra probes are not a problem.
- Legal but ambiguous reconstruction cases, such as early-terminated fixed AHB
  bursts or bounded read/write stream resynchronization, are counted as notes.
  They are suppressed by default and printed with `--show-notes`.

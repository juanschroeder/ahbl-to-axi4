# Bridge Trace Analyzer

Streaming analyzer for AHB-to-AXI bridge traces.

This tool is to proccess large Verilator captures where the relevant
signals for the bridge are traced from the top-level testbench.

## Goals

- Accept `.vcd` directly.
- Accept `.fst` by streaming it through `fst2vcd`.
- Accept Vivado `.ila` captures by streaming the embedded per-sample
  `waveform.csv` member with `unzip`.
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
./build/bridge_trace_analyzer --input /path/to/capture.ila
```

Useful options:

```bash
--input <file>
--format <auto|fst|vcd|ila>
--sample-mode <auto|clock|timestamp>
--max-mismatches <n>
--no-shadow-memory
--verbose
--progress-interval
--show-notes
```

## Notes

- `.fst` input currently depends on `fst2vcd` being available in `PATH`.
- `.ila` input currently depends on `unzip` being available in `PATH` and on
  the archive containing `waveform.csv`.
- The analyzer is designed to work in a streaming way; it does not need
  a converted VCD on disk.
- Signal discovery expects the bridge-facing AHB and AXI signals to be present
  in the same trace. Extra probes are not a problem.
- In `--sample-mode auto`, VCD/FST input uses `clk` posedges when a clock probe
  is present. If no clock probe exists, it treats each VCD timestamp as one
  sample. Vivado `.ila` input does not rely on sparse VCD timestamps; it reads
  the exported per-sample table from the archive.
- Legal but ambiguous reconstruction cases, such as early-terminated fixed AHB
  bursts or bounded read/write stream resynchronization, are counted as notes.
  They are suppressed by default and printed with `--show-notes`.

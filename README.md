# ahb-lite-to-axi4

AHB-lite to AXI4 bridge.

## Status: 
- RTL and TB is WIP. RTL is used (and working) with Core-V Wally (ttps://github.com/openhwgroup/cvw) replacing proprietary alternative.
- Cocotb TB is far from being complete

# Run tests

```bash
pip install -r requirements.txt

make
```

Or point to an RTL elsewhere:

```bash
make DUT_SOURCES=/absolute/path/to/ahb_to_axi4_burst.sv
```

## Remarks

By default the testbench needs Verilator.



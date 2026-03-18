SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= $(PWD)/rtl
TB_DIR  ?= $(PWD)/tb

#DUT_ROOT = /yocto/fpga/cvw_genesys2/addins/cvwsoc/
DUT_ROOT = $(RTL_DIR)
DUT_SOURCES ?= $(RTL_DIR)/ahb_to_axi4_burst.sv
WRAPPER := ahb_to_axi4_burst_cocotb_wrapper_v3.sv

VERILOG_SOURCES := \
	$(TB_DIR)/$(WRAPPER) \
	$(DUT_SOURCES)


MODULE   := test_ahb_to_axi4_burst

export PYTHONPATH := $(TB_DIR):$(PYTHONPATH)

EXTRA_ARGS += --timing --Wno-fatal
# generated .vcd. # newer way: EXTRA_ARGS += --trace-vcd
EXTRA_ARGS += --trace

COMPILE_ARGS += -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTH

COCOTB_HDL_TIMEUNIT      := 1ns
COCOTB_HDL_TIMEPRECISION := 1ps

include $(shell cocotb-config --makefiles)/Makefile.sim

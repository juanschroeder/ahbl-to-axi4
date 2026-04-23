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


# ---- Formal verification targets ----
FORMAL_DIR ?= formal
FORMAL_LOG_DIR ?= $(FORMAL_DIR)/logs
FORMAL_JOBS ?= 4
SBY ?= sby

FORMAL_SBY := $(sort $(wildcard $(FORMAL_DIR)/*.sby))

.PHONY: formal-list formalverif formalverif-par formal-clean

formal-list:
	@printf '%s\n' $(FORMAL_SBY)

formalverif:
	@set -e; \
	mkdir -p $(FORMAL_LOG_DIR); \
	for f in $(FORMAL_SBY); do \
		base=$$(basename $$f .sby); \
		echo "==> Running $$f"; \
		$(SBY) -f $$f 2>&1 | tee $(FORMAL_LOG_DIR)/$$base.log; \
	done

formalverif-par:
	@set -e; \
	mkdir -p $(FORMAL_LOG_DIR); \
	printf '%s\n' $(FORMAL_SBY) | xargs -n1 -P$(FORMAL_JOBS) -I{} sh -c '\
		f="{}"; \
		base=$$(basename "$$f" .sby); \
		echo "==> Running $$f"; \
		$(SBY) -f "$$f" > "$(FORMAL_LOG_DIR)/$$base.log" 2>&1'

formal-clean:
	rm -rf $(FORMAL_LOG_DIR) \
	       $(FORMAL_DIR)/*/ \
	       $(FORMAL_DIR)/*_bmc/ \
	       $(FORMAL_DIR)/*_cover/ \
	       $(FORMAL_DIR)/*_prove/ \
	       $(FORMAL_DIR)/*_live/ \
	       $(FORMAL_DIR)/*_abc/ \
	       $(FORMAL_DIR)/*_z3/

ifneq ($(MKENV_INCLUDED),1)
export SDK_SRC_ROOT_DIR := $(realpath $(dir $(realpath $(lastword $(MAKEFILE_LIST))))/../../../../)
endif

include $(SDK_SRC_ROOT_DIR)/tools/mkenv.mk
include $(SDK_TOOLS_DIR)/toolchain_rtsmart.mk

CC 	= $(CROSS_COMPILE)gcc
CPP = $(CROSS_COMPILE)g++

CC_CFLAGS=-mcmodel=medany -march=rv64imafdcv -mabi=lp64d -Werror -Wall -O0 -n --static $(KCFLAGS)

LINKFLAG=-T $(MPP_SRC_DIR)/userapps/sample/linker_scripts/riscv64/link.lds -lpthread

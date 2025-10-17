ifneq ($(MKENV_INCLUDED),1)
export SDK_SRC_ROOT_DIR := $(realpath $(dir $(realpath $(lastword $(MAKEFILE_LIST))))/../../../../../)
endif

include $(SDK_SRC_ROOT_DIR)/tools/mkenv.mk
include $(SDK_TOOLS_DIR)/toolchain_rtsmart.mk

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

CC_CFLAGS = -mcmodel=medany -march=rv64imafdcv -mabi=lp64d -O0 -Werror $(KCFLAGS)
ARFLAGS = -rc

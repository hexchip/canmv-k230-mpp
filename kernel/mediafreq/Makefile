
include $(MPP_SRC_DIR)/kernel/mpp.mk
include $(MPP_SRC_DIR)/kernel/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
LIB = $(MPP_SRC_DIR)/kernel/lib/lib$(CURRECT_DIR_NAME).a
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/kernel/$(CURRECT_DIR_NAME)

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)/src/ -I$(LOCAL_SRC_DIR)/src/sysctl/sysctl_media_clock

SRC_DIRS := src
SRC_DIRS += src/sysctl/sysctl_media_clock

SRCS = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))
COBJS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o))
CDEPS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o.d))

CPP_SRCS = $(wildcard *.cpp)
CPP_OBJS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o))
CPP_DEPS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o.d))

OBJS := $(COBJS) $(CPP_OBJS)
DEPS := $(CDEPS) $(CPP_DEPS)

.PHONY: all
all: $(LIB)
	@echo "Make lib$(CURRECT_DIR_NAME) done."

.PHONY: clean
clean:
	@rm -rf $(LIB) $(OBJS) $(DEPS)

$(LIB): $(OBJS)
	@echo "AR $@"
	@$(AR) $(ARFLAGS) $@ $(OBJS)

$(OBJS): $(BUILD)/%.o : %.c
	@echo [CC] $<
	@$(CC) $(CC_CFLAGS) -Wno-error $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_DRIVER_CFLGAS) -MD -MP -MF $@.d -c $< -o $@

# $(sort $(var)) removes duplicates
#
# The net effect of this, is it causes the objects to depend on the
# object directories (but only for existence), and the object directories
# will be created if they don't exist.
OBJ_DIRS = $(sort $(dir $(OBJS)))
$(OBJS): | $(OBJ_DIRS)
$(OBJ_DIRS):
	@mkdir -p $@

-include $(DEPS)

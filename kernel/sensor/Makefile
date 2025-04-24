
include $(MPP_SRC_DIR)/kernel/mpp.mk
include $(MPP_SRC_DIR)/kernel/rt-smart.mk

include $(SDK_SRC_ROOT_DIR)/.config

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
LIB = $(MPP_SRC_DIR)/kernel/lib/lib$(CURRECT_DIR_NAME).a
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/kernel/$(CURRECT_DIR_NAME)

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)/src/ \
			   -I$(RTSMART_SRC_DIR)/kernel/bsp/maix3/board/interdrv/gpio

src-y := src/sensor_comm.c src/sensor_dev.c
src-$(CONFIG_MPP_ENABLE_SENSOR_GC2093) += src/gc2093/gc2093.c
src-$(CONFIG_MPP_ENABLE_SENSOR_OV5647) += src/ov5647/ov5647.c
src-$(CONFIG_MPP_ENABLE_SENSOR_IMX335) += src/imx335/imx335.c
src-$(CONFIG_MPP_ENABLE_SENSOR_SC132GS) += src/sc132gs/sc132gs.c
src-$(CONFIG_MPP_ENABLE_SENSOR_XS9950) += src/xs9950/xs9950.c
src-$(CONFIG_MPP_ENABLE_SENSOR_BF3238) += src/bf3238/bf3238.c

COBJS 	:= $(patsubst %, $(BUILD)/%, $(src-y:.c=.o))
CDEPS 	:= $(patsubst %, $(BUILD)/%, $(src-y:.c=.o.d))

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

include $(MPP_SRC_DIR)/userapps/src/mpp.mk
include $(MPP_SRC_DIR)/userapps/src/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
LIB = $(MPP_SRC_DIR)/userapps/lib/lib$(CURRECT_DIR_NAME).a
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/userapps/$(CURRECT_DIR_NAME)

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)/

SRCS = $(wildcard *.c)
COBJS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o))
CDEPS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o.d))

CPP_SRCS = $(wildcard *.cpp)
CPP_OBJS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o))
CPP_DEPS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o.d))

OBJS := $(COBJS) $(CPP_OBJS)
DEPS := $(CDEPS) $(CPP_DEPS)

YMLS = $(wildcard $(LOCAL_SRC_DIR)/dewarp/*.yml)
DWCFGS = $(patsubst %.yml,%.bin,$(YMLS))

.PHONY: k230dwmapgen
k230dwmapgen:
	@cmake -S $(LOCAL_SRC_DIR)/dewarp/k230dwmapgen -B $(BUILD)/dewrap
	@cmake --build $(BUILD)/dewrap -j

$(DWCFGS): %.bin : %.yml k230dwmapgen
	@$(BUILD)/dewrap/k230dwmapgen-dump < $< > $@

.PHONY: all
all: $(LIB) $(DWCFGS)
	@mkdir -p $(RTSMART_SRC_DIR)/userapps/root/bin/
	@cp $(LOCAL_SRC_DIR)/config/* $(RTSMART_SRC_DIR)/userapps/root/bin/
	@cp $(LOCAL_SRC_DIR)/dewarp/*.bin $(RTSMART_SRC_DIR)/userapps/root/bin/
	@echo "Make lib$(CURRECT_DIR_NAME) done."

.PHONY: clean
clean:
	@rm -rf $(LIB) $(OBJS) $(DEPS)

$(LIB): $(OBJS)
	@echo "AR $@"
	@$(AR) $(ARFLAGS) $@ $^

$(OBJS): $(BUILD)/%.o : %.c
	@echo [CC] $<
	@$(CC) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -MD -MP -MF $@.d -c $< -o $@

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

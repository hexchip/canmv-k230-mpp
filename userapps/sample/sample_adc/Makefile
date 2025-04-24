include ../mkenv.mk

include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_USERAPP_SMAPLE_ELF_DIR)/$(CURRECT_DIR_NAME).elf
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/userapps/sample/$(CURRECT_DIR_NAME)
LIBPATH = $(MPP_LIB_PATH)
LIBPATH += -L$(RTSMART_SRC_DIR)/userapps/sdk/rt-thread/lib/risc-v/rv64
LIBS = $(MPP_LIBS)
LIBS += -lrtthread

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)

DEVICE_CFLAGS = -I$(RTSMART_SRC_DIR)/userapps \
		-I$(RTSMART_SRC_DIR)/userapps/sdk/rt-thread/include \
		-I$(RTSMART_SRC_DIR)/userapps/sdk/rt-thread/components/drivers

SRCS = $(wildcard *.c)
COBJS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o))
CDEPS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o.d))

CPP_SRCS = $(wildcard *.cpp)
CPP_OBJS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o))
CPP_DEPS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o.d))

OBJS := $(COBJS) $(CPP_OBJS)
DEPS := $(CDEPS) $(CPP_DEPS)

all: $(BIN)
	@echo "Make sample $(CURRECT_DIR_NAME) done."

$(COBJS): $(BUILD)/%.o : %.c
	@echo [CC] $@
	@$(CC) -DHAVE_CCONFIG_H $(CC_CFLAGS) $(LOCAL_CFLAGS) $(DEVICE_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -MD -MP -MF $@.d -c $< -o $@

$(BIN): $(OBJS)
	@echo [LD] $@
	@$(CC) -o $(BIN) $(LINKFLAG) -Wl,--whole-archive -Wl,--no-whole-archive -n --static $(OBJS) -L$(LIBPATH) -Wl,--start-group $(LIBS) -Wl,--end-group

clean:
	@rm -rf $(BIN) $(OBJS) $(DEPS)

.PHONY: all clean

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

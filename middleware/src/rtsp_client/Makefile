include ../../mkenv.mk

include $(MPP_SRC_DIR)/middleware/mpp.mk
include $(MPP_SRC_DIR)/middleware/rt-smart.mk

AR = riscv64-unknown-linux-musl-ar

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
LIB = $(MPP_MIDDLEWARE_LIB_INSTALL_PATH)/lib$(CURRECT_DIR_NAME).a
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/middleware/$(CURRECT_DIR_NAME)
LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)

LIBS_FOR_CONSOLE_APPLICATION = -lssl -lcrypto
CC_CFLAGS_EX=-mcmodel=medany -march=rv64imafdcv -mabi=lp64d -Wall -O0 -g -gdwarf-2 -n --static $(KCFLAGS)

LIVE555_USAGEENVIRONMENT= ../live555/UsageEnvironment
LIVE555_BASICUSAGEENVIRONMENT= ../live555/BasicUsageEnvironment
LIVE555_GROUPSOCK = ../live555/groupsock
LIVE555_LIVEMEDIA = ../live555/liveMedia

LOCAL_LIBS_PATH = -L$(LIVE555_USAGEENVIRONMENT) -L$(LIVE555_BASICUSAGEENVIRONMENT) -L$(LIVE555_GROUPSOCK) -L$(LIVE555_LIVEMEDIA)
LOCAL_LIBS = -lliveMedia -lgroupsock -lBasicUsageEnvironment -lUsageEnvironment

LIVE555_COMPILE_OPTS = -DSOCKLEN_T=socklen_t -DNO_SSTREAM=1 -D_LARGEFILE_SOURCE=1 -D_FILE_OFFSET_BITS=64 -DNO_OPENSSL=1

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR) -I$(LOCAL_SRC_DIR)/include -I$(LIVE555_GROUPSOCK)/include -I$(LIVE555_USAGEENVIRONMENT)/include -I$(LIVE555_BASICUSAGEENVIRONMENT)/include -I$(LIVE555_LIVEMEDIA)/include

WERROR_FLAGS = -Wno-register -Wno-multichar -Wno-pessimizing-move -Wno-deprecated-declarations -Wno-unused-result -Wno-unused-variable -Wno-format -Wno-return-type -Wno-sign-compare -Wno-unused-label

CXX_FLAGS = -std=c++20 \
			$(WERROR_FLAGS) \
			$(LIVE555_COMPILE_OPTS) \

SRCS = $(wildcard *.c)
COBJS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o))
CDEPS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o.d))

CPP_SRCS = $(wildcard *.cpp)
CPP_OBJS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o))
CPP_DEPS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o.d))

OBJS := $(COBJS) $(CPP_OBJS)
DEPS := $(CDEPS) $(CPP_DEPS)

all:$(LIB)
	@echo "Make lib$(CURRECT_DIR_NAME) done."

$(COBJS): $(BUILD)/%.o : %.c
	@echo [CC] $@
	@$(CC) $(CC_CFLAGS_EX) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LOCAL_LIBS) -MD -MP -MF $@.d -c $< -o $@

$(CPP_OBJS): $(BUILD)/%.o : %.cpp
	@echo [CXX] $@
	@$(CPP) $(CC_CFLAGS_EX) $(CXX_FLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LOCAL_LIBS) -MD -MP -MF $@.d -c $< -o $@

$(LIB): $(OBJS)
	@echo [LD] $@
	@$(AR) $(ARFLAGS) $@ $^

clean:
	@rm -rf $(LIB) $(OBJS) $(DEPS)

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

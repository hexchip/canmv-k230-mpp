include ../../mkenv.mk

include $(MPP_SRC_DIR)/middleware/mpp.mk
include $(MPP_SRC_DIR)/middleware/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_MIDDLEWARE_ELF_INSTALL_PATH)/$(CURRECT_DIR_NAME).elf
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/middleware/sample/$(CURRECT_DIR_NAME)
LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)

LIBS_FOR_CONSOLE_APPLICATION = -lssl -lcrypto
CC_CFLAGS_EX=-mcmodel=medany -march=rv64imafdcv -mabi=lp64d -Wall -O0 -g -gdwarf-2 -n --static $(KCFLAGS)

FFMPEG_DIR = $(MPP_SRC_DIR)/middleware/src/ffmpeg/src/lib_ffmpeg/
X264_DIR = $(MPP_SRC_DIR)/middleware/src/x264/src/lib_x264/
KDMEDIA = $(MPP_SRC_DIR)/middleware/src/kdmedia
RTSP_PUSHER = $(MPP_SRC_DIR)/middleware/src/rtsp_pusher

LOCAL_LIBS_PATH = -L$(MPP_MIDDLEWARE_LIB_INSTALL_PATH) -L$(FFMPEG_DIR)/lib -L$(X264_DIR)/lib
LOCAL_LIBS = -lkdmedia -lrtsp_pusher -lavformat -lavcodec  -lavutil -lswresample -lpostproc -lx264 -lpthread

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR) -I$(FFMPEG_DIR)/include -I$(KDMEDIA)/include -I$(RTSP_PUSHER)/include

WERROR_FLAGS = -Wno-register -Wno-multichar -Wno-pessimizing-move -Wno-deprecated-declarations -Wno-unused-result -Wno-unused-variable -Wno-format -Wno-return-type -Wno-sign-compare -Wno-unused-label

CXX_FLAGS = $(WERROR_FLAGS)

SRCS = $(wildcard *.c)
COBJS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o))
CDEPS 	:= $(patsubst %, $(BUILD)/%, $(SRCS:.c=.o.d))

CPP_SRCS = $(wildcard *.cpp)
CPP_OBJS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o))
CPP_DEPS 	:= $(patsubst %, $(BUILD)/%, $(CPP_SRCS:.cpp=.o.d))

OBJS := $(COBJS) $(CPP_OBJS)
DEPS := $(CDEPS) $(CPP_DEPS)

all:$(BIN)
	@echo "Make sample $(CURRECT_DIR_NAME) done."

$(COBJS): $(BUILD)/%.o : %.c
	@echo [CC] $@
	@$(CC) $(CC_CFLAGS_EX) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LOCAL_LIBS) -MD -MP -MF $@.d -c $< -o $@

$(CPP_OBJS): $(BUILD)/%.o : %.cpp
	@echo [CXX] $@
	@$(CPP) $(CC_CFLAGS_EX) $(CXX_FLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LOCAL_LIBS) -MD -MP -MF $@.d -c $< -o $@

$(BIN): $(OBJS)
	@echo [LD] $@
	@$(CPP) -o $(BIN) $(LINKFLAG) -Wl,--whole-archive -Wl,--no-whole-archive -n --static $^ -L$(LIBPATH) $(CXX_FLAGS) $(LOCAL_LIBS) $(LOCAL_LIBS_PATH) -Wl,--start-group $(LIBS) -Wl,--end-group

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

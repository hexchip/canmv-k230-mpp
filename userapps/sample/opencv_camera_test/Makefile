include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_SRC_DIR)/userapps/sample/elf/$(CURRECT_DIR_NAME).elf
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/userapps/sample/$(CURRECT_DIR_NAME)

LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)

OPENCV_LIB_PATH=$(MPP_SRC_DIR)/userapps/src/opencv/build/install/lib
OPENCV_3RDPARTY_LIB_PATH=$(MPP_SRC_DIR)/userapps/src/opencv/build/install/lib/opencv4/3rdparty
OPENCV_LIBS = -latomic \
              -lopencv_core \
              -lzlib \
              -llibjpeg-turbo \
              -llibopenjp2 \
              -llibpng \
              -llibtiff \
              -llibwebp \
              -llibprotobuf \
              -lopencv_imgproc \
              -lopencv_imgcodecs \
              -lopencv_videoio \
              -lopencv_video \
              -lopencv_highgui \
              -lopencv_objdetect


LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR) \
	       -I$(MPP_SRC_DIR)/userapps/src/opencv/build/install/include/opencv4 

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
	@$(CC) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -MD -MP -MF $@.d -c $< -o $@

$(CPP_OBJS): $(BUILD)/%.o : %.cpp
	@echo [CXX] $@
	@$(CPP) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -MD -MP -MF $@.d -c $< -o $@

$(BIN): $(OBJS) $(CPP_OBJS)
	@echo [LD] $@
	@$(CPP) -o $(BIN) $(LINKFLAG) -Wl,--whole-archive -Wl,--no-whole-archive -n --static $^ -L$(LIBPATH) -L$(OPENCV_LIB_PATH) -L$(OPENCV_3RDPARTY_LIB_PATH) -Wl,--start-group $(LIBS) $(OPENCV_LIBS) -Wl,--end-group

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

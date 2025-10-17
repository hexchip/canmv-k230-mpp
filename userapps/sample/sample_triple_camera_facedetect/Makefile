include ../mkenv.mk

include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
BUILD := $(SDK_RTSMART_BUILD_DIR)/mpp/userapps/sample/$(CURRECT_DIR_NAME)

all:
	$(Q)mkdir -p $(BUILD)
	$(Q)cmake -S . -B $(BUILD)
	$(Q)cmake --build $(BUILD)
	$(Q)cmake --install $(BUILD)

clean:
	$(Q)rm -rf $(BUILD)

.PHONY: all clean

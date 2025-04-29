subdirs-y := sensor connector fft cipher pm vg_lite uvc

all:
	@$(foreach dir,$(subdirs-y),$(MAKE) -C $(dir) all || exit $?;)
	@echo "Make MPP Userapps done."

clean:
	@$(foreach dir,$(subdirs-y),$(MAKE) -C $(dir) clean || exit $?;)
	@echo "Make MPP Userapps clean done."

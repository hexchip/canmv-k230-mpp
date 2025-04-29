subdirs-y := sensor fft connector mediafreq pm

all:
	@$(foreach dir,$(subdirs-y),$(MAKE) -C $(dir) all || exit $?;)
	@echo "Make MPP Kernel done."

clean:
	@$(foreach dir,$(subdirs-y),$(MAKE) -C $(dir) clean || exit $?;)
	@echo "Make MPP Kernel clean done."

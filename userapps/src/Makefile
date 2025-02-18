subdirs-y := sensor connector fft cipher pm vg_lite

all:
	@$(foreach dir,$(subdirs-y),make -C $(dir) all || exit $?;)
	@echo "Make MPP Userapps done."

clean:
	@$(foreach dir,$(subdirs-y),make -C $(dir) clean || exit $?;)
	@echo "Make MPP Userapps clean done."

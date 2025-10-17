all:
	$(Q)$(MAKE) -C src all || exit $?;
	$(Q)$(MAKE) -C sample all || exit $?;

clean:
	$(Q)$(MAKE) -C src clean || exit $?;
	$(Q)$(MAKE) -C sample clean || exit $?;

.PHONY: all clean

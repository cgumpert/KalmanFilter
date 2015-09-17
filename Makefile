# compile doxygen documentation
.PHONY: doc
doc:
	@$(MAKE) -C doc/ doc

# cleann all sub-directories
.PHONY: clean
clean:
	@$(MAKE) -C doc/ clean

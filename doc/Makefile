LOGFILE=doxygen.log
.PHONY: doc
doc: doxygen.config
	@doxygen $< > $(LOGFILE)

.PHONY: clean
clean:
	@rm -rf $(LOGFILE)
	@rm -rf latex/
	@rm -rf html/
	@rm -f doxygen_sqlite3.db


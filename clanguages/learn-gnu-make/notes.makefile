# First makefile for learning gnu-make.
# Use `make -f notes.makefile` to use this file.

.PHONY : all
all:
	@echo Hello world!
	@echo =======
	@echo A rule has following shape:
	@echo # shape start
	@echo target ... : prerequisites ...
	@echo   recipe
	@echo   ...
	@echo # shape end
	@echo There must be a tab character at each recipe line beginning.
	@echo Set .RECIPEPREFIX variable could use another character as recipe prefix.
	@echo =======
	@echo While .PHONY appeared as rule target,
	@echo the rule prerequisites are considered as phony targets.
	@echo Phony target recipe will be executed unconditionally when considering.
	@echo Phony targets can have prerequisites.
	@echo =======
	@echo Valid sequence of rules:
	@echo The entry rule followed with dependent rules.
	@echo =======
	@echo make works in two distinct phases: a read-in and a target-update phase
	@echo =======
	@echo =======
	@echo =======
	@echo =======
	@echo =======

.PHONY : clean
clean:
	@echo Dummy clean command called.

.PHONY : one two
one:
	@echo Dummy one.
two:
	@echo Dummy two.

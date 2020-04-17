args_c_standard = -std=c99
args_optimizers = -O2
args_includes = -Iinclude
args_warnings = -Wall

args_set = $(args_c_standard) $(args_optimizers) \
 	$(args_includes) $(args_warnings)

path_src = src
path_lib = lib
path_bin = bin
main_name = main.c
app_name = test

sources := $(filter-out $(path_src)/$(main_name),$(wildcard $(path_src)/*.c))
objects := $(sources:.c=.o)
objects := $(subst $(path_src)/,$(path_lib)/,$(objects))

src_main = $(path_src)/$(main_name)
app_path = $(path_bin)/$(app_name)

combined_args = $(args_set) \
	$(objects) $(src_main) -o $(app_path)

all: $(objects)
	$(CC) $(combined_args)
	@echo '====>>> Make Done.'
.PHONY: all

$(objects): $(path_lib)/%.o: $(path_src)/%.c
	$(CC) $(args_set) -c $< -o $@

cpp:
	$(CXX) -std=c++11 sandbox.cpp -o test
.PHONY: cpp

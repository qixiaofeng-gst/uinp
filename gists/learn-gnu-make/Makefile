args_c_standard = -std=c99
args_optimizers = -O2
args_includes = -Iinclude
args_warnings = -Wall

args_set = $(args_c_standard) $(args_optimizers) \
 	$(args_includes) $(args_warnings)

msg_make_separator = @echo '====>>>> Make done.'
path_src = src
path_lib = lib
path_bin = bin
main_name = main.c
app_name = main

sources := $(filter-out $(path_src)/$(main_name),$(wildcard $(path_src)/*.c))
objects := $(sources:.c=.o)
objects := $(subst $(path_src)/,$(path_lib)/,$(objects))

src_main = $(path_src)/$(main_name)
app_path = $(path_bin)/$(app_name)

combined_args = $(args_set) \
	$(objects) $(src_main) -o $(app_path)

all: $(objects)
	$(CC) $(combined_args)
	$(msg_make_separator)
.PHONY: all

$(objects): $(path_lib)/%.o: $(path_src)/%.c
	$(CC) $(args_set) -c $< -o $@

path_test = test
test_entrance_path = $(path_test)/main.c
test_bin_path = $(path_bin)/test
combined_args_for_test = $(CC) $(args_set) -I$(path_src) $(objects) \
 	$(test_entrance_path) -o $(test_bin_path)
test: $(objects)
	$(combined_args_for_test)
	$(msg_make_separator)
.PHONY: test

cpp:
	$(CXX) -std=c++11 sandbox.cpp -o test
.PHONY: cpp

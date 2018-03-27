
NINJA_BIN := ninja
ifndef NO_NINJA_BUILD
        NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)

        ifndef NINJA_BUILD
                NINJA_BIN := ninja-build
                NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)
        endif
endif

ifdef NINJA_BUILD
        PX4_CMAKE_GENERATOR := Ninja
        PX4_MAKE := $(NINJA_BIN)
else
        ifdef SYSTEMROOT
                # Windows
                PX4_CMAKE_GENERATOR := "MSYS\ Makefiles"
        else
                PX4_CMAKE_GENERATOR := "Unix\ Makefiles"
        endif
        PX4_MAKE = $(MAKE)
        PX4_MAKE_ARGS = --no-print-directory
endif



.PHONY: all test coverage format clean

all: test format

test:
	mkdir -p build/test
	cd build/test && cmake -G"$(PX4_CMAKE_GENERATOR)" -DTESTING=ON ../.. && $(PX4_MAKE) check

test_asan:
	mkdir -p build/test_asan
	cd build/test_asan && cmake -G"$(PX4_CMAKE_GENERATOR)" -DTESTING=ON -DASAN=ON ../.. && $(PX4_MAKE) check

test_ubsan:
	mkdir -p build/test_ubsan
	cd build/test_ubsan && cmake -G"$(PX4_CMAKE_GENERATOR)" -DTESTING=ON -DUBSAN=ON ../.. && $(PX4_MAKE) check

coverage:
	mkdir -p build/coverage
	cd build/coverage && cmake -G"$(PX4_CMAKE_GENERATOR)" -DCOV_HTML=ON -DCMAKE_BUILD_TYPE=Coverage -DTESTING=ON -DSUPPORT_STDIOSTREAM=ON ../.. && $(PX4_MAKE) check && $(PX4_MAKE) coverage_build && $(PX4_MAKE) coverage_html

format:
	mkdir -p build/format
	cd build/format && cmake -G"$(PX4_CMAKE_GENERATOR)" -DFORMAT=ON ../.. && $(PX4_MAKE) check


clean:
	@rm -rf  build

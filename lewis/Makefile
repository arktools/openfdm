targets = test

all: $(targets)

test: math.f90 aircraft.f90 f16.f90 test.f90
	gfortran -g $^ -o $@

clean:
	rm -rf $(targets) *.mod *.log fort.* .*.swp

.phony: clean

run_rv_tests: rv_test
	for file in riscv-tests/images/*; do ./rv_test $$file; done | tee run.log

rv_test: src/rv_tests.cpp $(wildcard src/*.h)
	g++ -O3 -o rv_test src/rv_tests.cpp

emulator: src/emulator_linux.cpp $(wildcard src/*.h)
	g++ -O3 -o emulator src/emulator_linux.cpp

linux: emulator
	./emulator Image

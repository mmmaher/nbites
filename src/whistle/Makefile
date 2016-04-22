CYAN            = `tput setaf 6`
NORMAL          = `tput sgr0`

SRC= *.cpp logcopy/*.cpp timing/*.cpp timing/rdtsc.s
HDR= *.h logcopy/*.h timing/*.h

sound2: $(HDR) $(SRC)
	@echo $(CYAN) "Compiling..." $(NORMAL)
	g++ -std=c++11 -o sound2 $(SRC) -lfftw3f -lasound -lpthread

clean:
	rm sound2

CXXFLAGS ?= -O2 -std=c++17 -pthread
CFLAGS ?= -O2
CPPFLAGS += -Iads1115_rpi_lib-master/src
LDLIBS += -lpigpio -lwiringPi -pthread -lrt

PROGRAMS = build/newmotortest build/testmotor build/test_adcs
ADC_OBJECT = build/ads1115rpi.o

.PHONY: all

all: $(PROGRAMS)

build/%: %.cpp $(ADC_OBJECT) ring.h ads1115_rpi_lib-master/src/ads1115rpi.h
	mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(ADC_OBJECT) $(LDLIBS) -o $@

$(ADC_OBJECT): ads1115_rpi_lib-master/src/ads1115rpi.c ads1115_rpi_lib-master/src/ads1115rpi.h
	mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

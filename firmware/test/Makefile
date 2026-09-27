# Host-side behaviour tests for the flight controller (no Arduino needed).
#   cd firmware/test && make
test: test_fc
	./test_fc
test_fc: test_fc.cpp ../veg_flight_controller/*.ino ../veg_flight_controller/*.h mock/*.h
	g++ -std=gnu++17 -Wall -Imock -o $@ test_fc.cpp
clean:
	rm -f test_fc

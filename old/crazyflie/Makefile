PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/crazyflie.ino.bin

SRC = ../src

any:
	echo "Please specify cf2 or bolt"

bolt: crazyflie.ino  *.cpp *.h $(SRC)/*.hpp $(SRC)/tasks/*.hpp \
	$(SRC)/msp/__messages__.h
	cp bolt/imu.cpp ./__imu__.cpp
	cp bolt/motors.cpp ./__motors__.cpp
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) crazyflie.ino

$(SRC)/msp/__messages__.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../crazyflie

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/crazyflie.ino.bin

clean:
	rm -rf build __*__.cpp

listen:
	miniterm.py $(PORT) 115200 --exit-char 3



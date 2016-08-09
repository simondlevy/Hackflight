
PORT = /dev/ttyUSB0
BAUD = 57600

slam: 
	./main.py $(PORT) $(BAUD)

flakes:
	pyflakes main.py

test:
	./test.py

edit:
	vim microslam.py

clean:
	rm -f *.pyc *~


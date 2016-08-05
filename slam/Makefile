
PORT = /dev/ttyUSB0
BAUD = 57600

slam: 
	./main.py $(PORT) $(BAUD)

flakes:
	pyflakes main.py

edit:
	vim slammin.py

clean:
	rm -f *.pyc *~


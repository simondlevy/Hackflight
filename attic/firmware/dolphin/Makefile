PORT = ACM0

all: build

edit:
	vim dolphin.ino

build: dolphin.ino
	arduino --verify dolphin.ino

upload: dolphin.ino
	arduino --upload dolphin.ino

probe:
	ls /dev/ttyA*

mini:
	miniterm.py /dev/tty$(PORT) 9600 --exit-char=27


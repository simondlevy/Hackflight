install: messages.json msppg.py
	python3 msppg.py
	cp msp.py ../client/__msp__.py
	cp messages.h ../src/msp/__messages__.h

edit:
	vim msppg.py

clean:
	rm -f msp.py messages.h MspParser.java

flake:
	flake8 msppg.py

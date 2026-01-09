install: messages.json msppg.py
	python3 msppg.py
	cp msp.py ../client/
	cp msp.py ../old/client/

edit:
	vim msppg.py

clean:
	rm -f msp.py messages.h MspParser.java

flake:
	flake8 msppg.py

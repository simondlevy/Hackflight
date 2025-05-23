install: messages.json msppg.py
	python3 msppg.py
	cp msp.py ../clients/
	cp messages.h ../src/msp/

edit:
	vim msppg.py

clean:
	rm -f msp.py messages.h MspParser.java

flake:
	flake8 msppg.py

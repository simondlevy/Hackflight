#
# Makefile for Hackflight Copilot tests
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Test.hs \
       ../Utils.hs \

RUNH = runhaskell -i.. Test.hs

test : main.c test.c test.h
	gcc -o test main.c test.c test.h

test.c: $(HSRC) Test.hs
	$(RUNH)

test.h: $(HSRC) Test.hs
	$(RUNH)

clean:
	rm -f test* *~

commit:
	git commit -a -m="More Haskell Copilot work"

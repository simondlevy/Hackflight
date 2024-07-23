# Makefile for ekf.cpp

all: LinAlg ekf.cpp

run: LinAlg
	./LinAlg

ekf.cpp : Ekf.hs
	ghc -package linear Ekf.hs
	./Ekf
	mv ekf.c ekf.cpp

LinAlg: LinAlg.hs
	ghc -package linear LinAlg.hs

clean:
	rm -f Ekf ekf.* *.hi *.o

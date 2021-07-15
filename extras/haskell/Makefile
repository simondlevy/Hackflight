ALL = test
all: $(ALL)

test: test.hs GetKey.hs
	ghc -Wall test.hs

run: test
	./test
clean:
	rm -rf $(ALL) *.hi *.o

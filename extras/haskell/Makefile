#
# Copyright (c) 2021 Simon D. Levy
# 
# MIT License

TARGET = dist/build/hackflight/hackflight

$(TARGET): src/*.hs src/pidcontrollers/*.hs
	cabal build

run: $(TARGET)
	cabal build
	$(TARGET)

clean:
	rm -rf dist/build

commit:
	git commit -a -m="More Haskell work"

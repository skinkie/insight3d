SOURCES=$(wildcard *.cpp)
OBJECTS=$(patsubst %.cpp,%.o,$(SOURCES))
DEBUG= -g
ANN_INCLUDE= -I./ann_1.1.1/include/

all: insight3d

insight3d: $(OBJECTS) sift_detector libsba libANN
	g++ $(DEBUG) -o insight3d *.o `pkg-config --libs opencv libxml-2.0 sdl2 gtk+-2.0` ./sift/lib/libfeat.a $(AGARLIB) -lblas -lGL -lGLU ./sba/libsba.a -llapack ./ann_1.1.1/lib/libANN.a -lpthread -lgdk-x11-2.0 


sift_detector:
	make -C ./sift
	make -C ./ann_1.1.1 linux-g++
	make -C ./sba

libsba:
	make -C ./sba

libANN:
	make -C ./ann_1.1.1 linux-g++

%.o: %.cpp
	g++ $(DEBUG) -c `pkg-config --cflags opencv libxml-2.0 sdl2 gtk+-2.0` $(ANN_INCLUDE) $<

clean: 
	rm *.o
	rm ./insight3d

.PHONY: sift_detector libsba libANN

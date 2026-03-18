CC=g++
CFLAGS=  -I/usr/local/include/opencv4 -L/usr/local/lib -g -Wall
CLIBS=  -lpthread 
OPENCVLIBS = -lopencv_highgui -lopencv_aruco -lopencv_imgcodecs -lopencv_core -lopencv_videoio -lopencv_calib3d -lopencv_imgproc
ZMQLIBS= -lzmq  -lzmqpp
JLIBS= -lSDL2 -lpthread

all:cli

# main: main.cc
# 	$(CC) $(CFLAGS) -o main main.cc  $(CLIBS) $(OPENCVLIBS) $(ZMQLIBS)
cli: src/cli.cc
	$(CC) $(CFLAGS) -o cli src/cli.cc  $(CLIBS) $(OPENCVLIBS) $(ZMQLIBS)
debug: main.cc
	$(CC) $(CFLAGS) -o main main.cc  $(CLIBS) $(OPENCVLIBS) $(ZMQLIBS) --debug

clean:
	rm -rf cli


INCLUDE_DIRS =  -I/usr/local/opencv/include
LIB_DIRS = 
##CC=gcc
CC=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
##may not need it
CPPLIBS= -L/usr/local/opencv/lib -lopencv_core -lopencv_flann -lopencv_video



HFILES= 
CFILES= capture.cpp

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	capture

clean:
	-rm -f *.o *.d
	-rm -f capture
	-rm -f *.ppm
	-rm -f *.log
	-rm -f *.mp4
	-rm -f *.jpg

distclean:
	-rm -f *.o *.d

capture: capture.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS)

##	${OBJS}
##	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:	
	$(CC) $(CFLAGS) -c $<

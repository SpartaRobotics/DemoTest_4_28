# build an executable for JetsonAX12

INCLUDE_DIRS = -I../../include
LIB_DIRS = 
CC = g++
CFLAGS = -g -Wall $(INCLUDE_DIRS) $(CDEFS) -std=c++11
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lopencv_imgproc -lopencv_highgui

HDIR = ../../include
SDIR = ../../src
ODIR = ../../src/obj

LAX28 = UsbMX
LGPIO = jetsonGPIO
LPWMD = JHPWMPCA9685

TARGET = manual

all: $(TARGET)

$(TARGET): $(TARGET).o $(LAX28).o $(LGPIO).o $(LPWMD).o
	$(CC) $(CFLAGS) $< $(ODIR)/$(LAX28).o $(ODIR)/$(LGPIO).o $(ODIR)/$(LPWMD).o -o $@ `pkg-config --libs opencv` $(CPPLIBS)
		
$(TARGET).o: $(TARGET).cpp
	$(CC) $(CFLAGS) -c $< -o $@ 
	
$(LAX28).o: $(SDIR)/$(LAX28).cpp $(HDIR)/$(LAX28).h $(HDIR)/$(LGPIO).h
	$(CC) $(CFLAGS) -c $< -o $(ODIR)/$@ 

$(LGPIO).o: $(SDIR)/$(LGPIO).c	$(HDIR)/$(LGPIO).h
	$(CC) $(CFLAGS) -c $< -o $(ODIR)/$@ 

$(LPWMD).o: $(SDIR)/$(LPWMD).cpp $(HDIR)/$(LPWMD).h
	$(CC) $(CFLAGS) -c $< -o $(ODIR)/$@

target: $(TARGET)

$(TARGET): $(TARGET).o $(LAX28).o $(LGPIO).o $(LPWMD).o
	$(CC) $(CFLAGS) $< $(ODIR)/$(LAX28).o $(ODIR)/$(LGPIO).o $(ODIR)/$(LPWMD).o -o $@ `pkg-config --libs opencv` $(CPPLIBS)

clean:
	$(RM) -f core *.o $(TARGET)

cleanall:
		$(RM) -f core *.o $(ODIR)/*.o $(TARGET) $(SDIR)/*.cpp~ *.cpp~ $(HDIR)/*.h~ 

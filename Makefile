CFLAGS=-I./lmic
LDFLAGS=-lwiringPi -lgps
CC=gcc
mapper: ttnmapper.cpp
	cd ./lmic && $(MAKE)
	$(CC) $(CFLAGS) -o mapper ttnmapper.cpp ./lmic/*.o $(LDFLAGS)

all: mapper

.PHONY: clean

clean:
	rm -f *.o mapper
install:
	sudo cp ./mapper /opt/mapper
	sudo chown root:root /opt/mapper
	sudo cp gpson /usr/bin/
	sudo cp gpsoff /usr/bin/
	cp ttnmapper.service  /lib/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable ttnmapper
	sudo service ttnmapper start

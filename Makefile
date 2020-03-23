9spi: spi.c libeval.c
	gcc -g libeval.c libcrc16.c spi.c -o 9spi -I/usr/include/python3.6m/ -lpython3.6m
exlib:
	gcc -g spi.c -o 9spi -I/usr/include/python3.6m/ -lpython3.6m
full: spi.c libcrc16.c
	gcc -g libcrc16.c spi.c -o 9spi -I/usr/include/python3.6m/ -lpython3.6m
f: spi.c libcrc16.c
	gcc -g libcrc16.c spi.c -o 9spi -I/usr/include/python3.6m/ -lpython3.6m
clean:
	rm 9spi

install: 
	 groupadd -f --system gpio
	 chgrp gpio ./9spi 
	 chmod u=rwxs,g=rx,o= ./9spi

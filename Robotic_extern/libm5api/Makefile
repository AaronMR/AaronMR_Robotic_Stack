all: libm5api

libm5api:
	# create link to lib depending on version (32-bit/64-bit)
	cd common/lib && ln -sf `uname -m`/* .
clean:
	rm -rf common/lib/*.so
wipe: clean
	-rm -rf common/lib/*.so
	

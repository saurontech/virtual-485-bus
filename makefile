all:advvbus

SRCS = $(shell find ./ -name '*.c')
HDRS = $(shell find ./ -name '*.h')
INCLUDE = $(shell find . \( ! -regex '.*/\..*' \) -type d -printf ' -I ./%P ' )


advvbus: ${SRCS} ${HDRS}
	${CC}  -o $@ ${SRCS} ${INCLUDE}

clean:
	rm -f ./advvbus

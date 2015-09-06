def: test .compiled run

REMOTE=odroid@odroid
#SRC=linux-i2c-test.c
SRC=test1.c
CCFLAGS=-Wall -Werror -g

test:
	gcc ${CCFLAGS} -o /tmp/ignore ${SRC}

.compiled: ${SRC}
	ssh ${REMOTE} cat '>/tmp/code.c' '&&' \
		gcc -o /tmp/code.exe ${CCFLAGS} /tmp/code.c < ${SRC}
	date > .compiled

run: .compiled
	ssh ${REMOTE} -t cd /tmp '&&' time ./code.exe

runget: run get

get:
	rsync -v ${REMOTE}:/tmp/acc-log-*.txt acc-log-`date +'%F-%H%M%S'`/

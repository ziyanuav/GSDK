#!/bin/bash

g++ -Wall -fPIC -g -I./include -fpermissive -DNO_STD_LIB -DSIMULATION  -DINCLUDE_JINGANSDK  -w  -o ./bin/test ./Example/main.cpp -L./lib -lZiyan -lssl -lcrypto -lavcodec -lavdevice -lswresample -lavformat -lavutil -lcrypt -lpcre2-8 -lz -lcurl -lzip  -ltinyxml2  -ljaiotcppsdk  -lssl -lcrypto -lmysqlclient -lpcre2-8 -lpcre2-posix

export LD_LIBRARY_PATH=$(pwd)/lib:$LD_LIBRARY_PATH

./bin/test
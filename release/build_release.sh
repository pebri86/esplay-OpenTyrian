#!/bin/sh

export IDF_PATH=~/esp/esp-idf/
cd ..
make -j4
cd release
export PATH=/c/ffmpeg/bin:$PATH

ffmpeg -i Tile.png -f rawvideo -pix_fmt rgb565 tile.raw
/home/Peruri/esp/esplay-base-firmware/tools/mkfw/mkfw.exe OpenTyrian tile.raw 0 16 1048576 app ../build/OpenTyrian.bin
rm OpenTyrian.fw
mv firmware.fw OpenTyrian.fw

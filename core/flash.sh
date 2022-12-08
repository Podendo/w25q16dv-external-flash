#!/bin/bash


echo building project...
echo
echo $(make firmware.bin)
wait $!
echo

echo flashing binary file to target...
echo
echo $(st-flash --reset --format binary write firmware.bin 0x08000000)
wait $!
echo
echo cleaning binaries...
echo $(make clean)
echo
echo done!
echo
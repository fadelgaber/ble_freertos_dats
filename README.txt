Name:
=====
 ble_freertos_dats


Description:
============
 ARM Cordio BLE - dats Server (Slave) Example.


Purpose:
========
This example is the server (slave) for the BLE dats.  
This example is meant to run on an Apollo3 EVB
along with another Apollo3 EVB serving as the client.  This example waits
for connection and initiation of data transfers by the client (master).

Printing takes place over the uart with 115200.

folder src : contains the sources information
folder gcc : contains the compile information

============
to compile:
============
cd gcc
./cmp   (is a batch file to save a lot of typing)
 
=======================
to load to edge board :
=======================
first make sure it is compiled, then

cd gcc
./load   (is a batch file to save you a lot of typing)

It will try to upload assuming you have the asb bootloader, sets the port and speed at 961200 baud.
else change this file

make sure to press the 'button 14' on the Edge to enable bootloader.

=========
to clean:
========
cd gcc
./clean

=========================
Bleutooth device address:
=========================
As it currently set, you will get the same devices address after each build, but if you
want a different each build then :

cd gcc
nano Makefile
around line 148 :

#setting custom_bdaddr will cause a different Blue device address at each start
# DEFINES+= -DAM_CUSTOM_BDADDR

remove the # before DEFINES, save and recompile ( e.g. ./clean  ./cmp)



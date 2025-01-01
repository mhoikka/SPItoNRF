To start: Image raspberry pi using the files in the RaspberryPiZero2W_Image folder, or use your own image. 
To do the latter: 
-Enable SPI 
-Get Software Packages
-Clone project
-Build project

After copying Pi image or updating your own image:
-Run project 
-Start the NRF transmissions from the PCB

If any changes are made to a forked Git repository, or if the author's repo is updated, follow the Build Project steps again to update local code



Get Software Packages
$ sudo apt-get install git-core
$ gpio -v (If this does not exist on your device, follow next step)

[$ sudo apt-get install --only-upgrade gpio]

$ sudo apt-get purge wiringpi
$ hash -r
$ git clone https://github.com/WiringPi/WiringPi.git
$ cd WiringPi
$ git pull origin
$ ./build

$ gpio -v
$ gpio readall
[$ ls /dev/*spi* to check SPI devices]

Enable SPI
-On raspberry pi Zero 2 W, click on the Raspberry Icon and navigate to Preferences/Raspberry Pi Communication/Interfaces, then click the button to enable SPI communication and click 'OK'
-Restart pi

Clone project
-enter in cmd terminal:
$ git clone https://github.com/mhoikka/stm32f0-discovery-basic-bme280_test.git

Build project
-enter in cmd terminal:
$ git pull (skip this step if project was just cloned)
$ gcc spitest.c -l wiringPi -o nrftest

Run project
-attach NRF module with jumper wires to header pins of pi as shown in the KiCAD schematic in the KICAD_schematics folder
-enter in cmd terminal:
$ cd /SPItoNRF
$ ./nrftest





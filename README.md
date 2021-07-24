# temperature

A Raspberry PI PICO project with an I2C connected BMP280 temperature and pressure module.

Follow the setup suggested [here](https://github.com/smart-t/pico-docs) to ensure that environment and pico-sdk are in the right place. Then:
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

After these theps the .uf2 is created. Press the button on the PICO, then plug the device to your PC and drag drop the .uf2 binary in the PICO folder. After this the project is running, indicated by the blinking light and temperature being written to the console.

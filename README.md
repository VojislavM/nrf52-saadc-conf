# ble_app_beacon_pascana

### setup
* download segger embedded studio 
* install J-link tools 
* download nRF SDK 15.3.0 
* clone repo in SDK folder: ../nRF5_SDK_15.3.0/examples/ble_peripherial/

### compile
* open segger embedded studio 
* load project 
* build project (F7)

### downalod project
* merge with SoftDvice hex command: mergehex -m firmware.hex softdevice.hex -o output.hex
* download the program by coping it to the device folder



# firmware_functional_specification

* Broadcast BLE ADV packets each 5 seconds. The packets contain some 
unique identifier bound to a device.
* When button is pushed ones change brodcast time to 1 seconds and 
change BLE ADV data packet
* ADV data struct:  
	typedef struct  
	{  
		&nbsp;&nbsp;&nbsp;&nbsp;uint8_t battery; //battery level  
		&nbsp;&nbsp;&nbsp;&nbsp;uint8_t status;  //status of a button pressed  
		&nbsp;&nbsp;&nbsp;&nbsp;uint8_t txPower; //ble tx power selected  
		&nbsp;&nbsp;&nbsp;&nbsp;const uint8_t build_version = VERSION_BUILD; //current 
firmware version  
	}AdvManufactureDataT;  
​
* after entering alarm state timeout is activatted after 15min and device is back in normal state.
* pushing button longer then 5 sec will turn off device, diode will be on for 2 sec before off state
* pushing button will turn on device (if the previous state was off)

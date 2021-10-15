## Firmware Installation (AI Thinker ESP32 CAM)

Make connections aa follows:

![ESP32-CAM-FTDI-programmer-5V-supply](C:/Users/leungp/Documents/GitHub/clamp_controller/camera_firmware/esp32%20hardware/ESP32-CAM-FTDI-programmer-5V-supply.png)



In Arduino IDE, open `esp32cam_firmware.ino` : 

   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
   - Upload



Confirm by opening Serial Monitor at selected COM port: 

```
Camera initialized.....
WiFi connected
Camera Stream Ready! Go to: http://192.168.1.2
```



The video streaming firmware is a fork of www.github.com/bnbe-club/rtsp-video-streamer-diy-14 (YouTube Video: https://youtu.be/1xZ-0UGiUsY). The M5CAM support is coded by https://github.com/bokse001 

The options `ENABLE_WEBSERVER` and `ENABLE_RTSPSERVER` are both enabled. However, we found the webserver to be much faster without delay.




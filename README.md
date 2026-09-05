This is a i2s to Bluetooth bridge for Martendo32 or other retro-go based devices.
You can enjoy fine stereo sound with the Martendo32 Mp3 Player App ;)
Simply use an M5 Atom Echo and connect it to the Martendo32.
Just put the Bluetooth Speaker / Headphones in pairing mode, and press the pairing button on the M5.
Once paired, it will re-connect on boot to the last paired device.

<img width="796" height="284" alt="image" src="https://github.com/user-attachments/assets/905247ed-44c6-47ca-95da-ff6137382512" />



Pin-Mapping
-----------
ESP32 M5Atom <->      Martendo

GND          <->      GND	

GPIO 19      <->      NS4168 PIN3 (I²S BCLK)

GPIO 33      <->      NS4168 PIN2 (I²S LRCK)

GPIO 22      <->      NS4168 PIN4 (I²S DATA)

GPIO 39      ->       Pairing Button (connect Button to GND and GPIO39)

GPIO 27      ->       Status LED (SK6812, NEOPIXEL LED)


 Status LED:
 ----------
 - green — streaming
 - blue — connected, no audio
 - orange pulse — connecting
 - purple pulse — disconnecting
 - red pulse — searching

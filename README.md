# RSC
Based on ESP32-based solution from https://github.com/imwitti/FootpodMimic

ESP32-software for making your treadmill smart. 

The setup is using a standard ESP32-board, with a Hall sensor (TW-492) attached to pin 26 (and 3.3V + GND from the board also). Pair with your phone and connect to the Zwift app to run online with others. The distance per revolution probably has to be adjusted to your treadmill. 

Attach the hall sensor to pin 26 and a digital sound threshold sensor to pin 25. The sound sensor is triggered by your footsteps hitting the mill, and is used to measure steps/cadence. The hall sensor measures rotations on the treadmill. You need a magnet on a rotating part of the treadmill, which is most likely already there for the treadmills own operation. See the video at https://youtu.be/-dS94BYqaKo  

Use on your own risk. 

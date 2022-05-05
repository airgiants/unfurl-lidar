# unfurl-lidar
TG30 lidar code for Beaglebone for Unfurl

Needs the YDLidar TG30 SDK installed from 

https://github.com/airgiants/YDLidar-SDK
then follow the instructions in doc/howto to install

To build, open in VSCode and do menu/Terminal/Run Build Task

or

/usr/bin/g++ -fdiagnostics-color=always -g /home/richard/unfurl-lidar/*.cpp -o /home/richard/unfurl-lidar/unfurl-lidar -I/usr/local/include/core/base -I/usr/local/include/core/common -I/usr/local/include/core/math -I/usr/local/include/core/network -I/usr/local/include/core/serial -I/usr/local/include/core/serial/impl -I/usr/local/include/src -lydlidar_sdk -lpthread




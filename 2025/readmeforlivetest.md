To get the LiDAR to have somewhat of a live obstacle avoidance test you have to first make sure nothing is running with the Arduino

This is very important or else MATLAB won't let you run the 'arduino.mlx' program
(ALSO, make sure to get the Arduino Hardware Support Add-On in MATLAB... actually idk if this made a difference bc I got a port pop up once and then never again so... yeah)
Actually, first step is getting the LiDAR to be connected and running

1. Power on the LiDAR
2. Connect it to the Ethernet port
3. Go to 'Network' in 'Settings' -> the Ethernet port should be a solid green color - sometimes turning off WiFi helps

SO now..... in the terminal run:

```
lsof | grep tty.usbmodem
```

Whatever the four digit sequence is, is what you have to kill:
```
kill -9 <PID value>
```
You have to do this EVERYTIME you run the MATLAB code... unfortunately

Okay.... Now we get to the fun part
The main problem we had was that MATLAB takes only plain, original .PCAP files not .PCAPPNG which is what tshark was giving us. So we switched to tcpdumps.

Here are the steps I followed... Mind you I am running everything on a Mac Mini...

```
tshark -D 
```
This is to see which network the LiDAR is connected to. Make a folder somewhere to store you .PCAP files. I manually delete the .PCAP files before each run just so that I can start fresh.
```
sudo tcpdump -i en12 -G 0.5  -w ~/Documents/lidar_pcaps/lidar_%s.pcap udp port 2368
```
Ok and that's everything! 

P.S. Arduino can only work using live scripts in MATLAB so that's why the file is in .mlx and not .m

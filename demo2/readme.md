This is our code for demo2
Our robot raspberry pi identifies the aruco marker angle and distance, and communicates the distance to the arduino

The control system approaches the marker and draws a loop around it
It is integrated and works. 
this code uses the same calibration files from demo 1 under/demoone/
After running these files and wireing propperly you are able to run the demo2 python code and it should work
if you want to recalibrate it, you must change the x, y, h, k, and other files generated by the calibration that are
hardcoded in demo 2
We are planning to make this automated however we couldnt figure out
how to get the xml to output propperly when working with one variable
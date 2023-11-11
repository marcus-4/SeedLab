import numpy as np
import cv2 as cv
import glob
import struct
from cv2 import aruco
from time import sleep

from smbus2 import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd



ADR_ADDR = 8

i2cSCREEN = board.I2C()
#removed code for debugging
#lcd_columns = 16
#lcd_rows = 2

#initilize lcd
#lcd = character_lcd.Character_LCD_RGB_I2C(i2cSCREEN, lcd_columns, lcd_rows)
#lcd.clear()

#lcd = character_lcd.Character_LCD_RGB_I2C(i2cSCREEN, lcd_columns, lcd_rows)
#lcd.clear()
i2c = SMBus(1)
# termination criteria

cv_file = cv.FileStorage()
cv_file.open('calib.xml', cv.FileStorage_READ)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
fileName = "filler"
#pull out calibration for cam
mapx = cv_file.getNode('mapx').mat()
mapy = cv_file.getNode('mapy').mat()
dist = cv_file.getNode('dist').mat()
mtx = cv_file.getNode('mtx').mat()
img = cv.imread('images/stereoLeft/imageL6.png')
#couldnt make it import one variable for some reason so we hardcoded this part
x = 4
y = 3
w = 632
h = 470

cap_left =  cv.VideoCapture(0)
#undistort using calibration
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image to usable stuff

dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png', dst)
#look for marker loop
while(cap_left.isOpened()):
    offset = 2
    succes_left, frame_left = cap_left.read()
    #crop image to usable
    frame_left = cv.remap(frame_left, mapx, mapy, cv.INTER_LINEAR)
    frame_left = frame_left[y:y+h, x:x+w]
    

    gray = cv.cvtColor(frame_left, cv.COLOR_BGR2GRAY)
    
    param = cv.aruco.DetectorParameters()
    
    corners, ids, rejects = cv.aruco.detectMarkers(gray, aruco_dict)
    markersize = 10
    #marker variable detection
    #two vectors for values to send to arduino for distance
    rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markersize, mtx, dist)
    ang= 0
   
    ww = 0.5*627
    hh = 0.5*466
    #magic number for height based on camera output view
    fov =54.5/2

    #detection 
    overlay = cv.cvtColor(gray,cv.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
    overlay = aruco.drawDetectedMarkers(frame_left,corners,borderColor =(255,0,0))
    cv.imshow("overlay",overlay)

    
        

    

    

    
    try:
        #if there is a marker 
        #take top left and bottom right
        corner_x1 = corners [0] [0] [0] [0]
        corner_y1 = corners [0] [0] [0] [1]
        corner_x2 = corners [0] [0] [2] [0]
        corner_y2 = corners [0] [0] [3] [1]
        #print(corner_x1)
        #print(corner_x2)
        #pull outratio numberfrom corners
        #calculate pixel values for angles
        rx = ((ww-(.5*(corner_x1+corner_x2)))/ww)
        ry = ((hh-(.5*(corner_y1+corner_y2)))/hh)

        #anglefrom ratio and fov of cam
        angx = int(rx*fov)
        angy = int(ry*fov)
        
        line_1 = 'x '+str(angx) + '\n'

        #new send
        #if a marker is found no need to search so search = 0
        search = int(0)
        far =  int(tvec [0] [0] [2])
        
        #easy way to send multiple thiungs to the arduino
        data = [search, angx, far]
        
        print(data)
        

        

        
        
    except:
        #print none found\
        #search since no markers found send to arduino to make it spin untill a new one is found
        angx = int(0)
        angy = 0
        far = int(0)
        
        search = int(1)
        
        data = [search, angx, far]
        print(data)
        #print(data)
        #i2c.write_byte_data(ADR_ADDR,offset,'test')
        
    try:
        i2c.write_block_data(ADR_ADDR,offset,data)
        sleep(0.05)
        #if arduino is off and python is still running it wont crash out
    except:
        print("no connection to arduino")
        
    #leave statement
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

#release all stuff 
cap_left.release()

#will probably remove windows later for better timing
cv.destroyAllWindows()


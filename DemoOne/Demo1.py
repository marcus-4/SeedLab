import numpy as np
import cv2 as cv
import glob
from cv2 import aruco

from smbus2 import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd



ADR_ADDR = 8
lcd_columns = 16
lcd_rows = 2

i2cSCREEN = board.I2C()
#initilize lcd
lcd = character_lcd.Character_LCD_RGB_I2C(i2cSCREEN, lcd_columns, lcd_rows)
lcd.clear()
i2c = SMBus(1)
lcd = character_lcd.Character_LCD_RGB_I2C(i2cSCREEN, lcd_columns, lcd_rows)
lcd.clear()
i2c = SMBus(1)
# termination criteria

cv_file = cv.FileStorage()
cv_file.open('calib.xml', cv.FileStorage_READ)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
fileName = "filler"
#pull out calibration for cam
mapx = cv_file.getNode('mapx').mat()
mapy = cv_file.getNode('mapy').mat()
"""
x = cv_file.getNode('x').mat()
y = cv_file.getNode('y').mat()
w = cv_file.getNode('w').mat()
h = cv_file.getNode('h').mat()
"""
img = cv.imread('images/stereoLeft/imageL6.png')

x = 58
y = 55
w = 570
h = 394

cap_left =  cv.VideoCapture(0)
#undistort using calibration
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image to usable stuff
lcd.clear()
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png', dst)
#look for marker loop
while(cap_left.isOpened()):
    succes_left, frame_left = cap_left.read()
    #crop image to usable
    frame_left = cv.remap(frame_left, mapx, mapy, cv.INTER_LINEAR)
    frame_left = frame_left[y:y+h, x:x+w]
    

    gray = cv.cvtColor(frame_left, cv.COLOR_BGR2GRAY)
    
    param = cv.aruco.DetectorParameters()
    
    corners, ids, rejects = cv.aruco.detectMarkers(gray, aruco_dict)
    ang= 0
   
    ww = 0.5*510
    lcd.clear()
    hh = 0.5*338
    #magic number for height based on camera output view
    fov =54.5/2

    #detection
    overlay = cv.cvtColor(gray,cv.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
    overlay = aruco.drawDetectedMarkers(frame_left,corners,borderColor =(255,0,0))
    cv.imshow("overlay",overlay)
    try:
        #if corner take top left and bottom right
        corner_x1 = corners [0] [0] [0] [0]
        corner_y1 = corners [0] [0] [0] [1]
        corner_x2 = corners [0] [0] [3] [0]
        corner_y2 = corners [0] [0] [3] [1]

        #pull outratio numberfrom corners
        rx = ((ww-(.5*(corner_x1+corner_x2)))/ww)
        ry = ((hh-(.5*(corner_y1+corner_y2)))/hh)

        #anglefrom ratio and fov of cam
        angx = int(rx*fov)
        angy = int(ry*fov)

        #print angles
        line_1 = 'x '+str(angx) + '\n'
        #line_2 = 'y '+str(angy)
        lcd.message = line_1#+line_2
        
        
    except:
        #print none found
        line_1 = 'x '+"none" + '\n'
        #line_2 = 'y '+"none"
        lcd.message = line_1#+line_2
        angx = 0
        angy = 0
    #leave statement
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

#release all stuff 
cap_left.release()


cv.destroyAllWindows()
lcd.clear()



    

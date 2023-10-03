#Marcus Becker


import threading
from time import sleep
import numpy as np
import cv2
import queue
from cv2 import aruco
from smbus2 import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def lcdPrint(state):
        #used to write to lcd for what quadrant we are in
        if(state == 1):
                lcd.message = ("[0,0]")
        elif(state == 2):
                lcd.message = ("[0,1]")
        elif(state == 3):
                lcd.message = ("[1,1]")
        elif(state == 4):
                lcd.message = ("[1,0]")
        else:
                lcd.message = ("none ")



ADR_ADDR = 8
lcd_columns = 16
lcd_rows = 2

i2cSCREEN = board.I2C()
#initilize lcd
lcd = character_lcd.Character_LCD_RGB_I2C(i2cSCREEN, lcd_columns, lcd_rows)
lcd.clear()
i2c = SMBus(1)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
fileName = "filler"

#fileName = input("File Name: ")



# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
# Let the camera warmup
sleep(0.7)
	
# Get an image from the camera stream
#corners = np.zeros(6)
#ids = np.zeros(6)
#state = 0
#myThread = threading.Thread(target=lcdPrint,args=(state,))
#myThread.start()
while True:
        
        
        
        
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        offset = 2
        # Capture frame-by-frame
        ret, image = camera.read()
        
        if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #canny = cv2.Canny(image,100,200)
        param = cv2.aruco.DetectorParameters()
        #corners, ids, rejects = cv2.aruco.ArucoDetector.detectMarkers(gray, aruco_dict, param)
        corners, ids, rejects = detector.detectMarkers(gray)

        #frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

        overlay = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
        overlay = aruco.drawDetectedMarkers(image,corners,borderColor =(255,0,0))
        try:
                #print(corners [0] [0] [0] [0])
                #print(corners [0] [0] [0] [1])
		#pulls from the tuple made by marker corners and does math to find positive or negative
                corner_x = (175 - corners [0] [0] [0] [0])
                corner_y = (150 - corners [0] [0] [0] [1])
                #print(corner_x)
                #print(corner_y)
		#generates a state based on what quadrant we are in
                if((corner_x >= 0) & (corner_y >= 0)):
                        state = 2
                elif((corner_x <= 0) & (corner_y >= 0)):
                        state = 1
                elif((corner_x <= 0) & (corner_y <= 0)):
                        state = 4 
                elif ((corner_x >= 0) & (corner_y <= 0)):
                        state = 3
                print(state)
		#writes to the arduino what quad we are in
                i2c.write_byte_data(ADR_ADDR,offset,state)
                lcdPrint(state)
        except:
            print("None found")
            

            
        if not ids is None:
                ids = ids.flatten()
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1])),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
                        
        #shows overlay of lines to show quads                
        #cv2.imshow("overlay",overlay)
        start_hori = (175,0)
        end_hori = (175,300)
        start_vert =(0,150)
        end_vert = (400,150)
        color = (0,255,0)
        thickness = 2
        overlay = cv2.line(overlay, start_hori, end_hori, color, thickness)
        overlay = cv2.line(overlay, start_vert, end_vert, color, thickness)

        cv2.imshow("overlay",overlay)
       # xmid = ((overlay.shape().at(0))/2)
       # ymid = (overlay.shape()[1]/2)
       # xsiz = (overlay.shape()[0])
       # ysiz = (overlay.shape()[1])
       # print('test', xmid, ymid, xsiz, ysiz)
       # overlay = cv2.line(overlay, (xmid,0), (xmid,ysiz), (0,255,0), thickness=2)

        #cv2.imshow('Image',frame_markers)
        if cv2.waitKey(1) == ord('q'):
                break


cv2.destroyAllWindows()


	
# Save the image to the disk
print("Saving image "+fileName)
try:
	#cv2.imwrite(f'{fileName}.jpg',image)
        print("didnt save")
        pass
except:
	print("Could not save "+fileName)
	pass


	

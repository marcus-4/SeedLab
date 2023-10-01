#Marcus Becker

from time import sleep
import numpy as np
import cv2
from cv2 import aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
fileName = "filler"

#fileName = input("File Name: ")
	
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
	
# Let the camera warmup
sleep(0.3)
	
# Get an image from the camera stream
#corners = np.zeros(6)
#ids = np.zeros(6)

while True:
        # Capture frame-by-frame
        ret, image = camera.read()
        
        if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #canny = cv2.Canny(image,100,200)
        param = cv2.aruco.DetectorParameters()
        #corners, ids, rejects = cv2.aruco.ArucoDetector.detectMarkers(gray, aruco_dict, param)
        corners, ids, rejects = cv2.aruco.detectMarkers(gray, aruco_dict)

        #frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

        overlay = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR) # Convert back to RGB for imshow, as well as for the next step
        overlay = aruco.drawDetectedMarkers(image,corners,borderColor =(255,0,0))
        
        if not ids is None:
                ids = ids.flatten()
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int (markerCorners[0,1])),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)

                        
        cv2.imshow("overlay",overlay)
        xmid = ((overlay.shape().at(0))/2)
        ymid = (overlay.shape()[1]/2)
        xsiz = (overlay.shape()[0])
        ysiz = (overlay.shape()[1])
        print('test', xmid, ymid, xsiz, ysiz)
        overlay = cv2.line(overlay, (xmid,0), (xmid,ysiz), (0,255,0), thickness=2)

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


	


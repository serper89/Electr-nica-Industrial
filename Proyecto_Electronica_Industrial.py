import cv2
import time
import serial
import numpy as np

cap = cv2.VideoCapture(0)
window_name = "Camara"
cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
azulBajo = np.array([100,100,20],np.uint8)
azulAlto = np.array([125,255,255],np.uint8)

ser = serial.Serial(port='COM8', baudrate=115200, timeout=1)

while True:
  ret,frame = cap.read()
  if ret==True:
    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frameHSV, azulBajo, azulAlto)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
      area = cv2.contourArea(c)
      if area > 1000:
        M = cv2.moments(c)
        if (M["m00"]==0): M["m00"]=1
        x = int(M["m10"]/M["m00"])
        y = int(M['m01']/M['m00'])
        #cv2.circle(frame, (x,y), 4, (0,0,255), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, '{},{}'.format(x,y),(500,50), font, 0.75,(0,255,0),1,cv2.LINE_AA)
        nuevoContorno = cv2.convexHull(c)
        cv2.drawContours(frame, [nuevoContorno], 0, (255,0,0), 3)
        angulox=round(((0.0920*x)+51),1)
        anguloy=round(((-0.0936*y)+102),1)
        #cv2.putText(frame, '{},{}'.format(angulox,anguloy),(0,100), font, 0.75,(0,255,0),1,cv2.LINE_AA)
        
        ser.write((str(angulox+2)+' '+str(anguloy+2.5)+'\n').encode())
          
    cv2.imshow(window_name,frame)
    if cv2.waitKey(1) & 0xFF == ord('s'):
      break
cap.release()
cv2.destroyAllWindows()






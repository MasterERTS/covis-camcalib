import cv2
import sys
import os


ref = sys.argv[1]
carry_on = True

def saveim(evt, x, y, flags, im):
	if evt == cv2.EVENT_LBUTTONDOWN:
		i = 0
		while os.path.exists(ref + str(i)+'.jpg'):
			i+=1
		cv2.imwrite(ref + str(i)+'.jpg', im)
		print('Writing %s%i.jpg' % (ref,i))	
        elif evt == cv2.EVENT_RBUTTONDOWN:
            sys.exit(0)

cap = cv2.VideoCapture(0)
cv2.namedWindow(ref)


while cap.isOpened() and carry_on:
	r,im = cap.read()
	cv2.setMouseCallback(ref, saveim, im)
	cv2.imshow(ref, im)
	cv2.waitKey(1)

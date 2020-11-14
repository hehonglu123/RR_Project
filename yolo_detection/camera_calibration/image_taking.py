import cv2, time
capture=cv2.VideoCapture(0)


for i in range(22):
	ret, frame=capture.read()
	cv2.imwrite("calibration"+str(i+22)+".jpg",frame)
	time.sleep(5)



# cv2.imshow("image", frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

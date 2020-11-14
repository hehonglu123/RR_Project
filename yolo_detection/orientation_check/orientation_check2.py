import numpy as np
import cv2, time
BACKGROUND=150
RESOLUTION=1
def square(im,desired_size):
	image=BACKGROUND*np.ones((desired_size, desired_size),np.uint8)
	shape_diff=[desired_size-im.shape[0],desired_size-im.shape[1]]
	start=[round(shape_diff[0]/2),round(shape_diff[1]/2)]
	end=[desired_size-round(shape_diff[0]/2),desired_size-round(shape_diff[1]/2)]
	image[start[0]:end[0],start[1]:end[1]]=cv2.resize(im,(end[1]-start[1],end[0]-start[0]))

	return image
def rotate_image(mat, angle):
	"""
	Rotates an image (angle in degrees) and expands image to avoid cropping
	"""

	height, width = mat.shape[:2] # image shape has 3 dimensions

	image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

	rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

	# rotation calculates the cos and sin, taking absolutes of those.
	abs_cos = abs(rotation_mat[0,0]) 
	abs_sin = abs(rotation_mat[0,1])

	# find the new width and height bounds
	bound_w = int(height * abs_sin + width * abs_cos)
	bound_h = int(height * abs_cos + width * abs_sin)

	# subtract old image center (bringing image back to origo) and adding the new image center coordinates
	rotation_mat[0, 2] += bound_w/2 - image_center[0]
	rotation_mat[1, 2] += bound_h/2 - image_center[1]

	# rotate image with the new bounds and translated rotation matrix
	rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h),cv2.BORDER_CONSTANT,borderValue=BACKGROUND)
	return rotated_mat


def orientation(gt_tensor,bound_img):
	return RESOLUTION*np.argmin(np.linalg.norm(gt_tensor-bound_img,axis=(1,2)))



im_pth = "soap.jpg"
gt = cv2.cvtColor(cv2.imread(im_pth),cv2.COLOR_BGR2GRAY)
max_size=170


test_im= "training_soap-13.jpg"
test_im = cv2.imread(test_im)
test_soap=cv2.cvtColor(test_im[264:264+164,229:229+143,:],cv2.COLOR_BGR2GRAY)

gt_tensor=np.zeros((int(360/RESOLUTION),max_size,max_size))
for i in range(0,360,RESOLUTION):
	gt_tensor[int(i/RESOLUTION)]=square(rotate_image(gt,i),max_size)

now=time.time()
print(orientation(gt_tensor,square(test_soap,max_size)))
print(time.time()-now)
# cv2.imshow("image", rotate_image(gt,236))
# cv2.waitKey(0)
# cv2.destroyAllWindows()
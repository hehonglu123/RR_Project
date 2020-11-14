import numpy as np
import cv2
BACKGROUND=[145,150,189]
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
def square(im,desired_size):
	old_size = im.shape[:2] # old_size is in (height, width) format

	ratio = float(desired_size)/max(old_size)
	new_size = tuple([int(x*ratio) for x in old_size])

	# new_size should be in (width, height) format

	im = cv2.resize(im, (new_size[1], new_size[0]))

	delta_w = desired_size - new_size[1]
	delta_h = desired_size - new_size[0]
	top, bottom = delta_h//2, delta_h-(delta_h//2)
	left, right = delta_w//2, delta_w-(delta_w//2)

	new_im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT,value=BACKGROUND)
	return new_im
def orientation(gt_img,bound_img):
	if np.amax(bound_img.shape)>np.amax(gt_img.shape):
		desired_size=np.amax(bound_img.shape)
	else:
		desired_size=np.amax(gt_img.shape)

	bound_img_temp=square(bound_img,desired_size)
	min_error=9999999
	act_angle=0
	for angle in range(0,360,5):
		gt_img_temp=square(rotate_image(gt_img,angle),desired_size)
		diff=np.linalg.norm(cv2.subtract(gt_img_temp,bound_img_temp))

		if diff<min_error:
			min_error=diff
			act_angle=angle

	return act_angle





# im_pth = "soap.jpg"
# gt = cv2.imread(im_pth)


# test_im= "training_soap-13.jpg"
# test_im = cv2.imread(test_im)
# test_soap=test_im[264:264+164,229:229+143,:]

# print(orientation(gt,test_soap))
# cv2.imshow("image", rotate_image(gt,236))
# cv2.waitKey(0)
# cv2.destroyAllWindows()
import yaml
with open('camera.yaml') as file:
	cam=yaml.load(file)['cam_coordinates']
print(cam)
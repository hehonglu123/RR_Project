import yaml

client_ur = {"robot_name":"ur","height":0.87,"obj_namelists":['tp'],"home":[-0.3,0.2,0.3],"pick_height":0.03,"place_height":0.11,"calibration_start":[-0.38,0.1,-0.08],"calibration_speed":0.07,"url":'rr+tcp://bbb2.local:58652?service=ur_robot'}
client_sawyer = {"robot_name":"sawyer","height":0.78,"obj_namelists":['bt','sp'],"home":[-0.1,0.3,0.3],"pick_height":0.105,"place_height":0.15,"calibration_start":[0.6,-0.2,0.155],"calibration_speed":0.05,"url":'rr+tcp://bbb1.local:58654?service=sawyer'}
client_abb = {"robot_name":"abb","height":1.0,"obj_namelists":['pf'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://bbb3.local:58655?service=staubli'}
client_staubli = {"robot_name":"staubli","height":1.0,"obj_namelists":['sp'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://bbb4.local:58656?service=abb'}

with open(r'client_ur.yaml', 'w') as file:
    documents = yaml.dump(client_ur, file)
with open(r'client_sawyer.yaml', 'w') as file:
    documents = yaml.dump(client_sawyer, file)
with open(r'client_abb.yaml', 'w') as file:
    documents = yaml.dump(client_abb, file)
with open(r'client_staubli.yaml', 'w') as file:
    documents = yaml.dump(client_staubli, file)


with open(r'client_ur.yaml') as file:
    documents = yaml.load(file, Loader=yaml.FullLoader)
    print(documents)
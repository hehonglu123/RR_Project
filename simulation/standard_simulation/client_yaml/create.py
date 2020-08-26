import yaml

client_ur = {"robot_name":"ur","height_offset":0.0,"obj_namelists":['toothpaste'],"home":[0.3,0.0,0.1],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58653?service=robot'}
client_sawyer = {"robot_name":"sawyer","height_offset":0.15,"obj_namelists":['bottle'],"home":[0.,0.3,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58654?service=robot'}
client_abb = {"robot_name":"abb","height_offset":0.21,"obj_namelists":['perfume'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58655?service=robot'}
client_staubli = {"robot_name":"staubli","height_offset":0.21,"obj_namelists":['soap'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58656?service=robot'}

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
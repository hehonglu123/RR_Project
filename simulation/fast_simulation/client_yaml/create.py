import yaml

client_ur1 = {"robot_name":"ur1","height_offset":0.0,"obj_namelists":['toothpaste'],"home":[0.3,0.0,0.1],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58653?service=robot'}
client_ur2 = {"robot_name":"ur2","height_offset":0.0,"obj_namelists":['toothpaste'],"home":[0.3,0.0,0.1],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58653?service=robot'}
client_ur3 = {"robot_name":"ur3","height_offset":0.0,"obj_namelists":['toothpaste'],"home":[0.3,0.0,0.1],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58653?service=robot'}
client_ur4 = {"robot_name":"ur4","height_offset":0.0,"obj_namelists":['toothpaste'],"home":[0.3,0.0,0.1],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58653?service=robot'}

client_abb1 = {"robot_name":"abb1","height_offset":0.21,"obj_namelists":['perfume'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58655?service=robot'}
client_abb2 = {"robot_name":"abb2","height_offset":0.21,"obj_namelists":['perfume'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58655?service=robot'}
client_abb3 = {"robot_name":"abb3","height_offset":0.21,"obj_namelists":['perfume'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58655?service=robot'}
client_abb4 = {"robot_name":"abb4","height_offset":0.21,"obj_namelists":['perfume'],"home":[0.3,0.0,0.3],"pick_height":-0.03,"place_height":-0.1,"url":'rr+tcp://localhost:58655?service=robot'}

with open(r'client_ur1.yaml', 'w') as file:
    documents = yaml.dump(client_ur1, file)
with open(r'client_ur2.yaml', 'w') as file:
    documents = yaml.dump(client_ur2, file)
with open(r'client_ur3.yaml', 'w') as file:
    documents = yaml.dump(client_ur3, file)
with open(r'client_ur4.yaml', 'w') as file:
    documents = yaml.dump(client_ur4, file)

with open(r'client_abb1.yaml', 'w') as file:
    documents = yaml.dump(client_abb1, file)
with open(r'client_abb2.yaml', 'w') as file:
    documents = yaml.dump(client_abb2, file)
with open(r'client_abb3.yaml', 'w') as file:
    documents = yaml.dump(client_abb3, file)
with open(r'client_abb4.yaml', 'w') as file:
    documents = yaml.dump(client_abb4, file)


with open(r'client_ur.yaml') as file:
    documents = yaml.load(file, Loader=yaml.FullLoader)
    print(documents)
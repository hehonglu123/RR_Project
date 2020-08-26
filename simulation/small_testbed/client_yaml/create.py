import yaml

client_rp260 = {"robot_name":"rp260","height_offset":0.0,"destination":[0.4,0],"obj_namelists":["cube150","cube151","cube152","cube153","cube154","cube155","cube156","cube157"],"home":[0.3,0.0,0.3],"pick_height":0.2,"place_height":0.13,"url":'rr+tcp://localhost:23333?service=robot'}

with open(r'client_rp260.yaml', 'w') as file:
    documents = yaml.dump(client_rp260, file)


with open(r'client_rp260.yaml') as file:
    documents = yaml.load(file, Loader=yaml.FullLoader)
    print(documents)

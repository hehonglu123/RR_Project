from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose
import math
import time
import rospy
import traceback

model_poses={}
def insert_model(model_name,pose,num=0):
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    f = open('urdf/Meshes/'+model_name+'/model.sdf','r')
    sdff = f.read()
    try:
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(model_name+str(num), sdff, "robotos_name_space", pose, "world")
    except:
        traceback.print_exc()
    return model_name+str(num)
def pose_publish(model_name):
    pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = model_name
    rate = rospy.Rate(30)
    angle = 0
    radius = 2
    vel = 0.3
    now = time.time()
    while True:
        pose_msg.pose.position.x = radius * math.cos(angle * math.pi / 180)
        pose_msg.pose.position.y = radius * math.sin(angle * math.pi / 180)
        angle = angle + (vel / radius) * 180 / (math.pi * 30)
        if(angle == 360):
            angle = 0

        pose_msg.pose.position.z = 0
        pose_pub.publish(pose_msg)
        rate.sleep()
        print(model_poses[model_name])
        if (time.time()-now)>10:
            break

def callback(data):
    global model_poses

    # print(data)
    model_poses={data.name[i]:data.pose[i] for i in range(len(data.name))}

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        remove_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        remove_model_proxy(model_name)
    except:
        traceback.print_exc()

def main():
    rospy.init_node('model_change', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    initial_pose = Pose()
    initial_pose.position.x = 1
    initial_pose.position.y = 1
    initial_pose.position.z = 1

    model_name=insert_model("soap_bar",initial_pose)
    time.sleep(1)
    pose_publish(model_name)
    delete_model(model_name)
    # rospy.spin()

if __name__ == '__main__':
    main()
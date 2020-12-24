import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import traceback,os

RRC.RegisterStdRobDefServiceTypes(RRN)


fname = r"my_robot_1-2020-12-24T21-33-51.robtap"


with open(fname,"rb") as f:
    reader = RR.TapFileReader(f)
    while True:
        m = reader.ReadNextMessage()
        if m is None:
            break
        entry = m.entries[0]
        print (entry.MemberName)
        if (entry.MemberName == "robot_state"):
            for i in range(len(entry.elements)):
                el = entry.elements[i]
                print(el.ElementName)
                if el.ElementName == "packet":
                    val = reader.UnpackMessageElement(el)
                    print(val.data.joint_position_command)
                # if el.ElementName == "packettime":
                #     val = reader.UnpackMessageElement(el)
                #     print(val.data)
        
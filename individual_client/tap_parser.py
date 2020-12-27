import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
import traceback, os, json

RRC.RegisterStdRobDefServiceTypes(RRN)


fname = r"my_robot_1-2020-12-24T21-33-51.robtap"

out=[]


def parse_structure(val):
    temp_js={}
    return temp_js



with open(fname,"rb") as f:
    reader = RR.TapFileReader(f)
    while True:
        m = reader.ReadNextMessage()
        if m is None:
            break
        entry = m.entries[0]
        # print (entry.MemberName)
        temp={entry.MemberName:[]}

        for i in range(len(entry.elements)):
            el = entry.elements[i]
            temp_el={el.ElementName:None}
            if el.ElementName=='packettime':
                continue

            val = reader.UnpackMessageElement(el)
            if el.ElementType == 101:
                temp_el[el.ElementName]=parse_structure(val)
            else:
                if isinstance(val.data,np.ndarray):
                    temp_el[el.ElementName]=val.data.tolist()
                else:
                    temp_el[el.ElementName]=val.data
                
            temp[entry.MemberName].append(temp_el)
        out.append(temp)

fname=fname[:-6]+'json'

with open(fname,'w') as f:
    json.dump(out,f,indent=2)
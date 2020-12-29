import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteur.RobotRaconteurPythonUtil import SplitQualifiedName
import numpy as np
import pandas as pd
import traceback, os, json

RRC.RegisterStdRobDefServiceTypes(RRN)


fname = r"my_robot_1-2020-12-24T21-33-51.robtap"

out=[]
def cvt_list(l):
    temp_list=[]
    for i in range(len(l)):
        if isinstance(l[i],tuple) or isinstance(l[i],list) or isinstance(l[i],np.ndarray) or isinstance(l[i],np.void):
            temp_list.extend(cvt_list(l[i]))
        else:
            temp_list.append(l[i])
    #json not accepting uint8, np.ndarray, np.void
    return list(map(float,temp_list)) 

def parse_structure(val):
    names=dir(val)
    temp_js={}
    for name in names:

        #skip internal ones
        if '__' in name:
            continue
        #get structure data
        data=getattr(val, name)

        isstructure=False
        isstructure2=False
        #dump data to dict
        if isinstance(data,np.ndarray):
            temp_js[name]=cvt_list(data)
            continue

        elif isinstance(data,list):
            if data:
                try:
                    json.dumps({'temp':data[0]})
                except TypeError:
                    isstructure2=True
                temp_ls_js={}
                for i in range(len(data)):
                    if isstructure2:
                        temp_ls_js[name+'_'+str(i)]=parse_structure(data[i])
                    else:
                        if isinstance(data[i],np.ndarray):
                            temp_ls_js[name+'_'+str(i)]=cvt_list(data[i])
                        else:
                            temp_ls_js[name+'_'+str(i)]=data[i]
                temp_js[name]=temp_ls_js
            else:
                temp_js[name]=None
            continue

        else:
            try:
                json.dumps({'temp':data})
            except TypeError:
                isstructure=True

        if isstructure:
            temp_js[name]=parse_structure(data)
        else:
            temp_js[name]=data


        
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
                #packettime not readable
                continue

            val = reader.UnpackMessageElement(el)
            if el.ElementType == 101:
                temp_el[el.ElementName]=parse_structure(val.data)
            else:
                if isinstance(val.data,np.ndarray):
                    temp_el[el.ElementName]=val.data.tolist()
                else:
                    temp_el[el.ElementName]=val.data
                
            temp[entry.MemberName].append(temp_el)
        out.append(temp)


# def check_element(d):
#     if isinstance(d,list):
#         for i in range(len(d)):
#             check_element(d[i])
#     elif isinstance(d,dict):
#         for key, value in d.items():
#             check_element(value)
#     else:
#         if isinstance(d,np.ndarray):
#             print('here')
# for i in range(len(out)):
#     check_element(out[i])


fname=fname[:-6]+'json'
with open(fname,'w') as f:
    json.dump(out,f,indent=2)

#pandas json to csv
df = pd.read_json(fname)
df.to_csv("data.csv")    
#!/usr/bin/env python

import json

#Takes a tuple from the kinect data and makes it a json string. 
#Most of the things I wrote to process the data assumes it will be in this format. 
def jsonMaker(tup):
    dict = {
            "Joints" : [                   
                {
                    "name": joint[0],
                    "pos" : {
                        "x" : joint[1][0],
                        "y" : joint[1][1],
                        "z" : joint[1][2]
                    },
                    "rot" : {
                        "x" : joint[2][0],
                        "y" : joint[2][1],
                        "z" : joint[2][2],
                        "w" : joint[2][3]
                    }
                } for joint in tup[0]
            ]
    }

    return json.dumps(dict)    


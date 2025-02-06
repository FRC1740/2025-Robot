# quick script to visualize the elevator paths and poses
# takes data in the form of cli path points then constraints 
# python ploy.py "{(p1x,p1y) (p2x,p2y), (p3x,p3y)}" "{(p1x,p1y) (p2x,p2y)}, {second constraint box}"

import sys
import matplotlib

# total arguments
if len(sys.argv) != 3:
    raise Exception("Incorrect number of args passed")

bezierPoints = sys.argv[1]
constraintPoints = sys.argv[2]
print(bezierPoints)
print(constraintPoints)
#Name : Sanchita Gupta
#Roll No. : 2019BCS-053

#FABRIK THREE LINK MANIPULATOR

import numpy as np
import math
from operator import add, sub


def euclid_dist(lst1, lst2):
    dist = list(map(sub, lst2, lst1))
    dist = [i ** 2 for i in dist]
    return math.sqrt(sum(dist))


def scalarMul(val, list):
    dist = [val * i for i in list]
    return dist


inital_position = np.zeros(shape=(4, 2))

print("Enter initial position of manipulator and end effector")
for i in range(4):
    print("Enter {} coordinate: ".format(i + 1))
    inital_position[i][0], inital_position[i][1] = list(map(float, input().split()))

goal = list(map(float, input("Enter coordinates of goal: ").split()))


link_lengths = []

tolerance = 0.01
for i in range(inital_position.shape[0] - 1):
    link_lengths.append(euclid_dist(inital_position[i + 1], inital_position[i]))

diff = euclid_dist(goal, inital_position[0])
itr = 0
if (diff > sum(link_lengths)):
    print("not reachable")
    for i in range(inital_position.shape[0] - 1):
        # finding distance between target and join
        r = euclid_dist(goal, inital_position[i])
        l_avg = link_lengths[i] / r
        inital_position[i + 1] = list(map(add, ((1 - l_avg) * inital_position[i]), scalarMul(l_avg, goal)))
    print(inital_position[3])
else:
    # if the target is reachable
    b = np.copy(inital_position[0])
    curr_tolerance = euclid_dist(goal, inital_position[3])
    while (curr_tolerance > tolerance):
        inital_position[3] = goal
        # Traveling backward
        for i in range(inital_position.shape[0] - 2, -1, -1):
            r = euclid_dist(inital_position[i + 1], inital_position[i])
            l_avg = link_lengths[i] / r
            inital_position[i] = list(map(add, (1 - l_avg) * inital_position[i + 1], scalarMul(l_avg, inital_position[i])))

        # Traveling forward

        inital_position[0] = b
        for i in range(inital_position.shape[0] - 1):
            r = euclid_dist(inital_position[i + 1], inital_position[i])
            l_avg = link_lengths[i] / r
            inital_position[i + 1] = list(map(add, (1 - l_avg) * inital_position[i], scalarMul(l_avg, inital_position[i + 1])))

        curr_tolerance = euclid_dist(goal, inital_position[3])
        itr += 1
        print("iteration: ", itr)
        print(curr_tolerance)

print(inital_position[3])
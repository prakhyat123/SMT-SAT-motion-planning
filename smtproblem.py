from z3 import *
import ast

# Total Constrains
constraints=[]

#declaring cost
cost = Real('cost')

#Define makespan(length of longest trajectory) L+1
L=7

#Define number of robots
R=4

#Final Positions (0, 0), (0, 4), (4, 0) and (4, 4)
FP=[[0,0],[0,4],[4,0],[4,4]]

#Obstacles list (2, 0), (3, 0), (1, 2), (3, 2), (1, 4), and (2, 4)
obstacles=[[2,0],[3,0],[1,2],[3,2],[1,4],[2,4]]

# motion primivites displacement (N,S,E,W,NE,NW,SE,SW,stay)
MP=[[0,1],[0,-1],[1,0],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1],[0,0]]

# Creating list of state vectors for all the time instints
S=[]
for i in range(L):
    temp_state= [ [Int('%sx%s' % (j,i)),Int('%sy%s' % (j,i))] for j in range(R)]
    S.append(temp_state)

#State boundary conditions (robot needs to lie in the grid itself)
stateboundary_constraints=[]
for state in S:
    for robot in state:
        stateboundary_constraints.append(And(robot[0]>-1,robot[0]<5))
        stateboundary_constraints.append(And(robot[1]>-1,robot[1]<5))

constraints+=stateboundary_constraints

#captures the change at every time step
statechange=[]
for i in range(L-1):
    temp_state= [ [Int('%sxd%s' % (j,i)),Int('%syd%s' % (j,i))] for j in range(R)]
    statechange.append(temp_state)

statechange_constraints=[]
# State change constraints, state can be allowed to change only in few motion primitives
for state in statechange:
    for robot in state:
            #the change can be among eight action primitives
            statechange_constraints.append(Or(And(robot[0]==MP[0][0],robot[1]==MP[0][1]),
                                            And(robot[0]==MP[1][0],robot[1]==MP[1][1]),
                                            And(robot[0]==MP[2][0],robot[1]==MP[2][1]),
                                            And(robot[0]==MP[3][0],robot[1]==MP[3][1]),
                                            And(robot[0]==MP[4][0],robot[1]==MP[4][1]),
                                            And(robot[0]==MP[5][0],robot[1]==MP[5][1]),
                                            And(robot[0]==MP[6][0],robot[1]==MP[6][1]),
                                            And(robot[0]==MP[7][0],robot[1]==MP[7][1]),
                                            And(robot[0]==MP[8][0],robot[1]==MP[8][1])))

constraints+=statechange_constraints

#constraints for initial condition
initial_constraints=[ S[0][0][0]==0,S[0][0][1]==0,S[0][1][0]==0,S[0][1][1]==1,
                        S[0][2][0]==1,S[0][2][1]==0,S[0][3][0]==1,S[0][3][1]==1]

constraints+=initial_constraints

continuity_trajectory_constraints=[]

#Ensuring continuity of trajectories
for state_index,state in enumerate(S):
    if state_index==0:
        continue
    for robot_index,robot in enumerate(state):
        continuity_trajectory_constraints.append(robot[0]==S[state_index-1][robot_index][0]+statechange[state_index-1][robot_index][0])
        continuity_trajectory_constraints.append(robot[1]==S[state_index-1][robot_index][1]+statechange[state_index-1][robot_index][1])

constraints+=continuity_trajectory_constraints

#Obstacle avoidance constraint
obstacle_avoidance_constraints=[]
for state in S:
    for robot in state:
        for obs in obstacles:
            obstacle_avoidance_constraints.append(Not(And(robot[0]==obs[0],robot[1]==obs[1])))

constraints+=obstacle_avoidance_constraints

# collision avoidance constraint
collision_avoidance_constraints=[]
for state in S:
    for index1,robot1 in enumerate(state):
        for index2,robot2 in enumerate(state):
            if (index1==index2 or index1>index2):
                continue
            collision_avoidance_constraints.append(Not(And(robot1[0]==robot2[0],robot1[1]==robot2[1])))

constraints+=collision_avoidance_constraints

# head-on horizontal collision avoidance constraint
headOn_horizontalcollision_avoidance_constraints=[]
for state_index,state in enumerate(S[:-1]):
    for index1,robot1 in enumerate(state):
        for index2,robot2 in enumerate(state):
            if (index1==index2):
                continue
            headOn_horizontalcollision_avoidance_constraints.append(Not(And(robot2[0]==robot1[0]+1,S[state_index+1][index1][0]==S[state_index+1][index2][0]+1)))

constraints+=headOn_horizontalcollision_avoidance_constraints

# head-on vertical collision avoidance constraint
headOn_verticalcollision_avoidance_constraints=[]
for state_index,state in enumerate(S[:-1]):
    for index1,robot1 in enumerate(state):
        for index2,robot2 in enumerate(state):
            if (index1==index2):
                continue
            headOn_verticalcollision_avoidance_constraints.append(Not(And(robot2[1]==robot1[1]+1,S[state_index+1][index1][1]==S[state_index+1][index2][1]+1)))

constraints+=headOn_verticalcollision_avoidance_constraints

#Goal conditions every final position needs to be occupied by a robot
final_goal_constraints=[]
for state in S[-1:]:
    for robot in state:              
        final_goal_constraints.append(Or(And(robot[0]==FP[0][0],robot[1]==FP[0][1]),
                                            And(robot[0]==FP[1][0],robot[1]==FP[1][1]),
                                            And(robot[0]==FP[2][0],robot[1]==FP[2][1]),
                                            And(robot[0]==FP[3][0],robot[1]==FP[3][1])))
constraints+=final_goal_constraints

#Linear temporal properties: every non obstacle point needs to be visited atleast once
temporal_properties_constraints=[]
for i in range(5):
    for j in range(5):
        if ([i,j] in obstacles) or ([i,j] in FP):
            continue
        temporal_poisiton=[]
        for state in S:
            for robot in state:
                temporal_poisiton.append(And(robot[0]==i,robot[1]==j))
        temporal_properties_constraints.append(Or(temporal_poisiton))

constraints+=temporal_properties_constraints

#Calculate cost as sum of state change
cost_all=[]
for state in statechange:
    cost_all.append(Sum([If(Or(And(robot[0]==1,robot[1]==1),And(robot[0]==-1,robot[1]==-1)
                ,And(robot[0]==1,robot[1]==-1),And(robot[0]==-1,robot[1]==1)),1.5,
                If(And(robot[0]==0,robot[1]==0),0.5,1)) for robot in state]))

cost=Sum(cost_all)

print("*****Total constraints start*****")
print(constraints)
print("*****Total constraints end*******")

solver = Solver()
solver.add(constraints)
solver.add(cost>0)
solver.add(cost<=22)
if solver.check() == sat:
    m=solver.model()
    r = [ [ str([m.evaluate(S[i][j][0]),m.evaluate(S[i][j][1])]) for j in range(R) ]for i in range(L) ]
    change = [ [ str([m.evaluate(statechange[i][j][0]),m.evaluate(statechange[i][j][1])]) for j in range(R) ]for i in range(L-1) ]
    print_matrix(r)
    print_matrix(change)
    print("Total cosst for the run is "+ str(m.evaluate(cost)))
else:
    print "failed to solve the model"


from z3 import *

# Total Constrains
constraints=[]

#Define makespan(length of longest trajectory) L+1
L=10

#Define number of robots
R=4

# motion primivites displacement (N,S,E,W,NE,NW,SE,SW,stay)
MP=[[0,1],[0,-1],[1,0],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1],[0,0]]
motionprimitive_constraints=[]
mp0x,mp1x,mp2x,mp3x,mp4x,mp5x,mp6x,mp7x,mp8x=Int('mp0x'),Int('mp1x'),Int('mp2x'),Int('mp3x'),Int('mp4x'),Int('mp5x'),Int('mp6x'),Int('mp7x'),Int('mp8x')
mp0y,mp1y,mp2y,mp3y,mp4y,mp5y,mp6y,mp7y,mp8y=Int('mp0y'),Int('mp1y'),Int('mp2y'),Int('mp3y'),Int('mp4y'),Int('mp5y'),Int('mp6y'),Int('mp7y'),Int('mp8y')
motionprimitive_constraints.append(mp0x==0)
motionprimitive_constraints.append(mp0y==1)
motionprimitive_constraints.append(mp1x==0)
motionprimitive_constraints.append(mp1y==-1)
motionprimitive_constraints.append(mp2x==1)
motionprimitive_constraints.append(mp2y==0)
motionprimitive_constraints.append(mp3x==-1)
motionprimitive_constraints.append(mp3y==0)
motionprimitive_constraints.append(mp4x==1)
motionprimitive_constraints.append(mp4y==1)
motionprimitive_constraints.append(mp5x==-1)
motionprimitive_constraints.append(mp5y==1)
motionprimitive_constraints.append(mp6x==1)
motionprimitive_constraints.append(mp6y==-1)
motionprimitive_constraints.append(mp7x==-1)
motionprimitive_constraints.append(mp7y==-1)
motionprimitive_constraints.append(mp8x==0)
motionprimitive_constraints.append(mp8y==0)

constraints+=motionprimitive_constraints

def motionprimitive(variable):
    if variable==0:
        return mp0x,mp0y
    elif variable==1:
        return mp1x,mp1y
    elif variable==2:
        return mp2x,mp2y
    elif variable==3:
        return mp3x,mp3y
    elif variable==4:
        return mp4x,mp4y
    elif variable==5:
        return mp5x,mp5y
    elif variable==6:
        return mp6x,mp6y
    elif variable==7:
        return mp7x,mp7y
    elif variable==8:
        return mp8x,mp8y
    else:
        return "error"


# Creating list of state vectors for all the time instints
S=[]
for i in range(L):
    temp_state= [ [Int('%sx%s' % (j,i)),Int('%sy%s' % (j,i))] for j in range(R)]
    S.append(temp_state)

# Creating list of motion primitives at every time instant
M=[]
for i in range(L):
    temp_state= [ Int('%sm%s' % (j,i)) for j in range(R)]
    M.append(temp_state)
print(M)

#Obstacles logic


# Primitive Selection
primitive_bound_constraints=[]
for m in M:
    for r in m:
        #primitive motion lies between 0 and  8 inclusive
        primitive_bound_constraints.append(r<9)
        primitive_bound_constraints.append(r>-1) 

constraints+=primitive_bound_constraints

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
        continuity_trajectory_constraints.append(robot[0]==S[state_index-1][robot_index][0]+motionprimitive(M[state_index-1][robot_index])[0])
        continuity_trajectory_constraints.append(robot[1]==S[state_index-1][robot_index][1]+motionprimitive(M[state_index-1][robot_index])[1])

constraints+=continuity_trajectory_constraints

#Obstacle avoidance
obstacle_avoidance_constraints=[]
for r in range(R):
    for t in range(L):
        

print("*****Total constraints start*****")
print(constraints)
print("*****Total constraints end*******")

solver = Solver()
solver.add(constraints)
if solver.check() == sat:
    m=solver.model()
    r = [ [ str([m.evaluate(S[i][j][0]),m.evaluate(S[i][j][1])]) for j in range(R) ]for i in range(L) ]
    motion = [ [ str(m.evaluate(M[i][j])) for j in range(R) ]for i in range(L) ]
    print_matrix(r)
    print_matrix(motion)
else:
    print "failed to solve the model"


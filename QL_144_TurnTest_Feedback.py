''' QL_TurnTest
09 07 20 Based on The awkwardly titled QLearning_ObstAvoid 
            2 Sonar states - 0 = no detection, 1 = Obstacle within 36cm (Robot diameter x 1.5).
            3 Motor states - 0 = -Turning Left, 1 = Stopped, 2 = Turning Right.
        4 x 2 sonar and 2 x 3 motor states = 2×2×2×2×3×3 = 144 states. 864 Actions in total.
        Reward will only come from sonar so the closer objects are the higher hte reward, No reward is gained from motor movement.
14 07 20 Added dictionary to collect info and matplotlib graph feedback on pause.
        Save dictionary for data retrieval when restatrting robot.
'''
# Import Stuff
import RPi.GPIO as GPIO
import time
import serial
import math
import random
import numpy as np
from numpy import save
from numpy import load
import matplotlib.pyplot as plt
import pickle
 

# Serial Setup
bot = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
bot.flush()

t = int(0) # Variable for timestep count

Q = load('Q_Table_TurnTest.npy')
states = load('States_List_TurnTest.npy')

tlog = open("QL_TurnTest_log.txt") # Open File for reading to retrieve data
tlog.readline()        #Read first line and do nothing with it
oldt = tlog.readline() # Read file and assign iteration data to variable
tlog.close() # Close file
print(oldt)
t = int(oldt) # convert numeric data into int


# Set up motors and GPIO pins
GPIO.setmode(GPIO.BCM) # Use Broadcom pin numbering
GPIO.setwarnings(False)
GPIO.setup(22, GPIO.OUT) # Left Motor Forward
GPIO.setup(23, GPIO.OUT) # Left Motor Backward
GPIO.setup(27, GPIO.OUT) # Right Motor Forward
GPIO.setup(18, GPIO.OUT) # Right Motor Backward
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Pause switch
leftFor = GPIO.PWM(22, 50)
leftBac = GPIO.PWM(23, 50)
rightFor = GPIO.PWM(27, 50)
rightBac = GPIO.PWM(18, 50)

# initiate Motors
leftDutyCycle, rightDutyCycle = 0, 0
leftFor.start(leftDutyCycle)
leftBac.start(0)
rightFor.start(rightDutyCycle)
rightBac.start(0)

button = 4 # Pause Switch GPIO 4
pause = 0  # Paused / Resume state

# Lets have a function to stop the motors
Stopped = False
def Stop():
    print("STOPPED")
    global leftDutyCycle, rightDutyCycle
    leftDutyCycle, rightDutyCycle = 0, 0
    leftFor.start(leftDutyCycle)
    leftBac.start(0)
    rightFor.start(rightDutyCycle)
    rightBac.start(0)

# For stats
STATS_EVERY = 100
rewards = []

if t > 0:
    # Load Stats Dictionary
    aggr_rewards = pickle.load(open("TurnTest_Stats.pkl", "rb"))
else:
    aggr_rewards = {'t': [], 'avg': [], 'max': [], 'min': [], 'eps': []}

def Pause(): # Pause routine, Uses sleep
    global pause
    if GPIO.input(button) == 1:
        time.sleep(.3)
        if pause == 0:
            pause = 1
            Stop()
            SaveData()
            plt.plot(aggr_rewards['t'], aggr_rewards['avg'], label="average rewards")
            plt.plot(aggr_rewards['t'], aggr_rewards['max'], label="max rewards")
            plt.plot(aggr_rewards['t'], aggr_rewards['min'], label="min rewards")
            plt.plot(aggr_rewards['t'], aggr_rewards['eps'], label="epsilon x 10")
            plt.xlabel('Iterations')
            plt.ylabel('Rewards')
            plt.title('Turn Test')
            plt.legend(loc=4)
            plt.show()
            print("Paused")
        elif pause == 1:
            pause = 0
            startT = t + 1
            print("Resumed")

def SaveData():
    global t, states, Q
    print("Time Steps :", t)
    #for i in range(0, len(states)): # print Q table in terminal
    #    print(i+1, states[i], Q[i])
    
    #save('Q_Table_TurnTest.npy', Q)
    np.save('Q_Tables/Q_Table_{t}.npy', Q)
    #save('States_List_TurnTest.npy', states)

    t = str(t) # convert back to string for writing
    tlog = open("QL_TurnTest_log.txt", "w+") # open file for writing
    tlog.write("Itteration Number :\n") # Lets write what the file is about
    tlog.write(t) # write new data to file              # insert variables after conversion to string
    tlog.close()  # Close file
    t = int(t)
    
    # Save Dictionary Data
    StatDict = open("TurnTest_Stats.pkl", "wb")
    pickle.dump(aggr_rewards, StatDict)
    StatDict.close
    
    print("Data Saved.")
    

dataList = [] # this is where we store the data from the arduino
noData = True # Boolean to let us know if we've received any info from the Arduino

def Serial():  # Communicate with arduino to read encoders, bumpers and sonars
    #print("Serial")
    try:
        global noData
        bot.write(b"Send\n")
        data = bot.readline().decode('utf-8').rstrip()
        if len(data) < 1:
            print("No data ", data)
            noData = True
        else:
            global dataList
            dataList = data.split(",") # split at comma and make a list
            dataList = list(map(int, dataList)) # Convert string to ints
            noData = False
            #print("DATA", dataList)
            # 0 = Left, 1 = Front and 2 = Right Bumper,
            # 3 = Left Sonar No1, 4 = Left Secondary Sonar No2, 5 = Front Left Secondary Sonar No 3, 6 = Front Left Sonar No 4
            # 7 = Front Right Sonar No5, 8 = Front Right Secondary Sonar No6, 9= Right Secondary Sonar No7, 10 = Right Sonar No 8.
            # 11 = left forward 12 = left back 13 = right forward 14 = right back encoders
    except:
        print('no Connection')

PrevLeftFor = 0
PrevLeftBak = 0
PrevRightFor = 0
PrevRightBak = 0
long_Distance = 36 # threshold in cm over which obstacles are ignored

def SONAR(position): # Retrieve the state of individual Sonar
    global dataList, long_Distance
    '''
    # Four Sonar Script
    if position == 0:
            distance = dataList[3] # Get Left Sonar reading from list
        if position == 1:
            distance = dataList[4] # Get Left Front Sonar reading from list
        if position == 2:
            distance = dataList[5] # Get Right Sonar reading from list
        if position == 3:
            distance = dataList[6] # Get Right Front Sonar reading from list
    '''
    # Eight Sonar Script
    if position == 0:
        if dataList[4] > 0 and dataList[3] == 0: # if Left Secondary(4) registers an obstacle which Left(3) doesnt, Override.
            distance = dataList[4]
        elif dataList[3] > dataList [4] and dataList[4] > 0:
            distance = dataList[4]
        else:
            distance = dataList[3] # Else Get Left Sonar reading from list
    if position == 1:
        if dataList[5] > 0 and dataList[6] == 0: # if Front Left Secondary(5) registers an obstacle which Front Left(6) doesn't, override
            distance = dataList[5]
        elif dataList[6] > dataList [5] and dataList[5] > 0:
            distance = dataList[5]
        else:
            distance = dataList[6] # Else Get Front Left Sonar reading from list
    if position == 2:
        if dataList[8] > 0 and dataList[7] == 0: # if Front Right Secondary(8) registers an obstacle which Front Right(7) doesn't, override
            distance = dataList[8]
        elif dataList[7] > dataList [8] and dataList[8] > 0:
            distance = dataList[8]
        else:
            distance = dataList[7] # Get Front Right Sonar reading from list
    if position == 3:
        if dataList[9] > 0 and dataList[10] == 0: # if Right Secondary(9) registers an obstacle which Right(10) doesn't, Override
            distance = dataList[9]
        elif dataList[10] > dataList [9] and dataList[9] > 0:
            distance = dataList[9]
        else:
            distance = dataList[10] # else Get Right Sonar reading from list

    # Return a state based on distance to objects.
    if distance > long_Distance or distance < 1: # newPing returns distances over 100cm as 0
        State = 0
    if distance < long_Distance + 1 and distance > 0:
        State = 1
    return int(State)

def wheelSpeed(DutyCycle): # returns motor state - DutyCycle into number from 0, 1 or 2
    global leftDutyCycle, rightDutyCycle
    if DutyCycle == -50: # Left:
        State = 0
    if DutyCycle == 0: # Stopped:
        State = 1
    if DutyCycle == +50: # right:
        State = 2
    return int(State)

def getState(): # Returns state of the percieved world as a list i,e, distances from sonars and speed of wheels
    #print("getState")
    global  Q, states, leftDutyCycle, rightDutyCycle
    S1 = SONAR(0)  # read left sonar and get number from 1 to 4
    S2 = SONAR(1)  # read left front sonar
    S3 = SONAR(2)  # read right front sonar
    S4 = SONAR(3)   # read right sonar
    S5 = wheelSpeed(leftDutyCycle) # turn leftDutyCycle into number from 1 to 5
    S6 = wheelSpeed(rightDutyCycle) # turn rightDutyCycle into number from 1 to 5
    newState = [S1, S2, S3, S4, S5, S6]
    s = np.where((states == newState).all(axis=1))#replaces s = states.index(newState) with tolist command at the beginning
    #print ("New State = ", newState)
    #print ("State, s = ", s)
    return s                   # return list index to retieve data about state and action values (Q values)


#numActions = 6 # maintain speed or speed up or slow down, left wheel or right wheel by 33%
def getAction(s, epsilon): # pass the s index of Q table and epsilon, to get maxQ make epsilon 1
    global Q, leftDutyCycle, rightDutyCycle
    action = 0
    valMax = -1000000.0
    val = 0
    aMax = 0
    randVal = 0
    allowedActions = [-1, -1, -1, -1, -1, -1]
    randomActionFound = False

    # Find optimal Action without going outside the allowed states
    allowedActions[0] = 1 # maintain left wheel duty cycle
    val = Q[s, 0]
    if val > valMax:
        valMax = val
        aMax = 0
        
    allowedActions[1] = 1 # maintain right wheel duty cycle
    val = Q[s, 1]
    if val > valMax:
        valMax = val
        aMax = 1
        
    if leftDutyCycle + 50 < 51: # increase left wheel speed but not above 50
        allowedActions[2] = 1
        val = Q[s, 2]
        if val > valMax:
            valMax = val
            aMax = 2

    if rightDutyCycle + 50 < 51: # increase right wheel speed but not above 50
        allowedActions[3] = 1
        val = Q[s, 3]
        if val > valMax:
            valMax = val
            aMax = 3

    if leftDutyCycle - 50 > -51 and rightDutyCycle > -50: # reduce left wheel speed but not below -50
        allowedActions[4] = 1
        val = Q[s, 4]
        if val > valMax:
            valMax = val
            aMax = 4

    if rightDutyCycle - 50 > -51 and leftDutyCycle > -50: # reduce right wheel speed but not below -50
        allowedActions[5] = 1
        val = Q[s, 5]
        if val > valMax:
            valmax = val
            aMax = 5
            
    #Epsilon Greedy - if epsilon is below 1 there is a chance of a random allowed action being chosen (exploration)
    randVal = random.randrange(1,100)
    if randVal < (1-epsilon)*100:
        action = aMax
    else:
        while randomActionFound != True:
            action = random.randrange(0,6)
            if allowedActions[action] == 1:
                randomActionFound = True
                print("Random Action = ", action, ", Random Value = ", randVal, ", Epsilon = ", epsilon)
    #print("Allowed Actions ", allowedActions)
    return(action)

def Act(action):
    global leftDutyCycle, rightDutyCycle
    if action == 0: # maintain left speed
        pass
    if action == 1: # maintain right speed
        pass
    if action == 2: # speed up left motor
        leftDutyCycle += 50
    if action == 3: # speed up right motor
        rightDutyCycle += 50
    if action == 4: # slow down left motor
        leftDutyCycle -= 50
    if action == 5: # slow down right motor
        rightDutyCycle -= 50
    # Apply changes to duty cycle
    if leftDutyCycle < 0: 
        leftFor.ChangeDutyCycle(0)
        leftBac.ChangeDutyCycle(-leftDutyCycle)
    else:
        leftFor.ChangeDutyCycle(leftDutyCycle)
        leftBac.ChangeDutyCycle(0)
    if rightDutyCycle < 0:
        rightFor.ChangeDutyCycle(0)
        rightBac.ChangeDutyCycle(-rightDutyCycle)
    else:
        rightFor.ChangeDutyCycle(rightDutyCycle)
        rightBac.ChangeDutyCycle(0)


def mapp(x, in_min, in_max, out_min, out_max):
    # Function to map reward values ie - r = round(mapp(a, 0, 75, 0, 10), 4)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def getReward():
    global step, dataList, long_Distance #, Q, s, a
    r = 0
    '''
    r = Lcount + Rcount
    if r > 0:
            r = r/(step / 0.075) # ensure rewards are proportional to timestep. i.e. if the computer lags divide the reward by the overrun
    #print("Count Reward = ", r)
    r = round(mapp(r, 0, 160, -10, 0), 4)
    #print("r = ", r)
    '''
    minDistance = long_Distance
    '''
    # Four Sonar Script
    for i in range(3,7):
        if 0 < dataList[i] < minDistance:
            minDistance = dataList[i]
    '''
    # Eight Sonar Script
    for i in range(3,11):
        if 0 < dataList[i] < minDistance:
            minDistance = dataList[i]
    
    #print("least Distance = ", minDistance)
    minDistance = round(mapp(minDistance, 0, long_Distance, -long_Distance, 0), 4) # previously -10, 0
    #print("Distance Reward = ", minDistance)
    
    r = r + minDistance
    print ("Reward = ", r)
    r = round(r, 2)
    return (r)

# Parameters for getAction()
epsilon = 0.3   # epsilon is the probability of choosing an action randomly. The higher, the greater the chance.
                # 1-epsilon is the probability of choosing the optimal action
EPSILON_DECAY = 0.00003
epsilon = epsilon - EPSILON_DECAY * t # reduces to 0 over 10,000 steps
if epsilon < 0:
    epsilon = 0

#Computational parameters
alpha = 0.5     #"Forgetfulness" weight or learning rate.  The closer this is to 1 the more weight is given to recent samples.
gamma = 0.5    #look-ahead weight or discount factor 0 considers new rewards only, 1 looks for long term rewards
# Step/time parameters
lasttime = time.time() # Variable to store time for timesteps
step = 0
previousStep = 0

startT = t + 1 # Set starting itteration number based on info saved in log
bumpTime = startT
bumplessTime = 0
longestBumpless = 0

print("Start")
print("setting up Serial")
time.sleep(2)
print("Getting Data")
while noData == True:
    Serial()
print("DATA", dataList)
s = getState() # s = index of state in states list

while True:

    Pause()
    if pause == 1:
        pass
    else:
        if  time.time() > lasttime + 0.075: # 0.05 = 75 millis = 13.3 Hertz - 50 milliseconds = 20 Hertz
            lasttime = time.time()
            step = time.time() - previousStep
            previousStep = time.time()
            if t % 1000 == 0:
                #print("another 1000")
                SaveData()
            while noData == True:
                Stop()
                Serial()
            Serial()
            LB = dataList[0]
            FB = dataList[1]
            RB = dataList[2]
            '''
            # Four Sonar Script
            leftForw = dataList[7]
            rightForw = dataList[9]
            '''
            # Eight Sonar Script
            leftForw = dataList[11]
            leftBack = dataList[12]
            rightForw = dataList[13]
            rightBack = dataList[14]
            Lcount = (leftForw - PrevLeftFor) - (leftBack - PrevLeftBak)
            Rcount = (rightForw - PrevRightFor) - (rightBack - PrevRightBak)
            print(Lcount, " - ", Rcount)
            PrevLeftFor = leftForw
            PrevRightFor = rightForw
            PrevLeftBak = leftBack
            PrevRightBak = rightBack
            if LB == 0 or FB == 0 or RB == 0: # if bumpers are hit, Stop.
                if Stopped == False:
                    Stop()
                    Stopped = True
                    SaveData() #  Lets take this time to save states and values.
                    bumplessTime = t - bumpTime
                    bumpTime = t
                    if bumplessTime > longestBumpless:
                        longestBumpless = bumplessTime
                    print("Max free run = ", round(longestBumpless/13.333333/60, 2))
                    #Reverse from obstruction
                    leftFor.ChangeDutyCycle(0)
                    leftBac.ChangeDutyCycle(50)
                    rightFor.ChangeDutyCycle(0)
                    rightBac.ChangeDutyCycle(50)
                    time.sleep(1)
                    startT = t + 1
                    s = getState()
                    Stopped = False
                    
            else:
                Stopped = False
                t += 1
                if epsilon > 0:
                    epsilon -= EPSILON_DECAY # reduces to 0 over 10,000 steps
                print("Time step = ", t)
                #print("Time ", step)
                #print("DATA", dataList)
                # Data stuff

                if t > startT: # on the first time through the loop there will be no reward or previous states or actions
                    r = getReward() # get reward based on encoder count and distance from obstacles
                    print ("Old State = ", states[s])
                    newS = getState() # newS = index of state in states list
                    max_future_Action = getAction(newS, 0)  # find optimum action without risk of 'exploration' from Epsilon
                    max_future_Q = Q[newS, max_future_Action] # Get Q Value of optimum action.
                    #oldSTate = states[s]
                    currentQ = Q[s,a]
                    print ("currentQ = ", currentQ)
                    #newQ = (1 - alpha) * currentQ + alpha * (r + gamma * max_future_Q)  # got from https://pythonprogramming.net/q-learning-reinforcement-learning-python-tutorial/
                    newQ = currentQ + alpha * (r + gamma * max_future_Q - currentQ) # Bellman equation, the heart of the Reinforcement Learning program
                    newQ = np.round(newQ, 2) # round down floats for a tidier Q Table
                    Q[s, a] = newQ # CHanged from Q[s, a] = int(newQ) - not sure why it had data type specification 'int'
                    s = newS
                    print ("NewQ = ", newQ)
                    rewards.append(r) 
                    if not t % STATS_EVERY:
                        average_reward = sum(rewards[-STATS_EVERY:])/STATS_EVERY
                        aggr_rewards['t'].append(t)
                        aggr_rewards['avg'].append(average_reward)
                        aggr_rewards['max'].append(max(rewards[-STATS_EVERY:]))
                        aggr_rewards['min'].append(min(rewards[-STATS_EVERY:]))
                        aggr_rewards['eps'].append(epsilon * 10)
                a = getAction(s, epsilon)   # getAction will find an action based on the index s in the Q list and exploration will be based on epsilon
                print("Action = ", a)
                Act(a)





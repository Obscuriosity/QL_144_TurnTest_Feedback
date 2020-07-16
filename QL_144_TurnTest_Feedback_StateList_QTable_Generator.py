import numpy as np
from numpy import save

sonar_range = 2
motor_range = 3
startVal = 20.0

Q_table = np.array([[startVal, startVal, startVal, startVal, startVal, startVal]])
states = []
for i in range(0, sonar_range):
    for j in range(0, sonar_range):
        for k in range(0, sonar_range):
            for l in range(0, sonar_range):
                for m in range(0, motor_range):
                    for n in range(0, motor_range):
                        newState = (i, j, k, l, m, n)
                        states.append(newState)
print('states is ', len(states), " long.")
for o in range(0, len(states)-1):
    newQ = np.array ([startVal, startVal, startVal, startVal, startVal, startVal])  # create new action value list
    Q_table = np.vstack((Q_table, newQ))
print("Q Table is ", len(Q_table), " long.")

LOG = open("QL_TurnTest_log.txt", "w")
LOG.write("Itteration Number :\n") # Lets write what the file is about
LOG.write("0\n")
LOG.close()

save('Q_Table_TurnTest_0000.npy', Q_table) # Added zeros so as not to overwrite current file if one in use. Delete '_0000' to use
save('States_List_TurnTest.npy', states)

print("Q Table Saved\nState List Saved\nLog created")
'''
# Print off state and q tables
for p in range(0, len(states)):
   print(p+1, states[p], Q_table[p])
'''
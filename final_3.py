from itertools import permutations
import numpy
from unmanned_systems_ros2_pkg import Pathfinders
import time
starttime = time.time()
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8,
8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6,
7, 8, 9, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]

max_x = 15
max_y = 15
min_x = 0
min_y = 0
gs = 0.5
domain_x = numpy.arange(min_x,max_x+gs,gs)
domain_y = numpy.arange(min_y,max_y + gs,gs)
domain_gs = numpy.arange(-gs,gs + gs,gs) 
start = (1,1)
perm_list = (start,(9,7), (1,9), (4,4), (9,4), (6,14), (3,11), (14,1), (1,14), (14,14), (7,10))
total_cost = list()
super_total_cost = list()
paths_x = list()
paths_y = list()
perms = list()
perm = list(permutations([start,(9,7), (1,9), (4,4), (9,4), (6,14), (3,11), (14,1), (1,14), (14,14), (7,10)]))
point_costs = dict()
point_paths_x = dict()
point_paths_y = dict()
#compute distance
startime = Pathfinders.AStar(min_x,min_y,max_x,max_y,gs,domain_x,domain_y,domain_gs)

for f in perm_list:
    for e in perm_list:
        if e!= f:
          
           point_paths_x[f[0],f[1],e[0],e[1]] , point_paths_y[f[0],f[1],e[0],e[1]], point_costs[f[0],f[1],e[0],e[1]]=  startime.main(e[0],e[1],f[0],f[1],Obstacle_x,Obstacle_y,)
for i in perm:
    if i[0] == start:
        perms.append(i)
print("number of combinations is", len(list(perms)))
super_total_cost.append(1000)
k = 0
for path in perms:
    costs = list()
    x_things = list()
    y_things = list()
    i = 0
    while i < len(path) -1:
        x_value,y_value,cost = point_paths_x[path[i][0],path[i][1],path[i+1][0],path[i+1][1]], point_paths_y[path[i][0],path[i][1],path[i+1][0],path[i+1][1]], point_costs[path[i][0],path[i][1],path[i+1][0],path[i+1][1]]
        costs.append(cost)
        x_things.append([x_value[::-1]])
        y_things.append([y_value[::-1]])
        i = i + 1
    if sum(costs) > min(super_total_cost):
        
        continue
    total_cost.append(costs)
    super_total_cost.append(sum(costs))
    paths_x.append(x_things)
    paths_y.append(y_things)
    k = k + 1
chosen_one = super_total_cost.index(min(super_total_cost))
chosen_path_x = paths_x[chosen_one]
chosen_path_y = paths_y[chosen_one]
chosen_perm = perms[chosen_one]
chosen_costs = total_cost[chosen_one]
chosen_super_cost = super_total_cost[chosen_one]
print('chosen path is',chosen_perm ,'with a total cost of ',chosen_super_cost)
print('x = ', chosen_path_x)
print('y = ', chosen_path_y)
end = time.time()
print(end - starttime)
#print('some other garbage, inferior paths are:\n\n ', perms[1] ,'and \n' , perms[43],'\nwith corresponding garbage, inferior costs of:\n\n', super_total_cost[1], 'and\n' ,super_total_cost[43])
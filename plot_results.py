from cProfile import label
import matplotlib.pyplot as plt


# #Random Search Plot 
# x = []
# y = []
# i=0

# with open('/home/user/projects/shadow_robot/base/optimisationDTparams/_random_results_with_range.txt', 'r') as f: # __random_results
#     for line in f:
#         values = line.split(',')
#         if i > 100:
#             x.append(int(values[0]))
#             y.append(float(values[4])) #Change to 4 i order to get best error 
#         i+=1
        

# plt.figure(figsize=(17, 7))
# plt.scatter(x, y, s=17)
# plt.xlabel('Iteration', fontsize=28)
# plt.ylabel('Mean Squared Error', fontsize=28)
# plt.title('Random Search', fontsize=28)
# plt.xticks(fontsize=24)
# plt.yticks(fontsize=24)
# plt.ylim(0.0055, 0.010)
# plt.show()




#Particle Swarm Plot (Iteration error)
# with open("/home/user/projects/shadow_robot/base/optimisationDTparams/__particle_swarm_results.txt", "r") as f:
#     lines = f.readlines()[1:] # Ignore first line

# data_by_num = {}
# for line in lines:
#     values = line.strip().split(", ")
#     num = int(values[1])
#     if num not in data_by_num:
#         data_by_num[num] = []
#     data_by_num[num].append((float(values[0]), float(values[4]))) #4 -> Iteration error // 5 -> Global error

# fig, ax = plt.subplots(figsize=(17,7))
# for i, (num, data) in enumerate(data_by_num.items()):
#     xs, ys = zip(*data)
#     ax.scatter(xs, ys, label=f"Particle {num+1}", s=17)

# ax.set_xlabel("Iterarion", fontsize=28)
# ax.set_ylabel("Mean Squared Error", fontsize=28)
# ax.set_title("Particle Swarm", fontsize=28)
# plt.xticks(fontsize=24)
# plt.yticks(fontsize=24)
# ax.legend()
# plt.show()



# #Particle Swarm Plot (Global error)
# x = []
# y = []
# i=0

# with open("/home/user/projects/shadow_robot/base/optimisationDTparams/_with_range_particle_swarm_results.txt", "r") as f: #__particle_swarm_results       ->          #_with_range_particle_swarm_results
#     lines = f.readlines()[1:] # Ignore first line

# for line in lines:
#     values = line.split(', ')
#     if i>100:
#         x.append(int(str(values[0]) + str(values[1])))
#         y.append(float(values[5]))
#     i+=1

# # Create the graph
# plt.figure(figsize=(17, 7))
# plt.scatter(x, y, s=17)
# plt.xlabel('Iteration', fontsize=28)
# plt.ylabel('Mean Squared Error', fontsize=28)
# plt.title('Particle Swarm', fontsize=28)
# plt.xticks(fontsize=24)
# plt.yticks(fontsize=24)
# plt.ylim(0.0055, 0.010)
# plt.show()





# #Bayesian Search Plot (Iteration error)
# x = []
# y = []

# with open('/home/user/projects/shadow_robot/base/optimisationDTparams/__bayesian_search_results.txt', 'r') as f:
#     lines = f.readlines()
#     for i, line in enumerate(lines):
#         if i < len(lines) - 1:  # Check if this is not the last line
#             values = line.split(',')
#             x.append(int(values[0]))
#             y.append(float(values[2]))

# plt.figure(figsize=(17, 7))
# plt.scatter(x, y, s=17)
# plt.xlabel('Iteration', fontsize=28)
# plt.ylabel('Mean Squared Error', fontsize=28)
# plt.title('Bayesian Search', fontsize=28)
# plt.xticks(fontsize=24)
# plt.yticks(fontsize=24)
# plt.show()





# #Bayesian Search Plot (global error)
x = []
y = []
lowest_y = float('inf')  # Initialize lowest_y with positive infinity

with open('/home/user/projects/shadow_robot/base/optimisationDTparams/bayesian_search_results.txt', 'r') as f: #__bayesian_search_results & _bayesian_search_results /  _thumb_bayesian_search_results ->For thumb
    lines = f.readlines()
    for i, line in enumerate(lines):
       if i < len(lines) - 1:  # Check if this is not the last line
            values = line.split(',')
            x_value = int(values[0])
            y_value = float(values[2])
            #if y_value < lowest_y:
            lowest_y = y_value
            #if i>100:    
            x.append(x_value)
            y.append(lowest_y)

plt.figure(figsize=(17, 7))
plt.scatter(x, y, s=17)
plt.xlabel('Iteration', fontsize=28)
plt.ylabel('Mean Squared Error', fontsize=28)
plt.title('Bayesian Search', fontsize=28)
plt.xticks(fontsize=24)
plt.yticks(fontsize=24)
#plt.ylim(0.00005, 0.0007)
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import from_levels_and_colors
from math import pi

# Obstacle : Circle Center = (0,2), Radius = 1
# obstacles = [[0,2,1]]
# obstacles = [[1.75, 0.75, 0.6], [0, -1, 0.7], [-2, 1, 0.3]]
obstacles = [[1.75, 0.75, 0.6], [0, -1, 0.7], [-2, 1, 0.3], [0, 2, 0.8]]
grid_range = 100

def main():
    # length_list = [1, 1]
    length_list = [1, 1.5]
    number_of_link = 2
    arm = RobotArm(number_of_link,length_list)
    # start_node = [10,10]  # node = [theta1, theta2]
    # goal_node = [50,70]
    start_node = [95, 10]
    goal_node = [50, 95]
    # print(arm.len_link_list)
    # print(arm.point)
    # print(arm.joint_rad)
    # print(arm.num_link)

    # print(arm.point)
    # print(arm.joint_rad)

    # theta_list = [pi/6, pi/3]

    # arm.update_joint_rad(theta_list)
    # arm.update_point()

    # print(arm.point)
    # print(arm.joint_rad)
    start_theta = [start_node[0]*2*pi/grid_range, start_node[1]*2*pi/grid_range]
    goal_theta = [goal_node[0]*2*pi/grid_range, goal_node[1]*2*pi/grid_range]
    arm.update_joint_rad(start_theta)
    arm.update_point()
    start_point = [arm.point[arm.num_link][0], arm.point[arm.num_link][1]]

    arm.update_joint_rad(goal_theta)
    arm.update_point()
    goal_point = [arm.point[arm.num_link][0], arm.point[arm.num_link][1]]



    """
    joint space[i][j] : theta1 = 2*pi/grid_range*i
                        theta2 = 2*pi/grid_range*j

    Draw_Joint_Space : x-axis = theta2
                       y-axis = theta1
    """

    fig ,(ax1, ax2) = plt.subplots(1,2)
    joint_space = Set_Joint_Space(arm, obstacles)
    # Draw Configuration space
    Draw_Space(ax1, ax2, arm, obstacles, joint_space,start_point, goal_point)


    # Find Path
    route = A_Star(joint_space, start_node, goal_node)
    if(route == False):
        Draw_Space(ax1, ax2, arm, obstacles, joint_space,start_point, goal_point)
        plt.show()
        return 0


    Draw_Path(ax1, ax2, arm, obstacles, joint_space, route, start_point, goal_point)


    




    
class RobotArm(object):
    point = [(0,0)] #Origin Of RobotArm = (0,0)
    joint_rad = []
    def __init__(self, num_link, len_link_list):
        if(num_link != len(len_link_list)):
            print("num_link != len(len_link_list)")
            return False
        self.num_link = num_link
        self.len_link_list = len_link_list
        for i in range(num_link):
            self.joint_rad.append(0)
            self.point.append((self.point[i][0]+len_link_list[i],0))

    def update_joint_rad(self, theta_list):
        for i in range(self.num_link):
            self.joint_rad[i]=theta_list[i]
    
    def update_point(self):
        for i in range(self.num_link):
            sum_rad = 0
            for j in range(i+1):
                sum_rad += self.joint_rad[j]
            self.point[i+1] = (self.point[i][0]+self.len_link_list[i]*np.cos(sum_rad),
                               self.point[i][1]+self.len_link_list[i]*np.sin(sum_rad))
        

        
def Set_Joint_Space(robotarm, obstacle_list):
    grid_value = [[0 for _ in range(grid_range)] for _ in range(grid_range)]
    for i in range(grid_range):
        for j in range(grid_range):
            theta_list = [2*pi/grid_range*i, 2*pi/grid_range*j]
            robotarm.update_joint_rad(theta_list)
            robotarm.update_point()
            if(Collision(robotarm,obstacle_list)):
                grid_value[i][j] = 1
    
    return grid_value

def Collision(robotarm, obstacle_list):
    for i in range(robotarm.num_link):
        link_point1 = (robotarm.point[i][0], robotarm.point[i][1])
        link_point2 = (robotarm.point[i+1][0],robotarm.point[i+1][1])
        for obstacle in obstacle_list:
            dist1 = np.sqrt((link_point1[0]-obstacle[0])**2+(link_point1[1]-obstacle[1])**2)
            dist2 = np.sqrt((link_point2[0]-obstacle[0])**2+(link_point2[1]-obstacle[1])**2)
            vec_from_1_to_center = (obstacle[0]-link_point1[0],obstacle[1]-link_point1[1])
            vec_unit_link = ((link_point2[0]-link_point1[0])/robotarm.len_link_list[i],
                             (link_point2[1]-link_point1[1])/robotarm.len_link_list[i])
            if((dist1<=obstacle[2]+0.03) | (dist2<=obstacle[2]+0.03)):
                return True
            # if((vec_from_1_to_center[0]**2+vec_from_1_to_center[1]**2) - np.dot(vec_from_1_to_center,vec_unit_link)**2<0):
            #        print((vec_from_1_to_center[0]**2+vec_from_1_to_center[1]**2) - np.dot(vec_from_1_to_center,vec_unit_link)**2)
            #        print(vec_from_1_to_center)
            #        print(vec_unit_link)
            if((0<np.dot(vec_from_1_to_center,vec_unit_link)<robotarm.len_link_list[i])&
               (np.sqrt(abs((vec_from_1_to_center[0]**2+vec_from_1_to_center[1]**2) - np.dot(vec_from_1_to_center,vec_unit_link)**2))<=obstacle[2]+0.03)):
               return True

    return False

def Draw_Space(ax1, ax2, robotarm, obstacle_list, joint_space, start_point, goal_point):
    X,Y=np.meshgrid([i for i in range(grid_range+1)],[i for i in range(grid_range+1)])
    colors = ['white', 'black', 'orange', 'red']
    levels = [0,1,2,3,4]
    
    cmap, norm = from_levels_and_colors(levels,colors)
    ax1.pcolormesh(X, Y, joint_space, cmap=cmap, norm=norm)
    ax1.set_aspect('equal')
    x=[]
    y=[]

    for obstacle in obstacle_list:
        circle = plt.Circle((obstacle[0],obstacle[1]), obstacle[2], color = 'k')
        ax2.add_artist(circle)
    for i in range(robotarm.num_link+1):
        x.append(robotarm.point[i][0])
        y.append(robotarm.point[i][1])
    ax2.plot(x,y,'ro-')
    ax2.scatter(start_point[0],start_point[1])
    ax2.scatter(goal_point[0],goal_point[1])
    axis_range = 0.5
    for length in robotarm.len_link_list:
        axis_range += length
    ax2.set_aspect('equal')

    plt.xlim(-axis_range,axis_range)
    plt.ylim(-axis_range,axis_range)
    


def Draw_Path(ax1, ax2, robotarm, obstacle_list, joint_space, route, start_point, goal_point):
    plt.ion()
    start_node = route.pop()
    joint_space[start_node[0]][start_node[1]] = 3
    joint_space[route[0][0]][route[0][1]] = 3
    theta_list = [2*pi/grid_range*start_node[0], 2*pi/grid_range*start_node[1]]
    robotarm.update_joint_rad(theta_list)
    robotarm.update_point()


    while(len(route)):
        plt.cla()
        path = route.pop()
        joint_space[path[0]][path[1]] = 2
        theta_list = [2*pi/grid_range*path[0], 2*pi/grid_range*path[1]]
        robotarm.update_joint_rad(theta_list)
        robotarm.update_point()
        Draw_Space(ax1, ax2, robotarm, obstacle_list, joint_space,start_point, goal_point)
        plt.pause(0.01)




# def Draw_Configuration_Space(robotarm, obstacle_list):
#     # start_node = route.pop()
#     fig, ax = plt.subplots()
#     x=[]
#     y=[]
#     for obstacle in obstacle_list:
#         circle = plt.Circle((obstacle[0],obstacle[1]), obstacle[2], color = 'k')
#         ax.add_artist(circle)
#     for i in range(robotarm.num_link+1):
#         x.append(robotarm.point[i][0])
#         y.append(robotarm.point[i][1])
#     ax.plot(x,y,'ro-')
#     plt.xlim(-10,10)
#     plt.ylim(-10,10)
#     plt.show()
#     ax=plt.gca()
#     return ax
        


    

def A_Star(joint_space,start_node, goal_node):
    h_map = [[0 for _ in range(grid_range)] for _ in range(grid_range)]
    g_map = [[np.inf for _ in range(grid_range)] for _ in range(grid_range)]
    g_map[start_node[0]][start_node[1]] = 0
    f_map = [[np.inf for _ in range(grid_range)] for _ in range(grid_range)]
    open_list = [start_node]
    closed_list = []
    parent_map = [[0 for _ in range(grid_range)] for _ in range(grid_range)]


    for i in range(grid_range):
        for j in range( grid_range):
            if(joint_space[i][j] == 1):
                h_map[i][j] = np.inf
            else:
                h_theta1 = min(abs(goal_node[0]-i),abs(goal_node[0]-grid_range-i))
                h_theta2 = min(abs(goal_node[1]-j), abs(goal_node[1]-grid_range-j))
                h_map[i][j] = h_theta1 + h_theta2

    f_map[start_node[0]][start_node[1]] = h_map[start_node[0]][start_node[1]]

    while(goal_node not in closed_list):

        # pick f_min_node in open_list
        f_min_node_value = np.inf
        for node in open_list:
            if(f_map[node[0]][node[1]] < f_min_node_value):
                f_min_node = node
                f_min_node_value = f_map[node[0]][node[1]]
        if(f_min_node_value == np.inf):
            print("No route found!")
            return False


        closed_list.append(f_min_node)
        open_list.remove(f_min_node)



        # Calculate f,g of adjacent_node
        # Update f,g of already existing node in open_list

        for adjacent_node in Adjacent_List(f_min_node):
            if(adjacent_node not in closed_list):
                if(adjacent_node not in open_list):
                    open_list.append(adjacent_node)
                    parent_map[adjacent_node[0]][adjacent_node[1]] = f_min_node
                    g_map[adjacent_node[0]][adjacent_node[1]] = g_map[f_min_node[0]][f_min_node[1]]+1
                    f_map[adjacent_node[0]][adjacent_node[1]] = g_map[adjacent_node[0]][adjacent_node[1]]+h_map[adjacent_node[0]][adjacent_node[1]]
                else:
                    if(g_map[adjacent_node[0]][adjacent_node[1]]>(g_map[f_min_node[0]][f_min_node[1]]+1)):
                        parent_map[adjacent_node[0]][adjacent_node[1]] = f_min_node
                        g_map[adjacent_node[0]][adjacent_node[1]] = g_map[f_min_node[0]][f_min_node[1]]+1
                        f_map[adjacent_node[0]][adjacent_node[1]] = g_map[adjacent_node[0]][adjacent_node[1]]+h_map[adjacent_node[0]][adjacent_node[1]]


    
    print("Route Found!")
    route= [goal_node] 
    final_node = goal_node
    while(final_node != start_node):
        route.append(parent_map[final_node[0]][final_node[1]])
        final_node = parent_map[final_node[0]][final_node[1]]
    
    return route

        




def Adjacent_List(node):
    adjacent_list = []
    # up_neighbor
    if(node[0]>=1):
        adjacent_list.append([node[0]-1, node[1]])
    else:
        adjacent_list.append([grid_range-1, node[1]])
    
    # down_neighbor
    if(node[0]<=grid_range-2):
        adjacent_list.append([node[0]+1, node[1]])
    else:
        adjacent_list.append([0, node[1]])

    #left_neighbor
    if(node[1]>=1):
        adjacent_list.append([node[0],node[1]-1])
    else:
        adjacent_list.append([node[0], grid_range-1])
    
    # right_neighbor
    if(node[1]<=grid_range-2):
        adjacent_list.append([node[0], node[1]+1])
    else:
        adjacent_list.append([node[0], 0])

    return adjacent_list

        


# class Node(object):
#     def __init__(self, row, col):
#         self.row=row
#         self.col=col
#         self.f = np.inf
#         self.g = np.inf
#         self.h = np.inf






if __name__=='__main__':
    main()
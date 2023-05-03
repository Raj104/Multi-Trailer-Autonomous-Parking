import settings
import numpy as np
import pygame
import utils
import math
import environment


#Implementing A* algorithm to find the path from start to goal withouth colliding with obstacles for a truck with multiple trailers


#Class to define the node in the graph
class Node():
    def __init__(self, x, y, orientations):
        self.x = x
        self.y = y
        self.orientations = orientations
        self.parent = None
        self.cost_to_come = 0
        self.cost = 0


    # comparing costs of two nodes
    def __lt__(self, other):
        return self.cost < other.cost
    
    def __gt__(self, other):
        return self.cost > other.cost
    
    def __le__(self, other):
        return self.cost <= other.cost
    
    # To retrun that the two nodes are equal if they have the same int(x), int(y) and orientation is similar to within 5 degrees
    def __eq__(self, other):
        return int(self.x) == int(other.x) and int(self.y) == int(other.y) and np.all(np.abs(self.orientations - other.orientations) < 5)




#Class to define the A* algorithm

class A_star():
    def __init__(self, start, goal, environment, truck_trailer):
        self.start = start
        self.goal = goal
        self.environment = environment
        self.truck_trailer = truck_trailer

        self.open_list = []
        self.closed_list = []
        self.path = []
        self.path_found = False

        self.num_trailers = len(self.truck_trailer.trailer_rects)



    def simulate_forward(self, time, position, orientation, contorl_input):
        # position is a tuple of x and y coordinates
        # orientation is the orientation of the truck
        # control input is a tuple of steering angle and velocity
        # returns the new position and orientation of the truck

        # unpacking the position and orientation
        x = position[0]
        y = position[1]
        theta = orientation.copy()
        for i in range(len(theta)):
            theta[i] = math.radians(theta[i])
        collision_flag = False
        

        # unpacking the control input
        steering_angle = contorl_input[1]
        velocity = contorl_input[0]


        # calculating the new position and orientation
        time_steps = np.arange(0.0,time,settings.time_step)

        for _ in time_steps:

            x += velocity * math.cos(theta[0])*settings.time_step
            y += velocity * math.sin(theta[0])*settings.time_step
            #print("check3", x, y, theta[0], contorl_input)
            theta[0] += velocity * math.tan(math.radians(steering_angle))*settings.time_step / settings.car_length
            if self.num_trailers > 0:
                theta[1] = theta[1] + settings.time_step*velocity * math.sin((theta[0] - theta[1])) / settings.trailer_length

            if self.num_trailers > 1:
                for i in range(2,self.num_trailers+1):
                    # lambda function to calculate the product of cosines of all differences of successive angles till angle i, that is product of cos(theta[0] - theta[1]) * cos(theta[1] - theta[2]) * ... * cos(theta[i-2] - theta[i-1])
                    product_cos = lambda i: np.prod(np.cos((np.diff(theta[:i-1]))))
                    theta[i] = theta[i] + settings.time_step*velocity *product_cos* math.sin((theta[i-1] - theta[i])) / settings.trailer_length
        
            # checking for collision
            self.truck_trailer.move_truck((x,y),theta)
            if self.truck_trailer.check_collision(self.environment):
                collision_flag = True
                break
            # if x < 0 or x > settings.grid_width or y < 0 or y > settings.grid_height:
            #     collision_flag = True
            #     break

        for i in range(len(theta)):
            theta[i] = math.degrees(theta[i])

        return (x, y), theta, collision_flag
    

    def A_star_plan(self):
        init_node = Node(self.start[0], self.start[1], self.start[2])
        end_node = Node(self.goal[0], self.goal[1], self.goal[2])
        self.open_list.append(init_node)
        
        velocity_controls = np.arange(-5.0,5.1,5)
        steering_controls = np.arange(-40.0,40.0,4.0)
        control_inputs = utils.generate_controls_matrix(velocity_controls, steering_controls)
       

        while len(self.open_list) > 0:
            print("open list is not empty", len(self.open_list))
            env.screen.fill((255,255,255))
            env.draw_obstacles()
            
            # find the node and argument with the least cost
            min_dist = np.inf
            current_node = None
            current_index = None
            for node in self.open_list:
                if node.cost < min_dist:
                    min_dist = node.cost
                    current_node = node
                    current_index = self.open_list.index(node)

            # current_node = min(self.open_list)
            # current_index = self.open_list.index(current_node)
 


            print("current distance from goal", np.linalg.norm(np.array([current_node.x, current_node.y]) - np.array([end_node.x, end_node.y])))
            print("current node", current_node.x, current_node.y, current_node.orientations)

            self.truck_trailer.draw_truck(self.environment.screen, (current_node.x, current_node.y), current_node.orientations)
            pygame.display.flip()

            #delete the node from the open list and add it to the closed list
            del self.open_list[current_index]
            self.closed_list.append(current_node)

            # check if the current node is close to the goal node

            if np.linalg.norm(np.array([current_node.x, current_node.y]) - np.array([end_node.x, end_node.y])) < 5:
                self.path_found = True
                print("path found")
                break

            # generate the children of the current node
            for control_input in control_inputs:
                new_position, new_orientation, collision_flag = self.simulate_forward(settings.sim_time, (current_node.x, current_node.y), current_node.orientations, control_input)
                if not collision_flag:
                    new_node = Node(new_position[0], new_position[1], new_orientation)
                    new_node.parent = current_node
                    new_node.cost_to_come = current_node.cost_to_come + utils.cost_to_go(new_node, current_node)
                    new_node.cost = new_node.cost_to_come +  3*utils.cost_to_go(new_node, end_node)

                    if new_node not in self.closed_list and new_node not in self.open_list:
                        self.open_list.append(new_node)
                    # elif new_node in self.closed_list:
                    #     continue
                    # else:
                    #     index  = self.open_list.index(new_node)
                    #     if new_node.cost < self.open_list[index].cost:
                    #         self.open_list[index] = new_node
                            

        if self.path_found:
            self.path.append(end_node)
            parent = end_node.parent
            while parent is not None:
                self.path.append(parent)
                parent = parent.parent
            self.path.reverse()
            return self.path
        




if __name__ == "__main__":
    settings.init()
    env = environment.environment()
    truck_trailer = environment.truck_trailer(1)
    truck_trailer.draw_truck(env.screen, (0.0,0.0), (0.0,0.0))
    env.create_obstacles()
    env.draw_obstacles()
    pygame.display.update()
    start = (0.0,0.0,np.array([0.0,0.0]))
    goal = (20.0,20.0,np.array([0.0,0.0]))
    a_star = A_star(start, goal, env, truck_trailer)
    path = a_star.A_star_plan()
    print(path)
    for node in path:
        truck_trailer.draw_truck(env.screen, (node.x, node.y), node.orientations)
        env.draw_obstacles()
        pygame.display.flip()
        pygame.time.wait(100)
        env.screen.fill((255,255,255))
        env.draw_obstacles()
        pygame.display.update()
    pygame.quit()





    
        
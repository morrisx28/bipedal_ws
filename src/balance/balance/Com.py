import numpy as np
import math
import matplotlib.pyplot as plt
#robot parameter
# wheel:120g
# lower leg:22g  7.5/12
# upper leg:150

class Body:
    def __init__(self,hip ,knee):
        self.links = []
        self.joint_angles = []
        self.endpoint_list = []
        self.fig, self.ax = plt.subplots()
        self.body_com_x = 0.0
        self.body_com_y = 0.0 
        self.AddLink("l1", 0.7142, 0.02258, 0.0127/0.02258, 35.31 - 180 , [0,0])  # base link
        self.AddLink("l2", 0.1345, 0.118, 0.059/0.118 , hip , [0,0])  # 0.02278/0.118
        self.AddLink("l3", 0.1224, 0.107, 0.07791/0.107, knee , [0,0])
        self.plot_coordinate_axis()
        self.com , self.foot_point ,self.theta , self.length , self.mass, self.theta3 = self.calculate_global_com()
        self.ax.plot(self.com[0],self.com[1],'bo')
        plt.show()

    def AddLink(self, name, mass, length, scale, initial_angle_degrees , end_point):
        self.links.append({'name': name,'mass': mass,'length': length,'scale': scale})
        self.joint_angles.append(math.radians(initial_angle_degrees))

    def update_joint_angle(self,new_joint_angel):
        self.joint_angles = [math.radians(angle) for angle in new_joint_angel]

    def calculate_global_com(self):
        total_mass = 0.0

        for link, angle in zip(self.links, self.joint_angles):
            link_length = link['length']
            link_scale = link['scale']
            link_mass = link['mass']
            
            # 計算值心
            if link['name'] == "l1":
                link_com_x = link_length * link_scale * np.cos(angle)
                link_com_y = link_length * link_scale * np.sin(angle)
            
                link_end_x = link_length * np.cos(angle)
                link_end_y = link_length * np.sin(angle)

                self.body_com_x = link_com_x * link_mass 
                self.body_com_y = link_com_y * link_mass

                total_mass = total_mass +link_mass
                self.ax.plot(link_com_x,link_com_y,'ro')
                self.ax.plot(link_end_x,link_end_y,'go')
                self.endpoint_list.append(np.array([link_end_x, link_end_y]))
            else:
                link_com_x = link_end_x + (link_length * link_scale * np.cos(angle))
                link_com_y = link_end_y + (link_length * link_scale * np.sin(angle))

                link_end_x = link_length * np.cos(angle) + link_end_x
                link_end_y = link_length * np.sin(angle) + link_end_y

                self.body_com_x = 2*link_com_x * link_mass + self.body_com_x
                self.body_com_y = 2*link_com_y * link_mass + self.body_com_y

                total_mass = total_mass +2*link_mass
                self.ax.plot(link_com_x,link_com_y,'ro')
                self.ax.plot(link_end_x,link_end_y,'go')
                self.endpoint_list.append(np.array([link_end_x, link_end_y]))
                
        self.body_com_x = self.body_com_x/total_mass
        self.body_com_y = self.body_com_y/total_mass
        global_com = np.array([self.body_com_x, self.body_com_y])
        print(global_com)
        foot_point = np.array([link_end_x, link_end_y])
        theta = np.arctan(abs(self.body_com_x-link_end_x)/(self.body_com_y-link_end_y))
        length = distance_between_points(global_com,foot_point)
        print(length)

        l1 = self.endpoint_list[1] - foot_point
        l2 = global_com - foot_point
        theta3 = math.acos(np.dot(l1, l2)/ (np.linalg.norm(l1)*np.linalg.norm(l2)))
        print('theta3 =', math.degrees(theta3))
        

        return global_com , foot_point , theta , length , total_mass, theta3
    def plot_coordinate_axis(self):
            
            self.ax.spines['left'].set_position('zero')
            self.ax.spines['left'].set_color('gray')
            self.ax.spines['bottom'].set_position('zero')
            self.ax.spines['bottom'].set_color('gray')
            self.ax.spines['right'].set_color('none')
            self.ax.spines['top'].set_color('none')
            self.ax.xaxis.set_ticks_position('bottom')
            self.ax.yaxis.set_ticks_position('left')
            self.ax.set_aspect('equal', 'box')
            self.ax.set_xlim([-0.05, 0.15])
            self.ax.set_ylim([-0.5, 0.0])
            
            self.ax.grid(True)

            plt.xlabel('X')
            plt.ylabel('Y')

def distance_between_points(point1, point2):
    return np.linalg.norm(point2 - point1)    





# fig, ax = plot_coordinate_axis()

if __name__ == "__main__":
    hip = 72.42
    knee = 125
    hip = hip - 90
    knee = hip - knee
    two_wheel = Body(hip=hip,knee=knee)

    
    
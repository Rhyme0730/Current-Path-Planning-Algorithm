'''
Date: 2024/04/01
Note: This Code should show a dynamic planning, since the peusdo code exists move point action
Bugs: It will pause after several clicks, may need to fix this...

'''
import os
import sys
import math
import matplotlib.pyplot as plt
import env, plotting

class dStarLite:
    def __init__(self, s_start, s_goal) -> None:
        self.s_start, self.s_goal = s_start, s_goal
        self.Env = env.Env()
        self.Plot = plotting.Plotting(s_start, s_goal)

        self.u_set = self.Env.motions
        self.obs = self.Env.obs

        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")
        
        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.calculate_key(self.s_goal)
        self.visited = set()
        self.count = 0
        self.fig = plt.figure()

        self.path_line = None
        self.path_count = 0
    
    def calculate_key(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def get_neighbor(self, s):
        neighbor_list = set()
        for u in self.u_set:
            s_next = (s[0]+u[0], s[1]+u[1])
            if s_next not in self.obs:
                neighbor_list.add(s_next)
        return neighbor_list
    
    def update_vertex(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.U:
            self.U.pop(s)

        if self.g[s] != self.rhs[s]:
            self.U[s] = self.calculate_key(s)
    
    def top_key(self):
        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def compute_path(self):
        while True:
            s, v = self.top_key()
            if v >= self.calculate_key(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break

            k_old = v
            self.U.pop(s)
            self.visited.add(s)

            if k_old < self.calculate_key(s): # since km changed, so we need to change key in priority queue
                self.U[s] = self.calculate_key(s)

            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.update_vertex(x)
            
            else:
                self.g[s] = float("inf")
                self.update_vertex(s)
                for x in self.get_neighbor(s):
                    self.update_vertex(x)  

    def is_collision(self, s_begin, s_end):
        if s_begin in self.obs or s_end in self.obs:
            return True
        
        if s_begin[0] != s_end[0] and s_begin[1] != s_end[1]:
            if s_end[0] - s_begin[0] == s_begin[1] - s_end[1]:
                s1 = (min(s_begin[0], s_end[0]), min(s_begin[1], s_end[1]))
                s2 = (max(s_begin[0], s_end[0]), max(s_begin[1], s_end[1]))
            else:
                s1 = (min(s_begin[0], s_end[0]), max(s_begin[1], s_end[1]))
                s2 = (max(s_begin[0], s_end[0]), min(s_begin[1], s_end[1]))
            if s1 in self.obs or s2 in self.obs:
                return True 


    def h(self, s_begin, s_end):
        if self.is_collision(s_begin, s_end):
            return float("inf")
        
        return math.hypot(s_end[0] - s_begin[0], s_end[1] - s_begin[1])

    def plot_path(self, path):
        # px = [x[0] for x in path]
        # py = [x[1] for x in path]
        # plt.plot(px, py, linewidth=2)
        # plt.plot(self.s_start[0], self.s_start[1], "bs")
        # plt.plot(self.s_goal[0], self.s_goal[1], "gs")
        if self.path_line is not None:
            self.path_line.remove()  # Remove the previous line from the plot
            self.path_line = None  # Reset the path_line to None
            self.path_count += 1
        px = [x[0] for x in path]
        py = [x[1] for x in path]

        if self.path_count % 2 == 0:
            self.path_line, = plt.plot(px, py, linewidth=2, color='blue')  # Save the new line object
        else:
            self.path_line, = plt.plot(px, py, linewidth=2, color='red')  # Save the new line object
            
        plt.plot(self.s_start[0], self.s_start[1], "bs")  # Start marker
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")  # Goal marker

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])

    def extract_path(self):
        path = [self.s_start]
        s = self.s_start

        for k in range(100):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break
        
        return path

    def cost(self, s_begin, s_end):
        if self.is_collision(s_begin, s_end):
            return float("inf")
        
        return math.hypot(s_end[0] - s_begin[0], s_end[1] - s_begin[1])

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")        
        else:
            x, y = int(x), int(y)
            print(f'Change Position: x={x}, y={y}')

            s_curr = self.s_start
            s_last = self.s_start
            update_flag = True
            path = [self.s_start]

            while s_curr != self.s_goal:
                s_list = {}

                for s in self.get_neighbor(s_curr):
                    s_list[s] = self.g[s] + self.cost(s_curr, s)
                s_curr = min(s_list, key=s_list.get)
                path.append(s_curr)
                
                if update_flag:
                    self.km += self.h(s_last, s_curr)
                    s_last = s_curr
                    if (x, y) not in self.obs:
                        self.obs.add((x, y))
                        plt.plot(x, y, 'sk')
                        self.g[(x, y)] = float("inf")
                        self.rhs[(x, y)] = float("inf")
                    else:
                        self.obs.remove((x, y))
                        plt.plot(x, y, marker='s', color='white')
                        self.update_vertex((x, y))
                    for s in self.get_neighbor((x, y)):
                        self.update_vertex(s)
                    update_flag = False

                    self.count += 1
                    self.visited = set()
                    self.compute_path()
            
            # self.plot_visited(self.visited)
            self.plot_path(path)
            self.fig.canvas.draw_idle()
                    
    def run(self):
        self.Plot.plot_grid("D* Lite")
        self.compute_path()
        self.plot_path(self.extract_path())
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        plt.show()


s_start = (5, 5)
s_goal = (40, 25)

dstar_lite = dStarLite(s_start, s_goal)
dstar_lite.run()
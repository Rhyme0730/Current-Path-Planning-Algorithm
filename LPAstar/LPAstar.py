import os
import sys
import math
import matplotlib.pyplot as plt
import plotting, env

class LPAstar:
    def __init__(self, s_start, s_goal, heurisitic_type="Euclidean"):
        self.s_start, self.s_goal = s_start, s_goal
        self.heurisitic_type = heurisitic_type

        self.Env = env.Env()
        self.Plot = plotting.Plotting(self.s_start, self.s_goal)

        self.u_set = self.Env.motions # Neighbors index
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        # Initialize g, rhs, U set of Map grids
        self.g, self.rhs, self.U = {}, {}, {}
        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")
        self.rhs[self.s_start] = 0
        self.U[self.s_start] = self.CalculateKey(self.s_start)
        self.visited = set()

        self.count = 0
        self.fig = plt.figure()
    
    
    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(s), min(self.g[s], self.rhs[s])]

    def ComputeShortestPath(self):
        while True:
            s, v = self.TopKey()
            if v >= self.CalculateKey(self.s_goal) and self.rhs[self.s_goal] == self.g[self.s_goal]:
                break
            self.U.pop(s)
            self.visited.add(s)

            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
            
            for s_n in self.get_neighbor(s): # Here is a little different from original paper, but why???
                self.UpdateVertex(s_n)
    
    def TopKey(self):
        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def UpdateVertex(self, s):
        if s != self.s_start:
            self.rhs[s] = min(self.g[s_n] + self.cost(s_n, s) for s_n in self.get_neighbor(s))
        if s in self.U:
            self.U.pop(s)
        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)

    def get_neighbor(self, s):
        s_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                s_list.add(s_next)
        return s_list

    def h(self, s):
        goal = self.s_goal

        if self.heurisitic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])
        
    def cost(self, s_current, s_end):
        if self.iscollision(s_current, s_end):
            return float("inf")
        
        return math.hypot(s_end[0] - s_current[0], s_end[1] - s_current[1])
        
    def iscollision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True
        
        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
        
            if s1 in self.obs or s2 in self.obs:
                return True
        return False

    def extract_path(self):
        path = [self.s_goal]
        s = self.s_goal

        for k in range(100):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.iscollision(x, s):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_start:
                break
        return list(reversed(path))

    def plot_path(self, path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            self.visited = set()
            self.count += 1

            if (x, y) not in self.obs:
                self.obs.add((x, y))
            else:
                self.obs.remove((x, y))
                self.UpdateVertex((x, y))

            self.Plot.update_obs(self.obs)

            for s_n in self.get_neighbor((x, y)):
                self.UpdateVertex(s_n)

            self.ComputeShortestPath()

            plt.cla()
            self.Plot.plot_grid("Lifelong Planning A*")
            # self.plot_visited(self.visited)
            self.plot_path(self.extract_path())
            self.fig.canvas.draw_idle()            
        
    def run(self):
        self.Plot.plot_grid("Lifelong Planning A*")
        self.ComputeShortestPath()
        self.plot_path(self.extract_path())   
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        plt.show()
    
    def __repr__(self):
        return f'{self.__class__.__name__}(x_range={self.x!r}, y_range={self.y!r}, number of obs={len(self.obs)})'



s_start = (2,4)
s_goal = (45,5)

lpastar = LPAstar(s_start, s_goal)
lpastar.run()
import networkx as nx
import matplotlib.pyplot as plt
import sys
from colorama import init, Fore, Back, Style
from collections import deque
from push import push_algorithm
from push_extra_info import push_algorithm_extra_info
from push_opt import push_opt_algorithm
from push_opt_extra_info import push_opt_algorithm_extra_info
init()
sys.setrecursionlimit(100000)

vis_states = []
vis_idx = 0
paths_global = []
SHOW_VISUALIZATION = True
NODES_NUMBERED = False
ARROW_THICKNESS=4
NEWLY_FOUND_PATH_THICKNESS=4
OLD_PATH_THICKNESS=2

VISUALIZE_PATHS = True
ROBOT_COLOR = '#FFC93C'
GOAL_COLOR ='#f54029'
FINISHED_COLOR = '#03C988'
PATH_COLOR =  '#0081C9'



def visualize_state(M, moves=None, show_new_path=False, highlight_new_path=False): # G is a "Graph" instance
    edges = {e:{'color':'k', 'width':1, 'arrow':False} for e in list(M.G.edges())}
    
    node_colors = ['#aaa'] * M.G.number_of_nodes()
    if VISUALIZE_PATHS:
        for i in range(len(paths)):
            pathstarts = paths[i][0][1]
            if pathstarts > M.t: # path starts later
                continue
            if M.t - pathstarts >= len(paths[i]): # path is already finished
                continue             
            if pathstarts == M.t and not show_new_path:
                continue
            
            nodes, _ = paths[i][M.t-pathstarts]
            for j in range(len(nodes) - 2, -1, -1):
                a, b = nodes[j], nodes[j+1]
                if (a,b) not in edges.keys(): # orient the edge
                    edges[(a,b)] = edges[(b,a)].copy()
                    edges.pop((b,a))
                edges[(a,b)]['color'] = PATH_COLOR
                edges[(a,b)]['arrow'] = True
                if pathstarts == M.t and highlight_new_path:
                    edges[(a,b)]['width'] = NEWLY_FOUND_PATH_THICKNESS
                else:
                    edges[(a,b)]['width'] = max(edges[(a,b)]['width'], OLD_PATH_THICKNESS)
                
                    
    if moves is not None:
        for a,b in moves:
            if (a,b) not in edges.keys(): # orient the edge
                edges[(a,b)] = edges[(b,a)]
                edges.pop((b,a))
            edges[(a,b)]['color'] = ROBOT_COLOR
            edges[(a,b)]['arrow'] = True
    
    for r in M.robots_vis:
        node_colors[r - 1] = ROBOT_COLOR
    for t in M.targets_vis:
        node_colors[t - 1] = FINISHED_COLOR if node_colors[t - 1] == ROBOT_COLOR else GOAL_COLOR
    vis_states.append((M.G, M.vis_pos, node_colors, edges))

def onkeypress(event):
    global vis_states, vis_idx
    if event.key == 'm':
        event.canvas.figure.clear()
        vis_idx = (vis_idx + 1) % len(vis_states)
    if event.key == 'n':
        event.canvas.figure.clear()
        vis_idx = (vis_idx - 1 + len(vis_states)) % len(vis_states)

    if event.key == 'n' or event.key == 'm':
        draw_state()
        event.canvas.draw()

def draw_state():
    global vis_idx
    G, pos, node_color, edges = vis_states[vis_idx]
    nx.draw_networkx_nodes(G, pos = pos, node_color = node_color)
    
    for arrow in range(2):
        endpoints = []
        colors = []
        widths = []
        for e, style in edges.items():
            if style['arrow'] == arrow:
                endpoints.append(e)
                colors.append(style['color'])
                widths.append(style['width'])
        if arrow == 0:
            nx.draw_networkx_edges(G, pos = pos, edgelist=endpoints, edge_color=colors, width=widths)
        else:
            nx.draw_networkx_edges(G, pos = pos, edgelist=endpoints, width=widths, edge_color=colors, arrows=True, arrowstyle='-|>')
    
    if NODES_NUMBERED:
        nx.draw_networkx_labels(G, pos = pos)
    
def show_visualization(start_showing_from = 0):
    if len(vis_states) == 0:
        return
    global vis_idx
    vis_idx = start_showing_from
    
    fig = plt.figure()
    fig.canvas.mpl_connect('key_release_event', onkeypress)
    draw_state()
    plt.show()

class MAPF:
    def __init__(self, n_vertices, edge_list, robots, targets):
        self.G = nx.Graph()
        self.G.add_nodes_from(range(1, n_vertices + 1))
        self.G.add_edges_from(edge_list)
        if SHOW_VISUALIZATION:
            self.vis_pos = nx.kamada_kawai_layout(self.G)
        self.robots = robots
        self.targets = targets
        self.robots_vis = self.robots.copy()
        self.targets_vis = self.targets.copy()
        self.t = 0

    def make_moves(self, moves): 
        for move in moves:
            assert move in self.G.edges(), f"The move '{move}' is not an edge in the graph!"
            assert move[0] in self.robots_vis, f"The move '{move}' does not move a robot!"
        old_number_of_robots = len(self.robots_vis)

        # this is garbage! but it's fine:)
        moves_dict = {}
        for move in moves:
            moves_dict[move[0]] = move[1]
        for i in range(len(self.robots_vis)):
            if self.robots_vis[i] in moves_dict:
                self.robots_vis[i] = moves_dict[self.robots_vis[i]]
        assert len(set(self.robots_vis)) == old_number_of_robots, "You crashed some robots into each other!" 


def analyze_solution(M, instructions):
    makespan = 0
    for t, u, v in instructions:
        makespan = max(makespan, t + 1)

    print(Back.WHITE + Fore.BLACK + "Solution errors:" + Style.RESET_ALL + Fore.BLACK)
    found_error = False
    # conflict detection
    for i in range(len(instructions)):
        for j in range(i + 1, len(instructions)):
            t1, u1, v1 = instructions[i]
            t2, u2, v2 = instructions[j]
            
            if t1 == t2 and u1 != u2 and v1 == v2:
                found_error = True
                print(Back.RED + f"The instructions {instructions[i]} and {instructions[j]} move a robot into the same vertex")
            if t1 == t2 and v1 == u2 and v2 == u1:
                found_error = True
                print(Back.RED + f"The instructions {instructions[i]} and {instructions[j]} instantaneously swap robot positions")
            if t1 == t2 and u1 == u2 and v1 != v2:
                found_error = True
                print(Back.RED + f"The instructions {instructions[i]} and {instructions[j]} simultaneously move a robot to two locations")

    moves = [[] for _ in range(makespan)]
    for t, u, v in instructions:
        moves[t].append((u,v))
    
    robot_positions = set(M.robots)
    for t in range(makespan):
        starts, ends = zip(*moves[t]) # unzip
        for s in starts:
            if s not in robot_positions:
                found_error = True
                print(Back.RED + f"A move of a nonexistent robot in {s} at time {t} is attempted")
            else:
                robot_positions.remove(s)
        robot_positions = robot_positions.union(set(ends))
        if len(set(robot_positions)) < len(robot_positions):
            print(Back.RED + f"A move crashed a robot into a stationary robot at time {t}")
    if set(robot_positions) != set(M.targets):
        found_error = True
        print(Back.RED + "The given solution does not move a robot to all targets")

    if not found_error:
        print(Back.GREEN + "No errors!", end="")
    print(Style.RESET_ALL)

    if SHOW_VISUALIZATION:
        M.t = -1
        visualize_state(M)
        M.t = 0
        for t in range(makespan):
            if VISUALIZE_PATHS and M.t <= paths[-1][0][1]:
                visualize_state(M, show_new_path=True, highlight_new_path=True)
                visualize_state(M, show_new_path=True)
            visualize_state(M, show_new_path=True, moves = moves[t])
            #try:
            M.make_moves(moves[t])
            M.t+=1
            visualize_state(M)

            #except:
            #    break

    ell = 0
    for r in M.robots:
        q = deque([r])
        dist = [1e8 for v in range(M.G.number_of_nodes() + 1)]
        dist[r] = 0
        while len(q) > 0:
            cur = q.popleft()
            for v in M.G.neighbors(cur):
                if dist[v] == 1e8:
                    dist[v] = dist[cur] + 1
                    q.append(v)
        for g in M.targets:
            ell = max(ell, dist[g])
    
    print("Makespan of calculated solution:", makespan)
    print("Maximum robot/goal distance (ell):", ell)    
    print(f"Theoretical upper bound (min(n, V - n) + ell - 1): {min(len(M.robots), M.G.number_of_nodes() - len(M.robots)) + ell - 1}")

    print(Back.BLUE + Fore.WHITE + "Use n and m keys to navigate the solution." + Style.RESET_ALL)
    show_visualization()
    
def read_instance(filename):
    with open(f'instances/{filename}') as f:
        n, m, k = map(int, f.readline().split())
        robots = list(map(int, f.readline().split()))
        targets = list(map(int, f.readline().split()))
        edges = []
        for _ in range(m):
            a, b = map(int, f.readline().split())
            edges.append((a, b))
    return MAPF(n, edges, robots, targets)


options = sys.argv[1:-1]
if "--novis" in options or "--compare-optimized" in options:
    SHOW_VISUALIZATION = False
if "--no-paths" in options:
    VISUALIZE_PATHS = False

algo = None
if "--improved" in options:
    algo = push_opt_algorithm_extra_info
else:
    algo = push_algorithm_extra_info

if "--numbered" in options:
    NODES_NUMBERED = True
graph = sys.argv[-1]
M = read_instance(graph)

instructions, paths = algo(M)
paths_global = paths.copy()
print(Back.GREEN + Fore.BLACK + "Solution calculation finished, starting analysis of solution." + Style.RESET_ALL)
analyze_solution(M, instructions)
if "--compare-optimized" in options:
    instructions = push_opt_algorithm(M)
    print("Solution calculation finished, starting analysis of solution.")
    analyze_solution(M, instructions)
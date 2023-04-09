import networkx as nx
import matplotlib.pyplot as plt
import sys
from colorama import init, Fore, Back, Style
from collections import deque
from push import push_algorithm
from push_opt import push_opt_algorithm
init()
sys.setrecursionlimit(100000)

vis_states = []
vis_idx = 0
SHOW_VISUALIZATION = True
NODES_NUMBERED = False

def visualize_state(M, moves=None): # G is a "Graph" instance
    node_colors = ['#aaa'] * M.G.number_of_nodes()
    edge_colors = ['k'] * M.G.number_of_edges()
    if moves is not None:
        edges = list(M.G.edges())
        for i in range(len(edges)):
            a, b = edges[i]
            if (a, b) in moves or (b, a) in moves:
                edge_colors[i] = '#FFC93C'
    for r in M.robots_vis:
        node_colors[r - 1] = '#FFC93C'
    for t in M.targets_vis:
        node_colors[t - 1] = '#03C988' if node_colors[t - 1] == '#FFC93C' else '#0081C9'
    vis_states.append((M.G, M.vis_pos, node_colors, edge_colors))

def onkeypress(event):
    global vis_states, vis_idx
    if event.key == 'm':
        event.canvas.figure.clear()
        vis_idx = (vis_idx + 1) % len(vis_states)
    if event.key == 'n':
        event.canvas.figure.clear()
        vis_idx = (vis_idx - 1 + len(vis_states)) % len(vis_states)

    if event.key == 'n' or event.key == 'm':
        G, pos, node_color, edge_color = vis_states[vis_idx]
        nx.draw_networkx(G, pos = pos, node_color = node_color, edge_color = edge_color, with_labels=NODES_NUMBERED)
        event.canvas.draw()

def show_visualization(start_showing_from = 0):
    if len(vis_states) == 0:
        return
    G, pos, node_color, edge_color = vis_states[start_showing_from]
    global vis_idx
    vis_idx = start_showing_from
    
    fig = plt.figure()
    fig.canvas.mpl_connect('key_release_event', onkeypress)
    nx.draw_networkx(G, pos = pos, node_color = node_color, edge_color = edge_color, with_labels=NODES_NUMBERED)
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
        visualize_state(M)
        for t in range(makespan):
            visualize_state(M, moves = moves[t])
            try:
                M.make_moves(moves[t])
                visualize_state(M)
            except:
                break

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
if "--numbered" in options:
    NODES_NUMBERED = True
graph = sys.argv[-1]
M = read_instance(graph)

instructions = push_algorithm(M)
print("Solution calculation finished, starting analysis of solution.")
analyze_solution(M, instructions)
if "--compare-optimized" in options:
    instructions = push_opt_algorithm(M)
    print("Solution calculation finished, starting analysis of solution.")
    analyze_solution(M, instructions)
import networkx as nx
import matplotlib.pyplot as plt
import numpy
import sys
from collections import deque
numpy.random.seed(4812)


vis_states = []
vis_idx = 0

def visualize_state(M, moves=None): # G is a "Graph" instance
    node_colors = ['#aaa'] * M.G.number_of_nodes()
    edge_colors = ['k'] * M.G.number_of_edges()
    if moves is not None:
        edges = list(M.G.edges())
        for i in range(len(edges)):
            a, b = edges[i]
            if (a, b) in moves or (b, a) in moves:
                edge_colors[i] = '#FFC93C'
    for r in M.robots:
        node_colors[r - 1] = '#FFC93C'
    for t in M.targets:
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
        nx.draw_networkx(G, pos = pos, node_color = node_color, edge_color = edge_color)
        event.canvas.draw()

def show_visualization(start_showing_from = 0):
    if len(vis_states) == 0:
        return
    G, pos, node_color, edge_color = vis_states[start_showing_from]
    global vis_idx
    vis_idx = start_showing_from
        
    fig = plt.figure()
    fig.canvas.mpl_connect('key_release_event', onkeypress)
    nx.draw_networkx(G, pos = pos, node_color = node_color, edge_color = edge_color)
    
    plt.show()

    


class MAPF:
    def __init__(self, n_vertices, edge_list, robots, targets):
        self.G = nx.Graph()
        self.G.add_nodes_from(range(1, n_vertices + 1))
        self.G.add_edges_from(edge_list)
        self.vis_pos = nx.kamada_kawai_layout(self.G)
        self.robots = robots
        self.targets = targets

    def make_moves(self, moves): 
        for move in moves:
            assert move in self.G.edges(), f"The move '{move}' is not an edge in the graph!"
            assert move[0] in self.robots, f"The move '{move}' does not move a robot!"
        self.new_robots = self.robots.copy()
        # this is garbage! but it's fine:)
        for i in range(len(self.robots)):
            for move in moves:
                if self.robots[i] == move[0]:
                    self.new_robots[i] = move[1]
                    break
        old_number_of_robots = len(self.robots)
        self.robots = self.new_robots
        assert len(set(self.robots)) == old_number_of_robots, "You crashed some robots into each other!" 

with open(f'graphs/{sys.argv[1]}.txt') as f:
    n, m, k = map(int, f.readline().split())
    robots = list(map(int, f.readline().split()))
    targets = list(map(int, f.readline().split()))
    edges = []
    for _ in range(m):
        a, b = map(int, f.readline().split())
        edges.append((a, b))

# ALGORITHM START
M = MAPF(n, edges, robots, targets)
# starts is the set of starting nodes, and zero_nodes
# is the set of nodes where entering them has zero cost.
# ignore_node is just an optional parameter in case a certain
# node in the graph should be ignored during the calculation
def zero_one_bfs(M, starts, zero_nodes, ignore_node=None):
    relevant_starts = [r for r in starts if r != ignore_node]
    q = deque(relevant_starts)
    N = M.G.number_of_nodes()
    distance = [N for _ in range(N + 1)]
    previous = [-1 for _ in range(N + 1)]
    for r in relevant_starts:
        distance[r] = 0
    while len(q) > 0:
        cur = q.popleft()
        if cur == ignore_node:
            continue
        for neighbor in M.G.neighbors(cur):
            w = 0 if neighbor in zero_nodes else 1
            if distance[cur] + w < distance[neighbor]:
                distance[neighbor] = distance[cur] + w
                previous[neighbor] = cur
                if w == 1:
                    q.append(neighbor)
                else:
                    q.appendleft(neighbor)
    return distance, previous
    

# dones is the list of robots whose trajecory has been calculated, and with respect
# to calculating the next routes, we can assume they will have arrived
dones = [r for r in robots if r in targets]
# arrived is the list of robots who have actually arrived in their desired position,
# although they may be pushed around and its target may be swapped in the future
arrived = dones.copy()
# active_robots and active_targets is the list of robots and targets whose initial
# trajectory has not yet been determined
active_robots = [r for r in robots if r not in dones]
active_targets = [t for t in targets if t not in dones]    

ongoing_paths = []
visualize_state(M)
time_steps = 0
while len(arrived) < len(robots): 
    time_steps += 1
    found_tie = False
    while len(active_robots) > 0 and not found_tie:
        distances, previous = zero_one_bfs(M, active_robots, dones)
        best_target = active_targets[0]
        for t in active_targets:
            if distances[t] < distances[best_target]:
                best_target = t
        
        # find best path
        best_path = [best_target]
        current_node = best_target
        while previous[current_node] != -1:
            best_path.append(previous[current_node])
            current_node = previous[current_node]
        
        best_distance = distances[best_target]
        best_robot = best_path[-1]
        
        distances, paths = zero_one_bfs(M, active_robots, dones, ignore_node = best_robot)
        closest_target_distance = M.G.number_of_nodes()
        for t in active_targets:
            if distances[t] < closest_target_distance:
                closest_target_distance = distances[t]
        
        # Tie
        if closest_target_distance == best_distance:
            found_tie = True
        ongoing_paths.append(best_path)
        active_robots.remove(best_robot)
        active_targets.remove(best_target)
        dones.append(best_target)

    moves = []
    updated_paths = []
    for path in ongoing_paths:
        while len(path) > 1 and path[-2] in arrived:
            moves.append((path[-1], path[-2]))
            path.pop(-1)
        moves.append((path[-1], path[-2]))
        path.pop(-1)
        if len(path) == 1:
            arrived.append(path[-1])
        else:
            updated_paths.append(path)
    ongoing_paths = updated_paths
    visualize_state(M, moves = moves)
    M.make_moves(moves)
    visualize_state(M)
# ALGORITHM END

# CALCULATING DIAMETER! NOT PART OF THE ALGORITHM
paths_list = list(nx.all_pairs_shortest_path(M.G))
diameter = 0
for i in range(1, n + 1):
    for j in range(1, n + 1):
        diameter = max(diameter, len(paths_list[i - 1][1][j]))
# DONE CALCULATNG!


print("DIAMETER:", diameter)
print(f"THIS SOLUTION TOOK {time_steps} TIME STEPS!")
print(f"UPPER BOUND (AGENT + DIAM - 2): {len(robots) + diameter - 2}")
show_visualization()

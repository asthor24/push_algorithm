import networkx as nx
import matplotlib.pyplot as plt
import numpy
import sys
from collections import deque
from colorama import init, Fore, Back, Style
init()
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
        old_number_of_robots = len(self.robots)

        # this is garbage! but it's fine:)
        moves_dict = {}
        for move in moves:
            moves_dict[move[0]] = move[1]
        for i in range(len(self.robots)):
            if self.robots[i] in moves_dict:
                self.robots[i] = moves_dict[self.robots[i]]
        assert len(set(self.robots)) == old_number_of_robots, "You crashed some robots into each other!" 

with open(f'graphs/{sys.argv[1]}.txt') as f:
    n, m, k = map(int, f.readline().split())
    robots = list(map(int, f.readline().split()))
    targets = list(map(int, f.readline().split()))
    edges = []
    for _ in range(m):
        a, b = map(int, f.readline().split())
        edges.append((a, b))


# tasks
# - rewrite the simulator so that it just takes in the (t, u, v) type instructions and generates the simulation
#   - if there is some error, then it reports it, but shows the simulation until the error happens
# - rewrite push algorithm to output these types of instructions

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
    

def push_algorithm_recursive(M, R, T, F, t):
    if len(R) == 0:
        return []
    distances, previous = zero_one_bfs(M, R, F)
    best_target = next(iter(T))
    for g in T:
        if distances[g] < distances[best_target]:
            best_target = g
    
    # find best path
    best_path = [best_target]
    current_node = best_target
    while previous[current_node] != -1:
        best_path.append(previous[current_node])
        current_node = previous[current_node]
    
    best_distance = distances[best_target]
    best_path = list(reversed(best_path))
    best_robot = best_path[0]

    answer = []
    t_plan = t
    for i in range(len(best_path) - 1):
        answer.append((t_plan, best_path[i], best_path[i + 1]))
        if best_path[i+1] not in F:
            t_plan += 1
    
    R.remove(best_robot)
    T.remove(best_target)
    F.add(best_target)

    return answer + push_algorithm_recursive(M, R, T, F, t + 1)


def push_algorithm(M):
    # dones is the list of robots whose trajecory has been calculated, and with respect
    # to calculating the next routes, we can assume they will have arrived
    F = set([r for r in M.robots if r in M.targets])

    # active_robots and active_targets is the list of robots and targets whose initial
    # trajectory has not yet been determined
    R = set([r for r in M.robots if r not in F])
    T = set([t for t in M.targets if t not in F])
    return push_algorithm_recursive(M, R, T, F, 0)


def visualize_solution(M, instructions):
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
            if t1 == t2 and v1 != u2 and v2 == u1:
                found_error = True
                print(Back.RED + f"The instructions {instructions[i]} and {instructions[j]} instantaneously swaps robot positions")
            if t1 == t2 and u1 == v2 and v1 != v2:
                found_error = True
                print(Back.RED + f"The instructions {instructions[i]} and {instructions[j]} simultaneously move a robot to two locations")

    moves = [[] for _ in range(makespan)]
    for t, u, v in instructions:
        moves[t].append((u,v))
    
    robot_positions = M.robots.copy()
    for t in range(makespan):
        starts, ends = zip(*moves[t]) # unzip
        for s in starts:
            if s not in robot_positions:
                found_error = True
                print(Back.RED + "A move of a nonexistent robot in {s} at time {t} is attempted")
            else:
                robot_positions.remove(s)
        robot_positions += ends
    if set(robot_positions) != set(M.targets):
        found_error = True
        print(Back.RED + "The given solution does not move a robot to all targets")

    if not found_error:
        print(Back.GREEN + "No errors!", end="")
    print(Style.RESET_ALL)

    visualize_state(M)
    for t in range(makespan):
        visualize_state(M, moves = moves[t])
        try:
            M.make_moves(moves[t])
            visualize_state(M)
        except:
            break

    paths_list = list(nx.all_pairs_shortest_path(M.G))
    ell = 0
    for r in M.robots:
        for g in M.targets:
            ell = max(ell, len(paths_list[i - 1][1][j]))

    print("Makespan of calculated solution:", makespan)
    print("Maximum robot/target distance (ell):", ell)    
    print(f"Theoretical upper bound (n + ell - 1): {len(robots) + ell - 1}")

    show_visualization()
    

M = MAPF(n, edges, robots, targets)
instructions = push_algorithm(M)
visualize_solution(M, instructions)

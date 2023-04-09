from collections import deque
def closest_pair(M, R, T, F, taken_nodes):
    q = deque(T)
    distance = [1e8 for _ in range(M.G.number_of_nodes() + 1)]
    previous = [v for v in range(M.G.number_of_nodes() + 1)]
    for g in T:
        distance[g] = 0
    while len(q) > 0:
        cur = q.popleft()
        if cur in taken_nodes:
            continue
        if cur in R:
            path = [cur]
            while previous[cur] != cur:
                path.append(previous[cur])
                cur = previous[cur]
            return (path[0], path, distance)
        for neighbor in M.G.neighbors(cur):
            w = 0 if neighbor in F else 1
            if distance[cur] + w < distance[neighbor]:
                distance[neighbor] = distance[cur] + w
                previous[neighbor] = cur
                if w == 1:
                    q.append(neighbor)
                else:
                    q.appendleft(neighbor)
    return (-1,-1,-1)

def push_opt_algorithm(M):
    R = set([r for r in M.robots if r not in M.targets])
    T = set([t for t in M.targets if t not in M.robots])
    F = set([t for t in M.targets if t in M.robots])
    i = 0
    S = []
    r,p,distx = closest_pair(M,R,T,F,set())
    last_path_found = distx[r]
    ignore_nodes = set()
    while len(R) > 0:
        r, p, dist1 = closest_pair(M, R, T, F, ignore_nodes)
        if r == -1 or last_path_found != dist1[r]: 
            ignore_nodes.clear()
            i += 1
            r,p,distx = closest_pair(M,R,T,F,set())
            last_path_found = distx[r]
            continue
            
        t = i
        for j in range(len(p) - 1):
            S.append((t, p[j], p[j+1]))
            ignore_nodes.add(p[j+1])
            if p[j+1] not in F:
                t += 1
        R.remove(r)
        T.remove(p[-1])
        F.add(p[-1])
    return S

from collections import deque
def closest_pair(M, robots, targets, zero_nodes):
    q = deque(robots)
    distance = [1e8 for _ in range(M.G.number_of_nodes() + 1)]
    previous = [v for v in range(M.G.number_of_nodes() + 1)]
    for r in robots:
        distance[r] = 0
    while len(q) > 0:
        cur = q.popleft()
        if cur in targets:
            path = [cur]
            while previous[cur] != cur:
                path.append(previous[cur])
                cur = previous[cur]
            path = list(reversed(path))
            return (path[0], path[-1], path)
        for neighbor in M.G.neighbors(cur):
            w = 0 if neighbor in zero_nodes else 1
            if distance[cur] + w < distance[neighbor]:
                distance[neighbor] = distance[cur] + w
                previous[neighbor] = cur
                if w == 1:
                    q.append(neighbor)
                else:
                    q.appendleft(neighbor)

def push_algorithm_recursive(M, R, T, F, t):
    if len(R) == 0:
        return []
    r, g, p = closest_pair(M, R, T, F)
    t_plan = t
    result = []
    for i in range(len(p) - 1):
        result.append((t_plan, p[i], p[i+1]))
        if p[i+1] not in F:
            t_plan += 1
    R.remove(r)
    T.remove(g)
    F.add(g)
    return result + push_algorithm_recursive(M, R, T, F, t + 1)

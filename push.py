from collections import deque
def closest_pair(M, R, F, g):
    q = deque([g])
    distance = [1e8 for _ in range(M.G.number_of_nodes() + 1)]
    previous = [v for v in range(M.G.number_of_nodes() + 1)]
    distance[g] = 0
    while len(q) > 0:
        cur = q.popleft()
        if cur in R:
            path = [cur]
            while previous[cur] != cur:
                path.append(previous[cur])
                cur = previous[cur]
            return (path[0], path)
        for neighbor in M.G.neighbors(cur):
            w = 0 if neighbor in F else 1
            if distance[cur] + w < distance[neighbor]:
                distance[neighbor] = distance[cur] + w
                previous[neighbor] = cur
                if w == 1:
                    q.append(neighbor)
                else:
                    q.appendleft(neighbor)

def push_algorithm(M):
    R = set([r for r in M.robots if r not in M.targets])
    T = set([t for t in M.targets if t not in M.robots])
    F = set([t for t in M.targets if t in M.robots])
    i = 0
    S = []
    while len(R) > 0:
        g = next(iter(T))
        r, p = closest_pair(M, R, F, g)
        t = i
        for j in range(len(p) - 1):
            S.append((t, p[j], p[j+1]))
            if p[j+1] not in F:
                t += 1
        R.remove(r)
        T.remove(g)
        F.add(g)
        i += 1
    return S

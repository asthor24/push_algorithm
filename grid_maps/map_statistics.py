import sys
mp = []
with open(f'{sys.argv[1]}.map') as f:
    h, w = map(int, f.readline().split())
    for x in range(h):
        mp.append(f.readline())


vertices = 0
edges_double = 0
for i in range(h):
    for j in range(w):
        if mp[i][j] != '@':
            vertices += 1
            if i > 0 and mp[i-1][j] != '@':
                edges_double += 1
            if i < h - 1 and mp[i+1][j] != '@':
                edges_double += 1
            if j > 0 and mp[i][j-1] != '@':
                edges_double += 1
            if i < w - 1 and mp[i][j+1] != '@':
                edges_double += 1
print(vertices, edges_double//2)

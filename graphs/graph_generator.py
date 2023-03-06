import random
print("Input the number of vertices: ")
V = int(input())
print("Input the number of edges (make sure it is at least V - 1): ")
E = int(input())
print("Input the number of robots: ")
R = int(input())
import sys
with open(f'{sys.argv[1]}.txt', 'w') as f:
    f.write(str(V) + " " + str(E) + " " + str(R) + "\n")
    robots = random.sample(list(range(1, V + 1)), R)
    targets = random.sample(list(range(1, V + 1)), R)
    f.write(" ".join(list(map(str, robots))) + "\n")
    f.write(" ".join(list(map(str, targets))) + "\n")
    for i in range(2, V + 1):
        to = random.randint(1, i - 1)
        f.write(str(to) + " " + str(i) + "\n")
    for i in range(V, E + 1):
        a = random.randint(1, V)
        b = random.randint(1, V)
        while a == b:
            b = random.randint(1, V)
        f.write(str(a) + " " + str(b) + "\n")

    

        

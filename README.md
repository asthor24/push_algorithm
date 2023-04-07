# Push Algorithm
## Usage
```python sim.py [--novis] [--numbered] [INSTANCE_FILE]```

[INSTANCE_FILE] is a path to the file describing the instance we want the algorithm to run on, as described below. [--novis] is a flag which indicates that the simulator should not visualize the solution, but it will still report relevant statistics about the solution. [--numbered] is a flag which indicates whether the visualizer should label the vertices with the numbers they are given in the instance file. 
## Instance files
Instance files are in the `instances` directory, and are of the following format:
```
[V] [E] [n]
[a list containing the n vertices that initially contain robots]
[a list containing the n goal vertices]
[E lines of the form "u v", indicating an edge between vertices u and v]
```
Note that V is the number of vertices in the graph, E is the number of edges in the graph, and n is the number of robots in the problem instance. Note that vertices are numbered from 1 to V. To generate a random problem instance (where you can set V, E and n), you can run the following command:

```python generate_instance.py [INSTANCE_FILE]```

where [INSTANCE_FILE] is what you would like to call the instance you are about to generate.
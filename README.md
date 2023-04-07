# Push Algorithm
This is a repo containing an implementation of the push algorithm (in `push.py`), and a visualizer of the solutions to anonymous MAPF which it finds.
## Installation
We require that you have python 3.x and git installed. Run the following commands in a directory of your choosing:

```
git clone https://github.com/asthor24/push_algorithm/
pip3 install networkx matplotlib colorama scipy
```

## Usage
Navigate to the directory you created during the installation called `push_algorithm` and run the following command:

```python3 sim.py [--novis] [--numbered] [INSTANCE_FILE]```

[INSTANCE_FILE] is a path to the file describing the instance we want the algorithm to run on, as described below. [--novis] is a flag which indicates that the simulator should not visualize the solution, but it will still report relevant statistics about the solution. [--numbered] is a flag which indicates whether the visualizer should label the vertices with the numbers they are given in the instance file. For an example of the visualization run the following:

```python3 sim.py gen_1.txt```

A window should pop up which looks like the one below. Yellow vertices are robots, blue vertices are goals, and green vertices are both robots and goals.

![image](https://user-images.githubusercontent.com/37704951/230690053-7cde3cd9-02a1-495d-87a4-00110a51c729.png)

You can use the `n` and `m` keys to navigate the solution over time. Note that between each timestep there is an intermediate step indicating which moves robots are about to take during that timestep.

## Instance files
Instance files are in the `instances` directory, and are of the following format:

```
[V] [E] [n]
[a list containing the n vertices that initially contain robots]
[a list containing the n goal vertices]
[E lines of the form "u v", indicating an edge between vertices u and v]
```

Note that V is the number of vertices in the graph, E is the number of edges in the graph, and n is the number of robots in the problem instance. Note that vertices are numbered from 1 to V. To generate a random problem instance (where you can set V, E and n), you can run the following command:

```python3 generate_instance.py [INSTANCE_FILE]```

where [INSTANCE_FILE] is what you would like to call the instance you are about to generate.

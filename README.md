# ECE 553 Computer Networks - Assignment 4 
Contains code for problem 3 on assignment 4.

Zachary Perry, 4/24/25

## Requirements
*Go version 1.22.9* - This is the version of Go installed on the Hydra/Tesla machines. As such, this assingment is written in this version.

## How to Run
A makefile is included here for compiling. 

**To Compile**: `make`

**To Run**: `./bin/main inputFileName`

*NOTE: I have provided input file examples in the input directory for use.*

### Question 3
Write a program that reads a weighted undirected graph and a source vertex, then computes the shortest distances from the source to all other vertices using Dijkstra’s algorithm

*Input format*
- First line: two integers *n* (number of nodes) and *m* (number of edges)
- Next m lines: three integers *u*, *v*, *w* — an undirected edge between nodes u and v with weight w
- Final line: a single integer *s* — the source vertex

*Output Format*
Print the shortest distance from s to every node from 1 to n. If a node is unreachable, print INF.

#### Intermediate Level (For ECE 553 Students Only):
In addition to computing the shortest distances, also output the actual shortest path from the source to every reachable node. If multiple shortest paths exist, printing any one of them is sufficient.

*Output Format*
- The shortest distance from s to v
- The shortest path from s to v as a sequence of nodes

*Constraints*
- 1 ≤ n ≤ 1000
- 0 ≤ w ≤ 10^4

#### Example Input: 
```
5 6
1 2 2
1 3 4
2 3 1
2 4 7
3 5 3
4 5 1
1
```

#### Example Output: 
```
1: Distance = 0, Path = 1
2: Distance = 2, Path = 1 -> 2
3: Distance = 3, Path = 1 -> 2 -> 3
4: Distance = 7, Path = 1 -> 2 -> 3 -> 5 -> 4
5: Distance = 6, Path = 1 -> 2 -> 3 -> 5
```

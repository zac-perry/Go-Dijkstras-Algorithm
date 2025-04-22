# ECE 553 Computer Networks - Assignment 4 
Contains code for problem 3 on assignment 4.

Zachary Perry, 4/24/25

## Requirements
*Go version 1.22.9* - This is the version of Go installed on the Hydra/Tesla machines. As such, this assingment is written in this version

## How to Run
```
./bin/main inputFileName
```
**All input file examples are provided in the input directory for use**


### Question 3
Write a program that reads a weighted undirected graph and a source vertex, then computes the shortest distances from the source to all other vertices using Dijkstra’s algorithm

*Input format*
- First line: two integers n (number of nodes) and m (number of edges)
- Next m lines: three integers u, v, w — an undirected edge between nodes u and v with weight w
- Final line: a single integer s — the source vertex

*Output Format*
Print the shortest distance from s to every node from 1 to n. If a node is unreachable, print INF.

#### Intermediate Level (For ECE 553 Students Only):
In addition to computing the shortest distances, also output the actual shortest path from the source to every reachable node. If multiple shortest paths exist, printing any one of them is sufficient


### progress / todo
- [x] reading in the graph via DIMACS format (same as 581) 
- [ ] shortest distance from source to all vertices
- [ ] shortest path from the source to each reachable node. If multiple exist, just print one

NOTE: 
- Input here is slightly different. The last number in the file is the source
- I usually have an empty vertex with key zero. Keep this is mind. Anything empty I may need to clean up and remove to avoid anything horrible. 
    - Am removing empty entries rn. Keep in mind if bug

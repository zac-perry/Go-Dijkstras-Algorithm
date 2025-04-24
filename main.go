/*
 * Zachary Perry
 * ECE 553 -- Computer Networks Assignment 4
 * 4/24/25
 *
 * Purpose of this program is to implement Dijkstra's shortest path algorithm.
 * The goal is to find the shortest path in an undirected, weighted graph.
 */

package main

import (
	"bufio"
	"container/heap"
	"fmt"
	"log"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"
)

/*
 * Graph struct will represent all edges and nodes.
 * nodes -> map containing all nodes in the Graph. (key: node value, val: node)
 * edges -> map containing all edges in the Graph. (Adj list, key: node value, val: edge)
 */
type Graph struct {
	nodes map[int]*Node
	edges map[int][]*Edge
}

/*
 * Node struct will represent each node in the graph.
 * value      -> Node value.
 * distance   -> Distance from the source node.
 * visited    -> If the node has been visited yet.
 * prevNode   -> Tracks the previous node visited to get to the
 *               current node (to track the actual path).
 */
type Node struct {
	value    int
	distance int
	visited  bool
	prevNode *Node
}

/*
 * Edge struct represents an individual edge for the adj list.
 * to     -> tracks the node this edge goes to.
 * weight -> weight of the edge, capacity that can flow through it.
 */
type Edge struct {
	to     int
	weight int
}

/*
 * Min Priority Queue implementation using Go's min heap interface.
 * NOTE: In Go, you must fulfill the interface and implement each function.
 */
type PriorityQueue []*Node

func (queue PriorityQueue) Len() int {
	return len(queue)
}

func (queue PriorityQueue) Less(i, j int) bool {
	return queue[i].distance < queue[j].distance
}

func (queue PriorityQueue) Swap(i, j int) {
	queue[i], queue[j] = queue[j], queue[i]
}

func (queue *PriorityQueue) Push(x any) {
	*queue = append(*queue, x.(*Node))
}

func (queue *PriorityQueue) Pop() any {
	temp := *queue
	length := len(temp)
	value := temp[length-1]
	*queue = temp[0 : length-1]

	return value
}

/*
 * NewGraph() just creates a new graph instance.
 * Initializes the nodes and edges maps.
 * NOTE: the edges slice for each node will also contain any backwards edges.
 */
func NewGraph(nodeCount int) *Graph {
	graph := &Graph{
		nodes: make(map[int]*Node),
		edges: make(map[int][]*Edge),
	}

	for i := 1; i <= nodeCount; i++ {
		graph.edges[i] = []*Edge{}
	}

	return graph
}

/*
 * AddNode() will initialize/add a new node to the Graph if it doesn't exist.
 * NOTE: The distance is initialized to inf to start.
 */
func (graph *Graph) AddNode(value int) {
	_, exists := graph.nodes[value]

	if !exists {
		node := &Node{
			value:    value,
			distance: math.MaxInt32,
			visited:  false,
			prevNode: nil,
		}

		graph.nodes[value] = node
	}
}

/*
 * AddEdge() will add an edge from node1 to node2 with the specified weight to the graph.
 * Additionally, it also adds a backwards edge (since the graph is assumed to be undirected).
 */
func (graph *Graph) AddEdge(node1, node2, weight int) {
	forward := &Edge{
		to:     node2,
		weight: weight,
	}

	backward := &Edge{
		to:     node1,
		weight: weight,
	}

	// Append to respective node Edge slice.
	graph.edges[node1] = append(graph.edges[node1], forward)
	graph.edges[node2] = append(graph.edges[node2], backward)
}

/*
 * Dijkstras() implements Dijkstras algorithm to find the shortest path from the source
 *             to each node in the graph.
 * NOTE: As previously mentioned, this is using a priority queue built from Go's heap interface.
 */
func (graph *Graph) Dijkstras(source int) {
	// Set the source node distance to 0 (was initalized to inf to start).
	sourceNode := graph.nodes[source]
	sourceNode.distance = 0

	// Create the priority queue and add the source.
	pqueue := &PriorityQueue{}
	heap.Init(pqueue)
	heap.Push(pqueue, sourceNode)

	/*
	 * Pop a node off the queue to process.
	 * If it hasn't been visited, then process the neighbors that have also not been visited.
	 * Check and update distances, add the neighbor to the queue if distance was updated.
	 */
	for pqueue.Len() != 0 {
		currNode := heap.Pop(pqueue).(*Node)

		if currNode.visited {
			continue
		}

		currNode.visited = true

		// Process the neighbors of this current node using adj list.
		for _, edge := range graph.edges[currNode.value] {
			neighbor := graph.nodes[edge.to]

			if neighbor.visited {
				continue
			}

			currentDistance := currNode.distance + edge.weight

			if currentDistance < neighbor.distance {
				neighbor.distance = currentDistance
				neighbor.prevNode = currNode

				heap.Push(pqueue, neighbor)
			}
		}
	}
}

/*
 * ReadFile() -- Read in the file based on modified DIMACS format (provided in assignment).
 * NOTE: Ending value in the file specifies the source node.
 */
func ReadFile(fileName string) (*Graph, int) {
	file, err := os.Open(fileName)
	if err != nil {
		log.Fatal("ReadFile(): Error opening the file")
	}

	defer file.Close()

	scanner := bufio.NewScanner(file)
	if !scanner.Scan() {
		log.Fatal("ReadFile(): Issue scanning the file -- may be empty..")
	}

	// First row (2 values) -> node count and then the edge count.
	firstLine := strings.Fields(scanner.Text())
	if len(firstLine) < 2 || len(firstLine) > 2 {
		log.Fatal(
			"ReadFile(): Error parsing the first line -- either too few arguments or too many.",
		)
	}

	nodeCount, err := strconv.Atoi(firstLine[0])
	if err != nil {
		log.Fatal("ReadFile(): Error reading in the nodeCount")
	}

	edgeCount, err := strconv.Atoi(firstLine[1])
	if err != nil {
		log.Fatal("ReadFile(): Error reading in the edgeCount")
	}

	graph := NewGraph(nodeCount)
	source := 0
	edgeReadCount := 1

	// Initialize nodes.
	for i := 1; i <= nodeCount; i++ {
		graph.AddNode(i)
	}

	// Read in the edges + weights & the specified source node.
	for scanner.Scan() {

		line := strings.Fields(scanner.Text())

		// NOTE: Last value is the source.
		if edgeReadCount > edgeCount {
			source, err = strconv.Atoi(line[0])
			if err != nil {
				log.Fatal("ReadFile(): Error reading in the last value (source)")
			}

			break
		}

		if len(line) < 3 || len(line) > 3 {
			log.Fatal("ReadFile(): edge row either has too few or too many arguments")
		}

		node1, err := strconv.Atoi(line[0])
		if err != nil {
			log.Fatal("ReadFile(): Error reading in node1")
		}

		node2, err := strconv.Atoi(line[1])
		if err != nil {
			log.Fatal("ReadFile(): Error reading in the node2")
		}

		weight, err := strconv.Atoi(line[2])
		if err != nil {
			log.Fatal("ReadFile(): Error reading in the weight")
		}

		graph.AddEdge(node1, node2, weight)
		edgeReadCount++
	}

	return graph, source
}

/*
 * PrintDistanceAndPaths() just prints out the path info (distance, actual path).
 */
func (graph *Graph) PrintDistanceAndPaths(source int) {
	// NOTE: Sorting the map keys here bc Go doesn't auto sort the map :(
	// Done so that I can output everything in sorted order.
	sortedKeys := make([]int, 0)

	for i := range graph.nodes {
		sortedKeys = append(sortedKeys, i)
	}
	sort.Ints(sortedKeys)

	// Loop over the sorted keys and output node info.
	for _, key := range sortedKeys {
		node := graph.nodes[key]
		fmt.Printf("%-3d: ", key)

		if node.distance == math.MaxInt32 {
			fmt.Printf("Distance = INF , ")
		} else {
			fmt.Printf("Distance = %-4d, ", node.distance)
		}

		if node.value == source && node.prevNode == nil {
			fmt.Printf("Path = %d\n", node.value)
			continue
		}

		if node.distance == math.MaxInt32 {
			fmt.Printf("Path = nil, no valid path\n")
			continue
		}

		// Loop through the prevNodes for the current node, building the path found.
		currPath := make([]int, 0)
		tempNode := node

		for {
			if tempNode == nil {
				break
			}

			currPath = append([]int{tempNode.value}, currPath...)
			tempNode = tempNode.prevNode
		}

		fmt.Printf("Path = ")

		for i, val := range currPath {
			if i+1 == len(currPath) {
				fmt.Printf("%d\n", val)
			} else {
				fmt.Printf("%d -> ", val)
			}
		}
	}
}

func main() {
	if len(os.Args) < 2 {
		log.Fatal("usage: ./bin/main filename")
	}

	fileName := os.Args[1]
	graph, source := ReadFile(fileName)
	graph.Dijkstras(source)
	graph.PrintDistanceAndPaths(source)
}

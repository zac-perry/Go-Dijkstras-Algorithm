/*
 * Zachary Perry
 * ECE 553 -- Computer Networks Assignment 4
 * 4/24/25
 *
 * Purpose of this program is to implement Dijkstra's shortest path algorithm.
 * The goal is to find the shortest path in an undirected, weighted graph
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
 * TODO: UPDATE THIS COMMENT
 * Key in the map       -> vertex value.
 * Val for each vertex  -> list of edges.
 */
type Graph struct {
	nodes map[int]*Node
	edges map[int][]*Edge
}

/*
 * Node struct will represent each node in the graph.
 * Value      -> vertex value.
 * Distance   -> Distance from the source node.
 * Visited    -> If the node has been visited yet.
 * PrevNode   -> Tracks the previous node visited to get to the
 *               current node (to track the actual path).
 */
type Node struct {
	value    int
	distance int
	visited  bool
	prevNode *Node
}

/*
 * Edge struct represents an individual edge.
 * to       -> destination vertex, where this edge goes to.
 * capacity -> weight of the edge, capacity that can flow through it.
 */
type Edge struct {
	to     int
	weight int
}

/*
 * Min Priority Queue implementation using Go's min heap interface
 * NOTE: These functions are just fulfilling the interface.
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
	placeholder := *queue
	length := len(placeholder)
	value := placeholder[length-1]
	*queue = placeholder[0 : length-1]

	return value
}

/*
 * NewGraph() just creates a new graph instance.
 * The map will hold vertex values as the keys and their outgoing edges in a slice as the val.
 * NOTE: the edges slice for each vertex will also contain any backwards edges.
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
 *
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
 * TODO: UPDATE COMMENTS
 * AddEdge() will add an edge from v1 to v2 with the specified weight as the capacity to the graph.
 * Additionally, it also adds a backwards edge (with 0 capacity).
 */
func (graph *Graph) AddEdge(v1, v2, weight int) {
	// Forward edge (v1 -> v2).
	forward := &Edge{
		to:     v2,
		weight: weight,
	}

	// Backward edge (v2 -> v1).
	backward := &Edge{
		to:     v1,
		weight: weight,
	}

	// Append to respective node Edge slice.
	graph.edges[v1] = append(graph.edges[v1], forward)
	graph.edges[v2] = append(graph.edges[v2], backward)
}

func (graph *Graph) Dijkstras(source int) {
	// Set the source node distance to 0 (was initalized to inf to start)
	sourceNode := graph.nodes[source]
	sourceNode.distance = 0

	// NOTE: again, this is just instantiating the Priority Queue using the heap interface
	pqueue := &PriorityQueue{}
	heap.Init(pqueue)
	heap.Push(pqueue, graph.nodes[source])

	for pqueue.Len() != 0 {
		currNode := heap.Pop(pqueue).(*Node)

		if currNode.visited {
			continue
		}

		currNode.visited = true

		// process the neighbors of this current node.
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
 * ReadFile() -- Read in the file based on modified DIMACS format
 * Ending value is the source node
 */
func ReadFile(fileName string) (*Graph, int) {
	file, err := os.Open(fileName)
	if err != nil {
		log.Fatal("Error opening the file")
	}

	defer file.Close()

	scanner := bufio.NewScanner(file)
	if !scanner.Scan() {
		log.Fatal("readFile(): Issue scanning the file -- may be empty..")
	}

	// First row (2 values) -> node count and then the edge count.
	firstLine := strings.Fields(scanner.Text())
	if len(firstLine) < 2 || len(firstLine) > 2 {
		log.Fatal(
			"readFile(): Error parsing the first line -- either too few arguments or too many. Should only include node count and edge count",
		)
	}

	nodeCount, err := strconv.Atoi(firstLine[0])
	if err != nil {
		log.Fatal("readFile(): Error reading in the nodeCount")
	}

	edgeCount, err := strconv.Atoi(firstLine[1])
	if err != nil {
		log.Fatal("readFile(): Error reading in the edgeCount")
	}

	graph := NewGraph(nodeCount)
	source := 0
	edgereadCount := 1

	for i := 1; i <= nodeCount; i++ {
		graph.AddNode(i)
	}

	// Read in the remaining rows containing the edges (v1, v2, arc weight).
	for scanner.Scan() {

		line := strings.Fields(scanner.Text())

		// NOTE: Last value is the source.
		if edgereadCount > edgeCount {
			source, err = strconv.Atoi(line[0])
			if err != nil {
				log.Fatal("readFile(): Error reading in the last value (source)")
			}

			break
		}

		if len(line) < 3 || len(line) > 3 {
			log.Fatal("readFile(): vertex and weight row either has too few or too many arguments")
		}

		v1, err := strconv.Atoi(line[0])
		if err != nil {
			log.Fatal("readFile(): Error reading in v1")
		}

		v2, err := strconv.Atoi(line[1])
		if err != nil {
			log.Fatal("readFile(): Error reading in the v2")
		}

		weight, err := strconv.Atoi(line[2])
		if err != nil {
			log.Fatal("readFile(): Error reading in the weight")
		}

		graph.AddEdge(v1, v2, weight)
		edgereadCount++
	}

	return graph, source
}

/*
 */
func (graph *Graph) PrintDistanceAndPaths(source int) {
	// NOTE: Sorting the map keys here bc Go doesn't auto sort the map :(
	sortedKeys := make([]int, 0)
	for i := range graph.nodes {
		sortedKeys = append(sortedKeys, i)
	}
	sort.Ints(sortedKeys)

	// Loop over the sorted key and output node info
	// Traversing backwards through each node to get their paths
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

		// loop here through prev nodes to build the path
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

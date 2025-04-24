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
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
  "container/heap"
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
func NewGraph(vertexCount int) *Graph {
	graph := &Graph{
		nodes: make(map[int]*Node),
		edges: make(map[int][]*Edge),
	}

	for i := range vertexCount {
		graph.edges[i] = []*Edge{}
	}

	return graph
}

func (graph *Graph) AddNode(value int) {
	_, exists := graph.nodes[value]
	if !exists {
		node := &Node{
			value:    value,
			distance: 0,
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
		weight: 0,
	}

	// Append to respective vertex Edge slice.
	graph.edges[v1] = append(graph.edges[v1], forward)
	graph.edges[v2] = append(graph.edges[v2], backward)
}

func outputShortestPath() {}

func dijkstras() {
	/*
			* TODO:
		  * - [ ] shortest distance from source to all other vertices (including source->source i guess)
		  *     - If unreachable, print INF
		  * - [ ] save the actual path of nodes from the source to target. Output this path
	*/

	/*
	 * Visited -- if node has been visited yet
	 * SavedPath -- tracks the path to each node from the source
	 *    - Also tracks the total distance count (number of nodes in the path)
	 */
	visited := make(map[int]bool)
	savedPath := make(map[int]int)

	log.Print(visited, savedPath)

  // NOTE: again, this is just instantiating the Priority Queue using the heap interface
  priorityQueue := make(PriorityQueue, 0)
  heap.Init(&priorityQueue)
}

/*
 * readFile() -- Read in the file based on modified DIMACS format
 * Ending value is the source vertex
 */
func readFile(fileName string) (*Graph, int) {
	file, err := os.Open(fileName)
	if err != nil {
		log.Fatal("Error opening the file")
	}

	defer file.Close()

	scanner := bufio.NewScanner(file)
	if !scanner.Scan() {
		log.Fatal("readFile(): Issue scanning the file -- may be empty..")
	}

	// First row (2 values) -> vertex count and then the edge count.
	firstLine := strings.Fields(scanner.Text())
	if len(firstLine) < 2 || len(firstLine) > 2 {
		log.Fatal(
			"readFile(): Error parsing the first line -- either too few arguments or too many. Should only include vertex count and edge count",
		)
	}

	vertexCount, err := strconv.Atoi(firstLine[0])
	if err != nil {
		log.Fatal("readFile(): Error reading in the vertexCount")
	}

	edgeCount, err := strconv.Atoi(firstLine[1])
	if err != nil {
		log.Fatal("readFile(): Error reading in the edgeCount")
	}

	graph := NewGraph(vertexCount)
	source := 0
	edgereadCount := 1

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

		// Add the nodes and the edges (both forward and backward)
		graph.AddNode(v1)
		graph.AddNode(v2)
		graph.AddEdge(v1, v2, weight)
		edgereadCount++
	}

	fmt.Println("-----NODES-----")
	for key, val := range graph.nodes {
		fmt.Println("Node ----- ", key)
		fmt.Println("     Val: ", val)
	}

	fmt.Println("-----------------------")

	// Print, remove any empty verticies
	// TODO: do this in a less lame way -- might accidentally remove vertices that are valid
	for key, val := range graph.edges {
		if len(val) == 0 {
			delete(graph.edges, key)
			continue
		}

		fmt.Println("Key: ", key)

		for _, w := range val {
			fmt.Println(" Edge to: ", w.to)
		}
	}

	return graph, source
}

func main() {
	if len(os.Args) < 2 {
		log.Fatal("usage: ./bin/main filename")
	}

	// read in the graph
	fileName := os.Args[1]
	// graph, source := readFile(fileName)
	_, source := readFile(fileName)
	fmt.Print("Source: ", source)
}

// Zachary Perry
// ECE 569 -- Computer Networks Assignment 4
// 4/24/25
package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
)

// Graph struct will represent all edges.
// Key in the map       -> vertex value.
// Val for each vertex  -> list of edges.
type Graph struct {
	edges map[int][]*Edge
}

// Edge struct represents an individual edge.
// to       -> destination vertex, where this edge goes to.
// capacity -> weight of the edge, capacity that can flow through it.
// flow     -> what is currently flowing through the edge.
type Edge struct {
	to       int
	capacity int
	flow     int
}

// NewGraph() just creates a new graph instance.
// The map will hold vertex values as the keys and their outgoing edges in a slice as the val.
// NOTE: the edges slice for each vertex will also contain any backwards edges.
func NewGraph(vertexCount int) *Graph {
	graph := &Graph{
		edges: make(map[int][]*Edge),
	}

	for i := range vertexCount {
		graph.edges[i] = []*Edge{}
	}

	return graph
}

// AddEdge() will add an edge from v1 to v2 with the specified weight as the capacity to the graph.
// Additionally, it also adds a backwards edge (with 0 capacity).
func (graph *Graph) AddEdge(v1, v2, weight int) {
	// Forward edge (v1 -> v2).
	forward := &Edge{
		to:       v2,
		capacity: weight,
		flow:     0,
	}

	// Backward edge (v2 -> v1).
	backward := &Edge{
		to:       v1,
		capacity: 0,
		flow:     0,
	}

	// Append to respective vertex Edge slice.
	graph.edges[v1] = append(graph.edges[v1], forward)
	graph.edges[v2] = append(graph.edges[v2], backward)
}

func dijkstras() {}

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

		// Create and add edge to the graph (both forward and backward)
		graph.AddEdge(v1, v2, weight)
		edgereadCount++
	}

	// Print, remove any empty verticies
	for key, val := range graph.edges {
		if len(val) == 0 {
			delete(graph.edges, key)
			continue
		}

		fmt.Println("Key: ", key)

		for _, w := range val {
			fmt.Println("Edge to: ", w.to)
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
	//graph, source := readFile(fileName)
	_, source := readFile(fileName)
	fmt.Print("Source: ", source)
}

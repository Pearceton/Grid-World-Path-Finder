# Path Finder in Grid World

## Project Description

This project implements a path-finding agent in a 50×50 grid world using four graph search algorithms:

* Breadth-First Search (BFS)
* Depth-First Search (DFS)
* Greedy Best-First Search (GBFS)
* A* Search

The agent moves from a source point to a destination point while navigating around obstacles and minimizing travel cost.

---

## Environment Description

The grid world contains two types of polygon obstacles:

### Enclosures (Black)

* Represent forbidden areas.
* The agent cannot enter or touch enclosure boundaries.

### Turfs (Green)

* Traversable regions.
* Movement through turf increases action cost.

---

## Movement Rules

The agent may move in four directions only:

1. Up
2. Right
3. Down
4. Left

Children are expanded strictly in this order.

---

## Action Cost Function

The action cost depends only on the destination point (p'):

* Cost = **1** if outside all turfs
* Cost = **1.5** if inside or on the edge of a turf

BFS and DFS ignore action costs during search and effectively minimize number of steps.

---

## Project Structure

```
project/
│
├── search.py          # Main program (run this file)
├── algorithms.py      # Search algorithm implementations
├── grid.py            # Grid drawing and Point class
├── utils.py           # Stack, Queue, PriorityQueue, Node
├── TestingGrid/
│   ├── world1_enclosures.txt
│   └── world1_turfs.txt
├── summary.txt        # Required results output
└── README.txt
```

---

## Requirements

* Python **3.11**
* matplotlib
* numpy

Install required packages:

```
pip install matplotlib numpy
```

---

## How to Run

From the project root directory:

```
python3.11 search.py
```

---

## Output

The console prints:

```
Algorithm : Path Cost, Nodes Expanded
```

Example:

```
BFS: 86.5 2029
DFS: 834.0 1250
GBFS: 93.0 103
A*: 85.0 1777
```
And then you choose what algorithm to use to plot.

---

# Dynamic Pathfinding Agent - Assignment Report

**Course:** Artificial Intelligence  
**Assignment:** 2 - Dynamic Pathfinding Agent  
**Student ID:** 23F-0550  
**Name:** Ahmed Rohan  
**Section:** 6-A  
**Date:** March 2026

---

## Table of Contents
1. [Introduction](#1-introduction)
2. [Implementation Logic](#2-implementation-logic)
3. [Algorithm Comparison](#3-algorithm-comparison)
4. [Test Cases](#4-test-cases)
5. [Conclusion](#5-conclusion)

---

## 1. Introduction

This project implements a Dynamic Pathfinding Agent capable of navigating a grid-based environment using informed search algorithms. The agent can handle dynamically appearing obstacles and perform real-time path re-planning.

### Features Implemented:
- Dynamic grid with configurable dimensions
- Random map generation with adjustable obstacle density
- Interactive map editor (click to place/remove walls)
- Two search algorithms: GBFS and A*
- Two heuristic functions: Manhattan and Euclidean distance
- Dynamic mode with obstacle spawning and re-planning
- Real-time visualization and metrics dashboard

---

## 2. Implementation Logic

### 2.1 Heuristic Functions

#### Manhattan Distance
```
D_manhattan = |x₁ - x₂| + |y₁ - y₂|
```

**Implementation:**
```python
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
```

- Best suited for 4-directional movement grids
- Always admissible (never overestimates)
- Provides tighter bound than Euclidean for grid movement

#### Euclidean Distance
```
D_euclidean = √((x₁ - x₂)² + (y₁ - y₂)²)
```

**Implementation:**
```python
def euclidean(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
```

- Represents true straight-line distance
- Admissible for any movement pattern
- Less tight bound for 4-way grids (explores more nodes)

---

### 2.2 Greedy Best-First Search (GBFS)

**Evaluation Function:** `f(n) = h(n)`

GBFS uses only the heuristic estimate to guide the search, always expanding the node that appears closest to the goal.

**Key Implementation Details:**
1. Uses a priority queue (min-heap) ordered by h(n)
2. Maintains a `visited` set for explored nodes
3. Uses `came_from` dictionary for path reconstruction
4. Does NOT track g(n) - path cost is ignored

**Pseudocode:**
```
1. Initialize open_set with start node, priority = h(start)
2. While open_set is not empty:
   a. Pop node with lowest h(n)
   b. If node is goal → reconstruct and return path
   c. Mark node as visited
   d. For each neighbor not visited:
      - Add to open_set with priority h(neighbor)
3. Return failure (no path)
```

---

### 2.3 A* Search

**Evaluation Function:** `f(n) = g(n) + h(n)`

Where:
- `g(n)` = actual cost from start to node n
- `h(n)` = heuristic estimate from n to goal

A* combines path cost with heuristic estimate, guaranteeing an optimal path when using an admissible heuristic.

**Key Implementation Details:**
1. Priority queue ordered by f(n) = g(n) + h(n)
2. Maintains `g_score` dictionary for actual path costs
3. Updates path if better route found to a node
4. Uses `came_from` dictionary for path reconstruction

**Pseudocode:**
```
1. Initialize open_set with start, g_score[start] = 0
2. While open_set is not empty:
   a. Pop node with lowest f(n) = g(n) + h(n)
   b. If node is goal → reconstruct and return path
   c. Mark node as visited
   d. For each neighbor:
      - Calculate tentative_g = g_score[current] + 1
      - If tentative_g < g_score[neighbor]:
        * Update g_score[neighbor]
        * Update came_from[neighbor]
        * Add to open_set with priority f(neighbor)
3. Return failure (no path)
```

---

### 2.4 Dynamic Obstacle Re-planning

When dynamic mode is enabled:

1. **Obstacle Spawning:** At each agent movement step, a random obstacle may appear with configurable probability (1-15%)

2. **Path Validation:** Before each move, the agent checks if the next position is still valid

3. **Re-planning Trigger:** If path is blocked:
   - Run search algorithm from current position to goal
   - Replace current path with new path
   - Increment replan counter

4. **Efficiency:** Only replans when necessary (obstacle on current path), not on every spawn

---

## 3. Algorithm Comparison

### 3.1 Greedy Best-First Search (GBFS)

| Aspect | Analysis |
|--------|----------|
| **Completeness** | Yes (in finite graphs) |
| **Optimality** | NO - may find suboptimal paths |
| **Time Complexity** | O(b^m) worst case |
| **Space Complexity** | O(b^m) |

**Pros:**
- ✅ Very fast when heuristic is accurate
- ✅ Explores fewer nodes in favorable cases
- ✅ Lower memory usage
- ✅ Simple implementation

**Cons:**
- ❌ Not guaranteed to find shortest path
- ❌ Can be misled by obstacles
- ❌ May explore suboptimal routes
- ❌ Performance depends heavily on heuristic quality

---

### 3.2 A* Search

| Aspect | Analysis |
|--------|----------|
| **Completeness** | Yes |
| **Optimality** | YES (with admissible heuristic) |
| **Time Complexity** | O(b^d) |
| **Space Complexity** | O(b^d) |

**Pros:**
- ✅ Guaranteed optimal path
- ✅ Complete - finds solution if exists
- ✅ Optimally efficient - no algorithm expands fewer nodes
- ✅ Well-suited for pathfinding applications

**Cons:**
- ❌ Higher memory usage than GBFS
- ❌ May explore more nodes than GBFS
- ❌ Slower in some scenarios
- ❌ Requires admissible heuristic for optimality

---

### 3.3 Heuristic Comparison

| Heuristic | Best For | Nodes Expanded | Accuracy |
|-----------|----------|----------------|----------|
| Manhattan | 4-way grids | Fewer | Tight bound |
| Euclidean | 8-way/free movement | More | Loose bound |

---

## 4. Test Cases

### 4.1 GBFS Test Cases

#### Best Case Scenario
**Setup:** Clear path with minimal obstacles between start and goal

[INSERT SCREENSHOT HERE]

**Observations:**
- Nodes Visited: [X]
- Path Cost: [X]
- Time: [X] ms
- GBFS quickly finds path by following heuristic directly to goal

#### Worst Case Scenario
**Setup:** Obstacles blocking direct path, forcing exploration

[INSERT SCREENSHOT HERE]

**Observations:**
- Nodes Visited: [X]
- Path Cost: [X]
- Time: [X] ms
- GBFS explores many nodes but may find suboptimal path

---

### 4.2 A* Test Cases

#### Best Case Scenario
**Setup:** Clear path with minimal obstacles

[INSERT SCREENSHOT HERE]

**Observations:**
- Nodes Visited: [X]
- Path Cost: [X]
- Time: [X] ms
- A* finds optimal path efficiently

#### Worst Case Scenario
**Setup:** Complex maze requiring extensive exploration

[INSERT SCREENSHOT HERE]

**Observations:**
- Nodes Visited: [X]
- Path Cost: [X]
- Time: [X] ms
- A* explores more nodes but guarantees optimal solution

---

### 4.3 Dynamic Mode Test

**Setup:** Start with clear path, enable dynamic obstacles

[INSERT SCREENSHOT HERE]

**Observations:**
- Initial Path Cost: [X]
- Replans Required: [X]
- Final Path Cost: [X]
- Agent successfully navigates around dynamically spawned obstacles

---

## 5. Conclusion

This project successfully implements a Dynamic Pathfinding Agent with both GBFS and A* search algorithms. Key findings:

1. **GBFS** is faster but may not find optimal paths
2. **A*** guarantees optimal paths but may be slower
3. **Manhattan heuristic** is more efficient for 4-directional grids
4. **Dynamic re-planning** effectively handles runtime obstacles

The interactive GUI allows users to compare algorithms visually and understand the trade-offs between speed and optimality in pathfinding applications.

---

## References

1. Russell, S., & Norvig, P. (2020). Artificial Intelligence: A Modern Approach (4th ed.)
2. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE Transactions.

---

*End of Report*

# Dynamic Pathfinding Agent

**Student:** Ahmed Rohan  
**Roll No:** 23F-0550  
**Section:** 6-A  
**Course:** Artificial Intelligence - Assignment 2

A simple GUI-based pathfinding visualization tool implementing **GBFS** and **A\*** search algorithms with dynamic obstacles.

## Features

- **GBFS**: `f(n) = h(n)` - Greedy, uses heuristic only
- **A\***: `f(n) = g(n) + h(n)` - Optimal, uses cost + heuristic
- **Heuristics**: Manhattan and Euclidean distance
- **Dynamic Mode**: Random obstacle spawning with real-time re-planning
- **Interactive**: Click to add walls, set start/goal positions
- **Visualization**: Frontier (yellow), Visited (blue), Path (green)

## Installation

```bash
pip install pygame
```

## Usage

```bash
python search_visualizer.py
```

## Controls

| Input | Action |
|-------|--------|
| Left-click | Toggle wall |
| Right-click | Set start |
| Middle-click | Set goal |
| ESC | Exit |

## GUI Controls

- **GBFS / A\***: Select algorithm
- **Manhat / Euclid**: Select heuristic  
- **Dynamic**: Toggle obstacle spawning during traversal
- **Run**: Start pathfinding
- **Reset**: Clear visualization
- **Random Map**: Generate random obstacles
- **Clear**: Remove all walls
- **Density slider**: Set obstacle percentage
- **Spawn slider**: Set dynamic spawn rate

## Metrics

- Nodes Visited
- Path Cost
- Execution Time (ms)
- Replans (dynamic mode)

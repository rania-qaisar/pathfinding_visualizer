# Pathfinding Algorithm Visualizer

A grid-based pathfinding visualizer implementing 6 classic search algorithms with real-time animation, best/worst case scenario testing, and a live performance stats panel.

## Features
- 6 search algorithms: BFS, DFS, UCS, DLS, IDDFS, and Bidirectional Search
- 8-directional movement including diagonals
- Best case and worst case grid generators for each algorithm
- Live animation with real-time frontier and explored node tracking
- Stats panel showing steps, explored nodes, path length, and elapsed time
- Random maze generator and clear grid option

## Requirements
- Python 3.x
- matplotlib
- numpy

## How to Use
- Click any **algorithm button** to run it on the current maze
- Click **BEST** next to any algorithm for a best case scenario
- Click **WORST** next to any algorithm for a worst case scenario
- Click **New Maze** to generate a fresh random maze
- Click **Clear** to reset to an empty grid

## Algorithms
| Algorithm | Strategy |
|-----------|----------|
| BFS | Explores level by level, guarantees shortest path |
| DFS | Explores deep first, fast but not always optimal |
| UCS | Expands by lowest cost, diagonal moves cost 1.414 |
| DLS | Depth-limited DFS with limit = 12 |
| IDDFS | Repeated DLS with increasing depth limits |
| Bidirectional | Simultaneous search from start and goal |

## Color Guide
| Color | Meaning |
|-------|---------|
| Green | Start node (S) |
| Blue | Target node (T) |
| Red | Wall / obstacle |
| Light Blue | Explored node |
| Yellow | Current frontier |
| Orange | Final path found |

---
Made by Rania Qaisar

## Global planning algorithms {#planning_algorithms}

[TOC]

This page describes the search algorithms behind the global planners (A*, Dijkstra, Lazy Theta*, RRT, RRT-Connect, RRT*): what they do, how a map is turned into something they can search, which parts can be swapped, and how they compare. For where a global planner sits in the system, see @ref outline and `path3d::GlobalPlanner`.

### Three layers {#planning_layers}

A global planner is built from three layers. Each layer knows only the one below it, so an algorithm never sees a map type and a map never sees an algorithm.

@dot
digraph layers {
  rankdir=TB; nodesep=0.5; ranksep=0.45;
  node [shape=box, style="rounded,filled", fillcolor="#e8f0fe", color="#5b7bb5", fontname="Helvetica", fontsize=10, width=3.4];
  edge [fontname="Helvetica", fontsize=9, color="#555555"];
  thread [label="GlobalPlanningThread", shape=ellipse, fillcolor="#f1f1f1", color="#999999"];
  planner [label=<Planner<BR/><FONT POINT-SIZE="9" COLOR="#666666">AStarGlobalPlanner, LazyThetaStarGlobalPlanner, Rrt*GlobalPlanner</FONT>>];
  adapter [label=<Map adapter<BR/><FONT POINT-SIZE="9" COLOR="#666666">GridGraph, OccupancySpace</FONT>>];
  algorithm [label=<Algorithm<BR/><FONT POINT-SIZE="9" COLOR="#666666">a_star, dijkstra, lazy_theta_star, rrt, rrt_connect, rrt_star</FONT>>];
  thread -> planner [label=" ReplanRequest  /  GlobalPath"];
  planner -> adapter [label=" map copy, search box, settings"];
  adapter -> algorithm [label=" SearchGraph  /  SamplingSpace"];
}
@enddot

| Layer | Files | Job |
|---|---|---|
| Planner | `planning/search_planners.hpp` | Talks to the system: takes a `path3d::ReplanRequest`, builds the search space on its map copy, runs the algorithm, and returns a `path3d::GlobalPath` (always: `Failed` when there is no path). |
| Map adapter | `planning/algorithms/map_adapters.hpp` | Turns a map into what an algorithm can search: `path3d::GridGraph` (voxels and their 26 neighbors) for A*, `path3d::OccupancySpace` (points and segments in meters) for RRT and RRT*. Holds the safety rules: margin around the drone, search box, unknown space. |
| Algorithm | `planning/algorithms/a_star.hpp`, `lazy_theta_star.hpp`, `rrt.hpp`, `rrt_connect.hpp`, `rrt_star.hpp`, `heuristics.hpp` | Pure search. Constrained only by the concepts `path3d::SearchGraph` and `path3d::SamplingSpace`, so they can be tested on tiny made-up worlds. |

### The algorithms {#planning_algorithm_list}

| Algorithm | Function | Idea | Strong side | Weak side |
|---|---|---|---|---|
| A* | `path3d::a_star` | Expands the most promising voxel first, ordered by f = g + h: cost so far plus an estimate of the cost left. | Shortest path on the grid; deterministic; fast with a good heuristic. | Work grows with the volume (half the voxel size, 8 times the voxels); the path follows the grid (45 degree turns). |
| Dijkstra | `path3d::dijkstra` | A* with h = 0. | Same shortest path. | Searches in every direction; the baseline that shows what the heuristic saves. |
| RRT | `path3d::rrt` | Grows a random tree from the start until it reaches the goal. | Finds some path very fast; works in continuous space. | Long, crooked paths; different for every seed. |
| RRT* | `path3d::rrt_star` | RRT that picks the cheapest parent for each new node and rewires its neighbors through it. | Paths get close to the shortest. | Slower than RRT; quality depends on the number of iterations. |
| RRT-Connect | `path3d::rrt_connect` | Two RRT trees, from the start and from the goal, that grow towards each other. | Usually the fastest to find a path. | As crooked as RRT. |
| Lazy Theta* | `path3d::lazy_theta_star` | A* where a node may link straight to its neighbor's parent when the line between them is free; the line is checked only when the node is expanded ("lazy"). | Shortest paths of all, at any angle, with the fewest waypoints. | Line checks make it slower than A* on large searches. |

#### Heuristics {#planning_heuristics}

The heuristic h estimates the cost from a voxel to the goal. A* returns the shortest path only if h never overestimates (an *admissible* heuristic); the closer h is to the true cost, the fewer voxels A* expands.

| Heuristic | Type | On the 26-connected grid |
|---|---|---|
| Octile | `path3d::OctileHeuristic` | Exact cost of an empty grid: space diagonals, then face diagonals, then straight steps. Admissible and the tightest, so the default. |
| Euclidean | `path3d::EuclideanHeuristic` | Straight-line distance. Admissible, but below the grid cost, so A* expands more. |
| Manhattan | `path3d::ManhattanHeuristic` | Sum of the axis differences. **Overestimates** with diagonal steps: A* is much faster but the path can be longer. |
| Zero | `path3d::ZeroHeuristic` | Always 0: turns A* into Dijkstra. |

Lazy Theta* uses Euclidean by default: its paths are not tied to the grid, so they can be shorter than the Octile estimate, which would then overestimate.

### From a map to a search space {#planning_adapters}

The map (`path3d::LogOddsGridMap`) answers only "is this voxel Free, Occupied or Unknown". The adapters add what a planner for a real drone needs, configured with `path3d::SearchSpaceConfig`:

| Setting | Default | Meaning |
|---|---|---|
| `margin` | 0.5 m | Free space needed around the drone. A voxel (or point) is free only if the ball of this radius around it is. |
| `unknown_is_free` | true | The map is built while flying, so most of it is Unknown. True plans through unknown space, and the local planner asks for a new path when it turns out blocked. False plans only through space seen free, which usually finds nothing at the start. |
| `unknown_cost` | 1 | `GridGraph` only. A step into an Unknown voxel costs this many times its length, so A* prefers space it has seen. At least 1. |
| `climb_cost` | 0 | `GridGraph` only. Every meter of climbing costs this many extra meters, because climbing takes more energy. With a high value A* flies around an obstacle instead of over it. At least 0. |

Both costs apply to Lazy Theta* too: a straight line is charged `unknown_cost` for the share of its voxels that are Unknown, and `climb_cost` for its climb.
| `check_step` | 0 | `OccupancySpace` only. RRT checks a segment point by point, this far apart. 0 picks half a voxel on a grid map, 0.1 m otherwise. |

The two costs keep every heuristic above admissible, because they only make steps more expensive.

The planners add `path3d::SearchPlannerConfig`:

| Setting | Default | Meaning |
|---|---|---|
| `padding` | 10 m | The search box is the box around the drone and the goal, grown by this much (see `path3d::search_bounds`). The map has no bounds; without a box, a search with no path would never end. |
| `min_altitude`, `max_altitude` | 1 m, 30 m | Height limits of the search box. Start and goal always stay inside. |
| `max_expansions` | 2 000 000 | A* gives up after expanding this many voxels. |

The planner keeps only the turning points of the A* path: the waypoints are the drone position, the centers of the voxels where the direction changes, and the goal.

### Swapping maps, heuristics and algorithms {#planning_combinations}

The map and the heuristic are template parameters, the algorithm is the planner type:

@code{.cpp}
using Map = LogOddsGridMap;

AStarGlobalPlanner<Map> a_star(map, uav, goal);                            // A* with the Octile heuristic
AStarGlobalPlanner<Map, EuclideanHeuristic> euclidean(map, uav, goal);     // another heuristic
AStarGlobalPlanner<Map, ZeroHeuristic> dijkstra(map, uav, goal);           // Dijkstra
LazyThetaStarGlobalPlanner<Map> theta(map, uav, goal);                     // any-angle paths
RrtGlobalPlanner<Map> rrt(map, uav, goal);                                 // RRT
RrtConnectGlobalPlanner<Map> connect(map, uav, goal);                      // RRT-Connect
RrtStarGlobalPlanner<Map> rrt_star(map, uav, goal, config, RrtStarParams{.max_iterations = 3000});
@endcode

Each planner is a `path3d::GlobalPlanner`, so it plugs into `path3d::GlobalPlanningThread` exactly like `path3d::LineGlobalPlanner` in the @ref tutorial_line. Each one also reports `last_stats()` (`path3d::PlanStats`): whether it found a path, its length and climb, and how much it explored.

**A new map** works with these planners without changes when it satisfies the concepts of `mapping/map_view.hpp`: `path3d::ReplicatedMap` and `path3d::GridMap` for A*, `path3d::ReplicatedMap` and `path3d::OccupancyMap` for RRT and RRT*.

**A new algorithm** is written against `path3d::SearchGraph` (graph search) or `path3d::SamplingSpace` (sampling), tested on small worlds like the ones in `test/search_algorithms_test.cpp`, and then wrapped in a planner like the ones in `search_planners.hpp`.

### Comparison {#planning_benchmark}

`apps/planner_benchmark.cpp` runs every planner on every scene of `path3d::benchmark_scenes()` and prints a table per scene. No simulation is needed:

@code{.sh}
./scripts/run_benchmark.sh                    # inside the container; builds first
./scripts/run_benchmark.sh -n results.csv 0.25   # no build, own CSV path, 0.25 m voxels
@endcode

It writes every row to `benchmark.csv` (scene, planner, runs, found, length_m, climb_m, time_ms, explored, waypoints). RRT and RRT* run with 5 seeds and report averages over the runs that found a path.

The scenes are made of boxes, and each one asks for something different in 3D:

| Scene | What it tests |
|---|---|
| `open_field` | A few pillars. The control case: the path can stay at one height. |
| `wall` | A 6 m wall that is shorter to fly over than around. With a climb cost, A* goes around. |
| `window` | A 20 m wall with one opening at 8-12 m. The only way is up through it. |
| `low_passage` | A 20 m wall with a 3 m high gap to the side. The drone has to go down. |
| `city_ceiling` | 4 x 4 blocks of 10 m and 25 m buildings under a 15 m ceiling: over the low ones, between the high ones. |

The scene map holds only the obstacles; the rest is Unknown and treated as free, so every planner sees the whole world, as if it had already been mapped.

What the first runs show (0.5 m voxels; times depend on the computer):

- **The heuristic matters most for time.** In `city_ceiling` A* with Octile expands about 72 000 voxels in 0.36 s, Dijkstra about 670 000 in 5.8 s, for the same path length.
- **Manhattan is fast but not shortest.** In `city_ceiling` it expands about 200 voxels in 3 ms, but its path is about 4 m longer than the shortest.
- **The search is really 3D.** In `wall` the shortest path climbs over the wall (4.75 m of climb); with `climb_cost = 2` it goes around instead (0.25 m of climb, 3 m longer). In `window` and `low_passage` every planner has to change height to get through.
- **Lazy Theta* gives the shortest paths.** In every scene it is shorter than A* (107 m against 113 m in `city_ceiling`) and needs the fewest waypoints (8 against 32), but in the city it takes about 1.7 s against 0.35 s for A*.
- **RRT and RRT-Connect are the fastest and the worst.** They find a path in a few milliseconds, but 20-50 % longer than A*'s, wandering up and down. RRT* closes most of that gap (6-20 % longer than A*) in 35-70 ms per plan.

Small differences in length between the A* variants come from the waypoints: paths of equal grid cost can have different turning points. A climb of 0.25 m on a flat path is the start and goal sitting half a voxel below the voxel centers.

### Limitations {#planning_limitations}

- Every replan is a new search from scratch. An incremental algorithm (D* Lite) would reuse the previous search.
- A* paths follow the grid; Lazy Theta* straightens them, but neither is smooth: a path is still a polyline with sharp turns.
- Unknown space is free by default, so a plan can go where the camera has never looked, most often above or below the drone. The local planner has to catch that.
- RRT and RRT* search the nearest node by checking every node of the tree, which gets slow for large trees.

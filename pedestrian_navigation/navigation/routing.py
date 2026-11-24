"""Small static-map A* guide, shared by both future planner variants.

Pedestrians are deliberately excluded. The local planner still checks all
current obstacles; the static guide prevents a symmetric local minimum.
"""
import heapq
import math
from .core import Parameters, segment_distance


def static_route(start, goal, obstacles, half_width, p: Parameters):
    margin = p.safety_margin + p.route_buffer
    def visible(a, b):
        if max(abs(a[1]), abs(b[1])) > half_width-p.robot_radius-margin:
            return False
        return all(segment_distance((a[0]-c.x, a[1]-c.y), (b[0]-c.x, b[1]-c.y))
                   >= c.radius+p.robot_radius+margin for c in obstacles)
    if visible(start, goal):
        return [goal]
    resolution = 0.2
    limit_y = math.floor((half_width-p.robot_radius-margin)/resolution)
    grid = {(i, j): (i*resolution, j*resolution)
            for i in range(-18, 19) for j in range(-limit_y, limit_y+1)
            if visible((i*resolution, j*resolution), (i*resolution, j*resolution))}
    def nearest(point):
        reachable = [key for key, value in grid.items() if visible(point, value)]
        if not reachable:
            raise ValueError('Static route endpoint has no free connection to the grid.')
        return min(reachable, key=lambda key: math.dist(point, grid[key]))
    first, last = nearest(start), nearest(goal)
    costs, parent = {first: 0.0}, {}
    queue = [(math.dist(grid[first], goal), 0.0, first)]
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs[current]:
            continue
        if current == last:
            path = [current]
            while path[-1] != first:
                path.append(parent[path[-1]])
            points = [start] + [grid[key] for key in reversed(path)] + [goal]
            # Greedy visibility smoothing, preserving the inflated-map clearance.
            route, index = [], 0
            while index < len(points)-1:
                next_index = next(j for j in range(len(points)-1, index, -1)
                                  if visible(points[index], points[j]))
                route.append(points[next_index])
                index = next_index
            return route
        for dx, dy in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
            neighbor = current[0]+dx, current[1]+dy
            if neighbor not in grid or not visible(grid[current], grid[neighbor]):
                continue
            tentative = cost + math.dist(grid[current], grid[neighbor])
            if tentative < costs.get(neighbor, math.inf):
                costs[neighbor], parent[neighbor] = tentative, current
                heapq.heappush(queue, (tentative+math.dist(grid[neighbor], goal), tentative, neighbor))
    raise ValueError('No static route found in the development walkway.')

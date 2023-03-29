import heapq

# Function to calculate the Euclidean distance between two points
def euclidean_distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

# Function to get the neighboring cells of a cell
def get_neighbors(cell):
    x, y = cell
    return [(x+i, y+j) for i in range(-1, 2) for j in range(-1, 2) if i != 0 or j != 0]

# Function to get the shortest path from start to end using A* algorithm
def get_shortest_path(start, end, grid):
    heap = []
    closed_set = set()
    parents = {}
    g_scores = {start: 0}
    f_scores = {start: euclidean_distance(start, end)}
    heapq.heappush(heap, (f_scores[start], start))

    while heap:
        current = heapq.heappop(heap)[1]
        if current == end:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            path.reverse()
            return path

        closed_set.add(current)
        for neighbor in get_neighbors(current):
            if neighbor in closed_set or grid[neighbor[0]][neighbor[1]] != 0:
                continue

            tentative_g_score = g_scores[current] + 1
            if neighbor not in g_scores or tentative_g_score < g_scores[neighbor]:
                parents[neighbor] = current
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = tentative_g_score + euclidean_distance(neighbor, end)
                heapq.heappush(heap, (f_scores[neighbor], neighbor))

    return None

# Function to get the time required for a drone to reach its destination
def get_time_to_destination(start, end):
    return euclidean_distance(start, end)

# Function to get the minimum time for a drone to start moving towards its destination
def get_min_start_time(drone, other_drones):
    min_time = 0
    for other_drone in other_drones:
        time_to_reach = get_time_to_destination(other_drone[0:2], other_drone[2:4])
        time_diff = other_drone[4] - drone[4]
        if time_diff >= time_to_reach:
            continue
        if time_diff < time_to_reach and min_time < time_to_reach - time_diff:
            min_time = time_to_reach - time_diff
    return drone[4] + min_time

# Main function to get the paths for all drones
def get_all_drone_paths(drones, grid):
    max_end_time = max(drone[4] + get_time_to_destination((drone[0], drone[1]), (drone[2], drone[3])) for drone in drones)
    for drone in drones:
        grid[drone[0]][drone[1]] = drone[4] + 1

    paths = [[] for _ in range(len(drones))]
    for t in range(min(drone[4] for drone in drones), int(max_end_time) + 1):
        for i, drone in enumerate(drones):
            if drone[4] <= t < drone[4] + get_time_to_destination((drone[0], drone[1]), (drone[2], drone[3])):
                x, y = drone[0], drone[1]
                neighbors = get_neighbors((x, y))
                valid_neighbors = [(nx, ny) for nx, ny in neighbors if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] <= t]
            end_pos = (drone[2], drone[3])
            if end_pos in valid_neighbors:
                paths[i].append((end_pos, t))
                grid[x][y] = 0
            else:
                min_start_time = get_min_start_time(drone, [d for j, d in enumerate(drones) if i != j])
                if t < min_start_time:
                    continue
                path = get_shortest_path((x, y), end_pos, grid)
                if path:
                    paths[i].append((path[-1], t))
                    for cell in path[-2::-1]:
                        paths[i].append((cell, t))
                        grid[cell[0]][cell[1]] =t
                        return paths
                        # Example input
drones = [[0, 0, 3, 3, 0], [5, 5, 1, 1, 5], [2, 2, 4, 4, 2]]
grid = [[0 for _ in range(10)] for _ in range (10)]
all_paths = get_all_drone_paths( drones, grid)
for i, path in enumerate(all_paths):
    print(f"Drone {i+1} path:{path}")
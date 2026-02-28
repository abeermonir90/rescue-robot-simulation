from enum import Enum
import time
import heapq

# =========================
# Environment
# =========================
class CellType(Enum):
    EMPTY = 0
    OBSTACLE = 1
    VICTIM = 2
    SAFE = 3

class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[CellType.EMPTY for _ in range(width)] for _ in range(height)]
        self.safe_zone = None       # (x, y)
        self.victims = set()        # set of (x, y)

    def is_within_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def cell(self, x, y):
        return self.grid[y][x]

    def is_obstacle(self, x, y):
        return self.cell(x, y) == CellType.OBSTACLE

    def is_victim(self, x, y):
        return (x, y) in self.victims

    def is_safe(self, x, y):
        return self.safe_zone == (x, y)

    def set_obstacle(self, x, y):
        if self.is_within_bounds(x, y):
            self.grid[y][x] = CellType.OBSTACLE

    def set_safe_zone(self, x, y):
        if self.is_within_bounds(x, y):
            self.grid[y][x] = CellType.SAFE
            self.safe_zone = (x, y)

    def add_victim(self, x, y):
        if self.is_within_bounds(x, y) and not self.is_obstacle(x, y):
            self.victims.add((x, y))
            self.grid[y][x] = CellType.VICTIM

    def remove_victim(self, x, y):
        if (x, y) in self.victims:
            self.victims.remove((x, y))
            # If it's not safe zone, set to empty
            if not self.is_safe(x, y):
                self.grid[y][x] = CellType.EMPTY

    def neighbors4(self, x, y):
        for dx, dy in [(0,-1), (1,0), (0,1), (-1,0)]:
            nx, ny = x + dx, y + dy
            if self.is_within_bounds(nx, ny) and not self.is_obstacle(nx, ny):
                yield (nx, ny)

    def display(self, robot=None, path=None):
        path_set = set(path) if path else set()
        for y in range(self.height):
            for x in range(self.width):
                if robot and robot.x == x and robot.y == y:
                    print("R", end=" ")
                elif (x, y) in path_set and not self.is_safe(x, y) and not self.is_victim(x, y):
                    print("*", end=" ")
                elif self.grid[y][x] == CellType.OBSTACLE:
                    print("#", end=" ")
                elif self.grid[y][x] == CellType.VICTIM:
                    print("P", end=" ")
                elif self.grid[y][x] == CellType.SAFE:
                    print("S", end=" ")
                else:
                    print(".", end=" ")
            print()
        print()

# =========================
# Planner (A*)
# =========================
class AStarPlanner:
    @staticmethod
    def heuristic(a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, env: Environment, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0, start))

        came_from = {}
        g_score = {start: 0}
        in_open = {start}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            in_open.discard(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for nb in env.neighbors4(current[0], current[1]):
                tentative_g = g_score[current] + 1
                if nb not in g_score or tentative_g < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    f = tentative_g + self.heuristic(nb, goal)
                    if nb not in in_open:
                        heapq.heappush(open_heap, (f, nb))
                        in_open.add(nb)

        return None

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

# =========================
# Robot
# =========================
class RobotState(Enum):
    SEARCHING = 0
    PLANNING_TO_VICTIM = 1
    MOVING_TO_VICTIM = 2
    PLANNING_TO_SAFE = 3
    CARRYING_TO_SAFE = 4
    FINISHED = 5

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.state = RobotState.SEARCHING
        self.carrying = False

        self.target = None          # victim coordinate (x,y)
        self.path = []              # planned path
        self.path_index = 0

        self.rescued_count = 0

    @staticmethod
    def manhattan(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def choose_next_victim(self, env: Environment):
        if not env.victims:
            return None
        here = (self.x, self.y)
        # pick closest by Manhattan distance (simple + good enough)
        return min(env.victims, key=lambda v: self.manhattan(here, v))

    def move_to(self, env: Environment, nx, ny):
        if not env.is_within_bounds(nx, ny):
            return False
        if env.is_obstacle(nx, ny):
            return False
        self.x, self.y = nx, ny
        return True

    def step_follow_path(self, env: Environment):
        if self.path_index >= len(self.path):
            return False
        nx, ny = self.path[self.path_index]
        if env.is_obstacle(nx, ny):
            return False
        ok = self.move_to(env, nx, ny)
        if ok:
            self.path_index += 1
        return ok

    def decide(self, env: Environment, planner: AStarPlanner):
        # finished condition
        if not env.victims and not self.carrying:
            self.state = RobotState.FINISHED
            return

        if self.state == RobotState.SEARCHING:
            self.target = self.choose_next_victim(env)
            if self.target is None:
                self.state = RobotState.FINISHED
            else:
                self.state = RobotState.PLANNING_TO_VICTIM

        elif self.state == RobotState.PLANNING_TO_VICTIM:
            path = planner.astar(env, (self.x, self.y), self.target)
            if path is None:
                # no path → just finish (or you can skip victim)
                self.state = RobotState.FINISHED
                return
            self.path = path
            self.path_index = 1  # skip current cell
            self.state = RobotState.MOVING_TO_VICTIM

        elif self.state == RobotState.MOVING_TO_VICTIM:
            if (self.x, self.y) == self.target:
                # picked up victim
                self.carrying = True
                env.remove_victim(self.x, self.y)
                self.state = RobotState.PLANNING_TO_SAFE
                return

            ok = self.step_follow_path(env)
            if not ok:
                # re-plan
                self.state = RobotState.PLANNING_TO_VICTIM

        elif self.state == RobotState.PLANNING_TO_SAFE:
            if env.safe_zone is None:
                raise ValueError("Safe zone not set!")
            path = planner.astar(env, (self.x, self.y), env.safe_zone)
            if path is None:
                self.state = RobotState.FINISHED
                return
            self.path = path
            self.path_index = 1
            self.state = RobotState.CARRYING_TO_SAFE

        elif self.state == RobotState.CARRYING_TO_SAFE:
            if env.is_safe(self.x, self.y):
                # drop victim
                self.carrying = False
                self.rescued_count += 1
                self.target = None
                self.state = RobotState.SEARCHING
                return

            ok = self.step_follow_path(env)
            if not ok:
                self.state = RobotState.PLANNING_TO_SAFE

# =========================
# Simulation
# =========================
class Simulation:
    def __init__(self, env, robot, planner, max_steps=500):
        self.env = env
        self.robot = robot
        self.planner = planner
        self.max_steps = max_steps
        self.step_count = 0

    def run(self, delay=0.15, show_path=True):
        while self.step_count < self.max_steps:
            # display
            path_to_show = self.robot.path if show_path else None
            self.env.display(self.robot, path=path_to_show)

            print(
                f"Step: {self.step_count} | State: {self.robot.state.name} | "
                f"Carrying: {self.robot.carrying} | Rescued: {self.robot.rescued_count}"
            )
            print("-" * 60)

            # decide/act
            self.robot.decide(self.env, self.planner)

            if self.robot.state == RobotState.FINISHED:
                break

            self.step_count += 1
            time.sleep(delay)

        print("\nSimulation finished.")
        print(f"Total rescued: {self.robot.rescued_count}")

# =========================
# Main
# =========================
def main():
    env = Environment(width=12, height=10)

    # Obstacles (example layout)
    for y in range(1, 9):
        env.set_obstacle(5, y)
    env.set_obstacle(6, 8)
    env.set_obstacle(7, 8)
    env.set_obstacle(8, 8)

    # Safe zone
    env.set_safe_zone(11, 9)

    # Victims (static)
    env.add_victim(2, 7)
    env.add_victim(9, 2)
    env.add_victim(10, 6)

    robot = Robot(x=0, y=0)
    planner = AStarPlanner()
    sim = Simulation(env, robot, planner, max_steps=500)
    sim.run(delay=0.12, show_path=True)

if __name__ == "__main__":
    main()
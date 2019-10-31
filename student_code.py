from helpers import Map, load_map, show_map
import heapq
import math
import sys

DEBUG = False

class State:
    def __init__(self, id, path_cost, est_distance, path):
        """
        Initialize the state.
        """
        self.id = id
        self.update(path_cost, est_distance, path)

    def update(self, path_cost, est_distance, path):
        """
        Update the state information.
        """
        self.path_cost = path_cost
        self.est_distance = est_distance
        self.total_cost = path_cost + est_distance
        self.path = path

class AStar:
    def __init__(self, intersections=None, roads = None, goal_id = None):
        """
        Initializes A* Search.
        """
        self.explored = set()
        self.frontier = {}
        self.min_pq = []

        self.intersections = intersections
        self.roads = roads
        self.goal_id = goal_id

    def calc_est_distance(self, state_id):
        """
        Calculates the estimated distance between the state with the given state_id and the goal.
        """
        est_distance = self.euclidean_distance(state_id, self.goal_id)
        return est_distance

    def euclidean_distance(self, state_id_1, state_id_2):
        """
        Calculates the  euclidean distance between two states.
        """
        x1, y1 = self.intersections[state_id_1]
        x2, y2 = self.intersections[state_id_2]
        return math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))

    def is_goal(self, state):
        """
        Checks if the state with the given state_id is the goal.
        """
        return self.goal_id == state.id

    def is_valid(self, state):
        """
        Checks if a state is a valid state.

        A valid state is a state that is not yet explored (i.e. on the frontier) and 
        that has the most up to date total cost.
        """
        not_explored = state.id not in self.explored
        up_to_date = state.total_cost == self.frontier[state.id].total_cost
        return not_explored and up_to_date
    
    def get_new_state(self, state, new_state_id):
        new_path_cost = state.path_cost + self.euclidean_distance(state.id, new_state_id)
        new_est_distance = self.calc_est_distance(new_state_id)
        new_path = state.path + [new_state_id]
        debug_print(f"\n--- old path cost: {state.path_cost}, added cost: {self.euclidean_distance(state.id, new_state_id)}, new path cost: {new_path_cost}")
        debug_print(f"--- estimated distance: {new_est_distance}, total distance: {new_path_cost + new_est_distance}")
        debug_print(f"--- new_path: {new_path}")
        return State(new_state_id, new_path_cost, new_est_distance, new_path)

    def a_star_search(self):
        """
        A* search to find shortest path from starlidt to goal.
        """
        # Repeat the following until is_goal() is true or there are no more states on the min_pq.
        debug_print(f"Before the search starts, min_pq: {self.min_pq}")
        while len(self.min_pq) > 0:
            debug_print(f"\nmin_pq: {self.min_pq}")
            # Pop min element from min priority queue
            total_cost, state_id = heapq.heappop(self.min_pq)
            state = self.frontier[state_id]
            debug_print(f"Popped state with id: {state_id}")

            # Check if the state is a valid state (not yet explored, and up-to-date total cost) .
            # Explanation: The min_pq may contain states with an old, not updated cost. 
            # Those are deprecated states.
            # If the state is valid: use it. If not: discard it and get a new state from min_pq.
            while not self.is_valid(state):
                # Pop min element from min priority queue
                total_cost, state_id = heapq.heappop(self.min_pq)
                state = self.frontier[state_id]
                debug_print(f"Popped state with id: {state_id}")

            # Check if the state is our goal.
            if self.is_goal(state):
                return state.path

            # Mark the state as explored.
            self.explored.add(state.id)

            # Look up and loop through all connections of the state.
            for new_state_id in self.roads[state.id]:
                # Check if the state is already explored
                if new_state_id in self.explored:
                    continue

                # If not, create a new State object with the calculated cost.
                new_state = self.get_new_state(state, new_state_id)

                # There are two cases we have to handle: The state is either already in the frontier,
                # or it has been unexplored so far.
                # If it is already on the frontier, we want to check if the new path is more efficient
                # than the previously explored one. If yes, we're updating the state information to contain
                # the better path.
                # If it is not a more optimal path, we ignore it.
                # If it is unexplored, we're simply adding it to the frontier and min_pq.
                # Both positive cases are handled the same way.
                if new_state.id not in self.frontier or (new_state.total_cost < self.frontier[new_state.id].total_cost):
                    self.frontier[new_state.id] = new_state
                    heapq.heappush(self.min_pq, (new_state.total_cost, new_state.id))

                    # Note: This method adds duplicate states to the priority queue. The old states are
                    # now invalid and will be simply discared as they are popped out of the min_pq.
                    # It is assumed that there will be only a small number of duplicate states in the min_pq,
                    # so this should add only little extra requirements on time and space.
                    # This design choice allows us to use a simple dict and min priority queue to keep track
                    # of the frontier, reducing the time complexity.
        return None
    
    def shortest_path(self, M, start_id, goal_id):
        """
        Finds and returns the shortest path using A* search.
        """
        # Save map information
        self.intersections = M.intersections
        self.roads = M.roads
        self.goal_id = goal_id
        
        num_intersections = len(self.roads)
        assert(start_id < num_intersections), f"Start id is out of bounds, has to be <= {num_intersections-1}"
        assert(goal_id < num_intersections), f"Goal id is out of bounds, has to be <= {num_intersections-1}"

        # Add start to min-PQ and frontier
        start_state = State(start_id, 0, self.calc_est_distance(start_id), [start_id])
        heapq.heappush(self.min_pq, (start_state.total_cost, start_id))
        self.frontier[start_id] = start_state

        # Use A* to find and return the shortest path from start to goal
        return self.a_star_search()


def shortest_path(M, start_id, goal_id):
    print(f"\n======= Start Search for (start, goal): ({start_id}, {goal_id}) =======\n")
    if start_id == goal_id:
        path = [start_id]
    else:
        a_star = AStar()
        path = a_star.shortest_path(M, start_id, goal_id)
    print(f"\nRESULT: {path}")
    return path

def debug_print(text):
    if DEBUG:
        print(text)

if __name__ == "__main__":
    assert(len(sys.argv) == 3), "Needs two command line arguments: state_id_1 and state_id_2"
    DEBUG = True
    map = load_map('map-40.pickle')
    # map = load_map('map-10.pickle')
    
    print("\n----\n")
    path = shortest_path(map, int(sys.argv[1]), int(sys.argv[2]))

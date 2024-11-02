import math
import queue

# Node class to represent each state in the search
class Node:
    def __init__(self, state, parent=None, cost=0, depth=0):
        self.state = state  # Current state of the puzzle (as a list)
        self.parent = parent  # Reference to the parent node (previous state)
        self.cost = cost  # Cost to reach this node (g(n))
        self.depth = depth  # Depth level of this node in the search tree

    # Getter for the state
    def get_state(self):
        return self.state

    # Getter for the cost (g(n))
    def get_cost(self):
        return self.cost

    # Getter for the depth level
    def get_depth(self):
        return self.depth

    # Less-than operator for priority comparison in the priority queue
    def __lt__(self, other):
        return self.cost < other.cost

# Problem class that defines the puzzle and its properties
class Problem:
    def __init__(self, initial_state):
        self.initial_state = initial_state  # The starting state of the puzzle
        self.goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]  # The goal state to achieve

    # Function to check if a state is the goal state
    def goal_test(self, state):
        return state == self.goal_state

    # Function to find the position of the blank tile (0) in the current state
    def find_blank_position(self, state):
        return state.index(0)

    # Generate all valid neighboring states (children) from the current state
    def get_neighbors(self, node):
        neighbors = []
        blank_pos = self.find_blank_position(node.get_state())  # Find the blank tile position
        possible_moves = self.possible_moves(blank_pos)  # Get valid moves for the blank tile

        for move in possible_moves:
            new_state = self.apply_move(node.get_state(), blank_pos, move)  # Apply the move to create a new state
            new_node = Node(new_state, parent=node, cost=node.get_cost() + 1, depth=node.get_depth() + 1)
            neighbors.append(new_node)  # Add the new node to the list of neighbors

        return neighbors

    # Determine valid moves based on the position of the blank tile
    def possible_moves(self, blank_pos):
        moves = []
        if blank_pos > 2:  # Can move up
            moves.append(-3)
        if blank_pos < 6:  # Can move down
            moves.append(3)
        if blank_pos % 3 > 0:  # Can move left
            moves.append(-1)
        if blank_pos % 3 < 2:  # Can move right
            moves.append(1)
        return moves

    # Apply a move to create a new state
    def apply_move(self, state, blank_pos, move):
        new_state = state[:]  # Create a copy of the current state
        target = blank_pos + move  # Determine the target position for the blank tile
        # Swap the blank tile with the target tile to create a new state
        new_state[blank_pos], new_state[target] = new_state[target], new_state[blank_pos]
        return new_state

# Heuristic function for A* search: Euclidean distance
def euclidean_distance_heuristic(state):
    goal_positions = {
        1: (0, 0), 2: (0, 1), 3: (0, 2),
        4: (1, 0), 5: (1, 1), 6: (1, 2),
        7: (2, 0), 8: (2, 1), 0: (2, 2)
    }
    total_distance = 0

    for index, value in enumerate(state):
        if value != 0:  # Skip the blank space
            current_row = index // 3
            current_col = index % 3
            goal_row, goal_col = goal_positions[value]
            # Calculate Euclidean distance and add it to the total
            distance = math.sqrt((current_row - goal_row) ** 2 + (current_col - goal_col) ** 2)
            total_distance += distance

    return total_distance

# A* search algorithm using the Euclidean distance heuristic
def a_star_euclidean_search(problem):
    start_node = Node(problem.initial_state, cost=0, depth=0)  # Initialize the start node
    priorityQueue = queue.PriorityQueue()  # Priority queue for nodes to explore
    start_heuristic = euclidean_distance_heuristic(start_node.get_state())
    start_node.cost = start_heuristic  # Set f(n) = g(n) + h(n), with g(n) = 0 initially
    priorityQueue.put((start_node.get_cost(), start_node))  # Add the start node to the queue
    priorityQueue_states = {tuple(start_node.get_state()): start_node}  # Track states in the queue
    explored = set()  # Set to track explored states
    total_nodes_expanded = 0  # Counter for expanded nodes
    max_queue_size = 1  # Track the maximum size of the queue

    print("\nExpanding state")
    print_state(start_node)

    while not priorityQueue.empty():
        current_cost, current_node = priorityQueue.get()  # Get the node with the lowest cost
        current_state = tuple(current_node.get_state())

        # Remove the state from the priority queue states
        if current_state in priorityQueue_states:
            del priorityQueue_states[current_state]

        # Check if the goal state is reached
        if problem.goal_test(current_node.get_state()):
            print("Goal!!!")
            print(f"\nTo solve this problem the search algorithm expanded a total of {total_nodes_expanded} nodes.")
            print(f"The maximum number of nodes in the queue at any one time was {max_queue_size}.")
            print(f"The depth of the goal node was {current_node.get_depth()}.")
            return True  # Indicate that a solution was found

        # Mark the current state as explored
        explored.add(current_state)
        total_nodes_expanded += 1  # Increment the expanded nodes counter

        # Generate and evaluate neighbors
        for neighbor in problem.get_neighbors(current_node):
            neighbor_state = tuple(neighbor.get_state())
            # Calculate g(n) and h(n) for the neighbor node
            g_cost = current_node.get_cost() - euclidean_distance_heuristic(current_node.get_state()) + 1  # g(n)
            h_cost = euclidean_distance_heuristic(neighbor.get_state())  # h(n)
            f_cost = g_cost + h_cost  # f(n) = g(n) + h(n)
            neighbor.cost = f_cost

            # Check if the neighbor should be added to the priority queue
            if neighbor_state not in explored:
                if neighbor_state not in priorityQueue_states or f_cost < priorityQueue_states[neighbor_state].get_cost():
                    print(f"\nThe best state to expand with g(n) = {g_cost} and h(n) = {h_cost} is:")
                    print_state(neighbor)
                    print("Expanding this node...")
                    priorityQueue.put((f_cost, neighbor))
                    priorityQueue_states[neighbor_state] = neighbor

        # Update the maximum queue size if needed
        max_queue_size = max(max_queue_size, priorityQueue.qsize())

    print("No solution found.")
    return False  # Indicate that no solution was found

# Uniform Cost Search function
def uniform_cost_search(problem):
    start_node = Node(problem.initial_state, cost=0, depth=0)  # Initialize the start node with cost 0
    priorityQueue = queue.PriorityQueue()  # Priority queue for nodes to explore
    priorityQueue.put((start_node.get_cost(), start_node))
    priorityQueue_states = {tuple(start_node.get_state()): start_node}
    explored = set()  # Set to track explored states
    total_nodes_expanded = 0  # Counter for expanded nodes
    max_queue_size = 1  # Track the maximum size of the queue

    print("\nExpanding state")
    print_state(start_node)

    while not priorityQueue.empty():    
        current_cost, current_node = priorityQueue.get()  # Get the node with the lowest cost
        current_state = tuple(current_node.get_state())

        # Remove the state from the priority queue states
        if current_state in priorityQueue_states:
            del priorityQueue_states[current_state]

        # Check if the goal state is reached
        if problem.goal_test(current_node.get_state()):
            print("Goal!!")
            print(f"\nTo solve this problem the search algorithm expanded a total of {total_nodes_expanded} nodes.")
            print(f"The maximum number of nodes in the queue at any one time was {max_queue_size}.")
            print(f"The depth of the goal node was {current_node.get_depth()}.")
            return True  # Indicate that a solution was found

        # Mark the current state as explored
        explored.add(current_state)
        total_nodes_expanded += 1  # Increment the expanded nodes counter

        # Generate and evaluate neighbors
        for neighbor in problem.get_neighbors(current_node):
            neighbor_state = tuple(neighbor.get_state())
            if neighbor_state not in explored:
                # Add to priority queue if not already explored or has a lower cost
                if neighbor_state not in priorityQueue_states or neighbor.get_cost() < priorityQueue_states[neighbor_state].get_cost():
                    print(f"\nThe best state to expand with g(n) = {neighbor.get_cost()} and h(n) = 0 is:")
                    print_state(neighbor)
                    print("Expanding this node...")
                    priorityQueue.put((neighbor.get_cost(), neighbor))
                    priorityQueue_states[neighbor_state] = neighbor

        # Update the maximum queue size if needed
        max_queue_size = max(max_queue_size, priorityQueue.qsize())
        

    print("No solution found.")
    return False  # Indicate that no solution was found

# Heuristic function for misplaced tiles
def misplaced_tile_heuristic(state):
    goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]  # Goal state to compare against
    misplaced_count = 0  # Counter for misplaced tiles
    for index, value in enumerate(state):
        if value != 0 and value != goal_state[index]:  # Count misplaced tiles, skip the blank tile
            misplaced_count += 1
    return misplaced_count

# A* search function using the Misplaced Tile heuristic
def a_star_misplaced_tile_search(problem):
    start_node = Node(problem.initial_state, cost=0, depth=0)  # Initialize the start node
    priorityQueue = queue.PriorityQueue()  # Priority queue for nodes to explore
    start_heuristic = misplaced_tile_heuristic(start_node.get_state())
    start_node.cost = start_heuristic  # Set f(n) = g(n) + h(n), with g(n) = 0 initially
    priorityQueue.put((start_node.get_cost(), start_node))
    priorityQueue_states = {tuple(start_node.get_state()): start_node}
    explored = set()  # Set to track explored states
    total_nodes_expanded = 0  # Counter for expanded nodes
    max_queue_size = 1  # Track the maximum size of the queue

    print("\nExpanding state")
    print_state(start_node)

    while not priorityQueue.empty():
        current_cost, current_node = priorityQueue.get()  # Get the node with the lowest cost
        current_state = tuple(current_node.get_state())

        if current_state in priorityQueue_states:
            del priorityQueue_states[current_state]

        # Check if the goal state is reached
        if problem.goal_test(current_node.get_state()):
            print("Goal!!!")
            print(f"\nTo solve this problem the search algorithm expanded a total of {total_nodes_expanded} nodes.")
            print(f"The maximum number of nodes in the queue at any one time was {max_queue_size}.")
            print(f"The depth of the goal node was {current_node.get_depth()}.")
            return True  # Indicate that a solution was found

        # Mark the current state as explored
        explored.add(current_state)
        total_nodes_expanded += 1  # Increment the expanded nodes counter

        # Generate and evaluate neighbors
        for neighbor in problem.get_neighbors(current_node):
            neighbor_state = tuple(neighbor.get_state())
            g_cost = current_node.get_cost() - misplaced_tile_heuristic(current_node.get_state()) + 1  # g(n)
            h_cost = misplaced_tile_heuristic(neighbor.get_state())  # h(n)
            f_cost = g_cost + h_cost  # f(n) = g(n) + h(n)
            neighbor.cost = f_cost

            # Check if the neighbor should be added to the priority queue
            if neighbor_state not in explored:
                if neighbor_state not in priorityQueue_states or f_cost < priorityQueue_states[neighbor_state].get_cost():
                    print(f"\nThe best state to expand with g(n) = {g_cost} and h(n) = {h_cost} is:")
                    print_state(neighbor)
                    print("Expanding this node...")
                    priorityQueue.put((f_cost, neighbor))
                    priorityQueue_states[neighbor_state] = neighbor

        # Update the maximum queue size if needed
        max_queue_size = max(max_queue_size, priorityQueue.qsize())

    print("No solution found.")
    return False  # Indicate that no solution was found

# Print the current state of the puzzle in a 3x3 grid format
def print_state(node):
    for index, value in enumerate(node.get_state()):
        if value == 0:
            print("b", end=' ')  # Print 'b' for the blank space
        else:
            print(value, end=' ')
        if (index + 1) % 3 == 0:  # Move to the next line after every 3 elements
            print()
    print()

# Test cases for the puzzle
test_cases = {
    "Trivial": [1, 2, 3, 4, 5, 6, 7, 8, 0],  # Already solved state
    "Very Easy": [1, 2, 3, 4, 5, 6, 7, 0, 8],  # Near solution
    "Easy": [1, 2, 0, 4, 5, 3, 7, 8, 6],  # Simple puzzle
    "Doable": [0, 1, 2, 4, 5, 3, 7, 8, 6],  # More steps needed
    "Impossible": [1, 2, 3, 4, 5, 6, 8, 7, 0],  # Unsolvable state
    "Oh Boy": [8, 7, 1, 6, 0, 2, 5, 4, 3],  # Complex puzzle
    "Trace": [1, 0, 3, 4, 2, 6, 7, 5, 8]  # Specific example for tracing
}

# User interface to input puzzle and choose algorithm
def projectIntro():
    print("Welcome to Saachi Raju's and Akanksh Divyananda's 8 puzzle solver.")
    print("Type '1' to use a default puzzle, or '2' to enter your own puzzle.")

    choice = int(input("\t"))

    if choice == 1:
        puzzle = [1, 0, 3, 4, 2, 6, 7, 5, 8]  # Default puzzle state for testing
        print("\nUsing the default puzzle configuration.")
    elif choice == 2:
        print("\nEnter your puzzle, use a zero to represent the blank")
        puzzle = []
        for i in range(3):  # Prompt for 3 rows of input
            row = input(f"Enter the {i + 1} row, use space or tabs between numbers:\t").split()
            puzzle.extend([int(num) for num in row])
    else:
        print("Invalid choice.")
        return None, None

    print("\nEnter your choice of algorithm")
    print("\t1. Uniform Cost Search")
    print("\t2. A* with the Misplaced Tile heuristic")
    print("\t3. A* with the Euclidean distance heuristic")

    algorithm_choice = int(input("\t"))

    return puzzle, algorithm_choice

# Main program to run based on user input
if __name__ == '__main__':
    initial_puzzle, algo_choice = projectIntro()
    if initial_puzzle is None or algo_choice not in [1, 2, 3]:
        print("Exiting the program.")
    else:
        problem = Problem(initial_puzzle)
        if algo_choice == 1:
            result = uniform_cost_search(problem)
            if result:
                print("\nSolution found.")
            else:
                print("No solution found. Could be impossible")
        elif algo_choice == 2:
            result = a_star_misplaced_tile_search(problem)
            if result:
                print("\nSolution found.")
            else:
                print("No solution found. Could be impossible")
        elif algo_choice == 3:
            result = a_star_euclidean_search(problem)
            if result:
                print("\nSolution found.")
            else:
                print("No solution found. Could be impossible")
        else:
            print("Algorithm not yet implemented.")

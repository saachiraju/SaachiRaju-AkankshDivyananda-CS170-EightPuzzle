# Node class
class Node:
    def __init__(self, state, parent=None, cost=0, depth=0):
        self.state = state
        self.parent = parent
        self.cost = cost  # g(n)
        self.depth = depth  # Depth of the node

    def get_state(self):
        return self.state

    def get_cost(self):
        return self.cost

    def get_depth(self):
        return self.depth

    def __lt__(self, other):
        return self.cost < other.cost


# Problem class
class Problem:
    def __init__(self, initial_state):
        self.initial_state = initial_state
        self.goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]

    def goal_test(self, state):
        return state == self.goal_state

    def find_blank_position(self, state):
        return state.index(0)

    def get_neighbors(self, node):
        neighbors = []
        blank_pos = self.find_blank_position(node.get_state())
        possible_moves = self.possible_moves(blank_pos)

        for move in possible_moves:
            new_state = self.apply_move(node.get_state(), blank_pos, move)
            new_node = Node(new_state, parent=node, cost=node.get_cost() + 1, depth=node.get_depth() + 1)
            neighbors.append(new_node)

        return neighbors

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

    def apply_move(self, state, blank_pos, move):
        new_state = state[:]
        target = blank_pos + move
        new_state[blank_pos], new_state[target] = new_state[target], new_state[blank_pos]
        return new_state


# Uniform Cost Search function
import queue

def uniform_cost_search(problem):
    start_node = Node(problem.initial_state, cost=0, depth=0)
    frontier = queue.PriorityQueue()
    frontier.put((start_node.get_cost(), start_node))
    frontier_states = {tuple(start_node.get_state()): start_node}
    explored = set()
    total_nodes_expanded = 0
    max_queue_size = 1

    print("\nExpanding state")
    print_state(start_node)

    while not frontier.empty():
        current_cost, current_node = frontier.get()
        current_state = tuple(current_node.get_state())
        del frontier_states[current_state]

        if problem.goal_test(current_node.get_state()):
            print("Goal!!")
            print(f"\nTo solve this problem the search algorithm expanded a total of {total_nodes_expanded} nodes.")
            print(f"The maximum number of nodes in the queue at any one time was {max_queue_size}.")
            print(f"The depth of the goal node was {current_node.get_depth()}.")
            return True  # Indicating a solution was found

        explored.add(current_state)
        total_nodes_expanded += 1

        for neighbor in problem.get_neighbors(current_node):
            neighbor_state = tuple(neighbor.get_state())
            if neighbor_state not in explored:
                if neighbor_state not in frontier_states or neighbor.get_cost() < frontier_states[neighbor_state].get_cost():
                    print(f"\nThe best state to expand with g(n) = {neighbor.get_cost()} and h(n) = 0 is:")
                    print_state(neighbor)
                    print("Expanding this node...")
                    frontier.put((neighbor.get_cost(), neighbor))
                    frontier_states[neighbor_state] = neighbor

        # Check the max size of the queue
        max_queue_size = max(max_queue_size, frontier.qsize())

    print("No solution found.")
    return False  # Indicating no solution was found


def print_state(node):
    for index, value in enumerate(node.get_state()):
        if value == 0:
            print("b", end=' ')
        else:
            print(value, end=' ')
        if (index + 1) % 3 == 0:
            print()
    print()


# User interface to input puzzle and choose algorithm
def projectIntro():
    print("Welcome to Saachi Raju's and Akanksh Divyananda's 8 puzzle solver.")
    print("Type '1' to use a default puzzle, or '2' to enter your own puzzle.")

    choice = int(input("\t"))

    if choice == 1:
        puzzle = [1, 2, 3, 4, 0, 5, 6, 7, 8] 
        print("\nUsing the default puzzle configuration.")
    elif choice == 2:
        print("\nEnter your puzzle, use a zero to represent the blank")
        puzzle = []
        for i in range(3):  # Prompt for 3 rows
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
                print("No solution found.")
        else:
            print("Algorithm not yet implemented.")

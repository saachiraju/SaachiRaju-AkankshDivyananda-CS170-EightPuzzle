def projectIntro():
    # Custom introductory message with student names/IDs
    IntroMessage1 = "Welcome to Saachi Raju's and Akanksh Divyananda's 8 Puzzle Solver."
    IntroMessage2 = "Type '1' to use a default puzzle, or '2' to enter your own puzzle."

    print(IntroMessage1)
    print(IntroMessage2)
    
    Puzzle = []
    DefaultPuzzle = [1, 2, 0, 4, 5, 3, 6, 7, 8] 
    
    UserPuzzleChoice = int(input("\t"))

    if UserPuzzleChoice == 1: 
        Puzzle = DefaultPuzzle
        print("\nUsing the default puzzle configuration.")

    elif UserPuzzleChoice == 2:
        print("\nEnter your puzzle, use a zero to represent the blank")
        Puzzle = []
        for i in range(3):  # Assumes a 3x3 puzzle
            row = input(f"Enter row {i + 1}, use space or tabs between numbers\t").strip()
            Puzzle.extend([int(x) for x in row.split()])
    
    print("\nEnter your choice of algorithm")
    print("\t1. Uniform Cost Search")
    print("\t2. A* with the Misplaced Tile heuristic")
    print("\t3. A* with the Euclidean distance heuristic")

    # for testing sake
    alg_choice = input("\t")  
    return Puzzle, alg_choice 

if __name__ == '__main__':
    # all for tempororary testing to see if it works
    puzzle, alg_choice = projectIntro() 
    print(f"Selected puzzle: {puzzle}")
    print(f"Chosen algorithm: {alg_choice}")



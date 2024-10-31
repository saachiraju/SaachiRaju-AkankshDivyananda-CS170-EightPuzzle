
def projectIntro():
    IntroMessage1 = "Welcome to Saachi Raju's and Akanksh Divyananda's 8 puzzle solver.\n"
    IntroMessage2 = "Type '1' to use a default puzzle, or '2' to enter your own puzzle."
    
    print(IntroMessage1)
    print(IntroMessage2)        
    
    Puzzle = []
    DefaultPuzzle = [1, 2, 0, 4, 5, 3, 6, 7, 8]
    
    UserPuzzleChoice = int(input("Enter your choice: "))
    
    if UserPuzzleChoice == 1: 
        Puzzle = DefaultPuzzle
    
    return Puzzle


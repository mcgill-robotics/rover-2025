import sys

class MorseNode:
    def __init__(self, translation: str | None, dot: MorseNode | None = None, dash: MorseNode | None = None):
        self.dot = dot
        self.dash = dash
        self.translation = translation

morse_tree = MorseNode(None, 
                       MorseNode("E", 
                                 MorseNode("I", 
                                           MorseNode("S", 
                                                     MorseNode("H", 
                                                               MorseNode("5"), 
                                                               MorseNode("4")), 
                                                     MorseNode("V", 
                                                               None, 
                                                               MorseNode("3"))), 
                                           MorseNode("U", 
                                                     MorseNode("F"), 
                                                     MorseNode(None, 
                                                               None, 
                                                               MorseNode("2")))), 
                                 MorseNode("A", 
                                           MorseNode("R", 
                                                     MorseNode("L"), 
                                                     MorseNode(None, 
                                                               MorseNode("+"), 
                                                               None)),
                                           MorseNode("W",
                                                     MorseNode("P"),
                                                     MorseNode("J",
                                                               None,
                                                               MorseNode("1"))))),
                       MorseNode("T",
                                 MorseNode("N", 
                                           MorseNode("D",
                                                     MorseNode("B",
                                                               MorseNode("6"),
                                                               MorseNode("=")),
                                                     MorseNode("X",
                                                               MorseNode("/"),
                                                               None)),
                                           MorseNode("K",
                                                     MorseNode("C"),
                                                     MorseNode("Y"))),
                                 MorseNode("M",
                                           MorseNode("G",
                                                     MorseNode("Z",
                                                               MorseNode("7"),
                                                               None),
                                                     MorseNode("Q")),
                                           MorseNode("O",
                                                     MorseNode(None,
                                                               MorseNode("8"),
                                                               None),
                                                     MorseNode(None,
                                                               MorseNode("9"),
                                                               MorseNode("0"))))))

if __name__ == "__main__":
    dot = "."
    dash = "-"
    restart = "r"
    try:
        args = sys.argv[1:]
        i = 0
        while i < len(args):
            if args[i] == "--dot":
                dot = args[i + 1]
            elif args[i] == "--dash":
                dash = args[i + 1]
            elif args[i] == "--restart":
                restart = args[i + 1]
            i += 2
    except:
        print("Invalid or nonexistent command line args, reverting to defaults...\n")
    
    print(f"Type {dot} to represent a dot or {dash} to represent a dash. Do not put spaces between dots and dashes. Press enter after each letter. The translation for the message thus far will print after each letter. Press {restart} followed by enter to clear the translation and start again.")
    
    translation = ""
    while True:
        code = input("Type one morse letter: ").strip()
        location = morse_tree
        for symbol in code:
            if symbol == dot:
                location = location.dot
            elif symbol == dash:
                location = location.dash
            elif symbol == restart:
                print("Restart code received. Cleared translation.")
                location = None
                translation = ""
                break
            else:
                print("Unknown symbol in morse code. Ignoring...")
                location = None
                break
            if not location:
                print("Invalid morse code letter. Ignoring...")
                break
        if location and not location.translation:
            print("Invalid morse code letter. Ignoring...")
        elif location:
            translation += location.translation
            print(f"Running translation: {translation}")
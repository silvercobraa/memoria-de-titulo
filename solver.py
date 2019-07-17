import solver.solver as Solver

def main():
    state = 'BDDFUULUULLFFRRFRRDBBDFDDLDRDRBDBUUURLBRLBLLBLRUFBUFFF'
    face = 'URFDLB'
    print(state)
    for f in range(6):
        print face[f] + ':'
        for i in range(3):
            for j in range(3):
                print state[9*f + 3*i + j],
            print ''
        print ''
    moves = Solver.solve(state, 20, 2)
    print(moves)


if __name__ == '__main__':
    main()

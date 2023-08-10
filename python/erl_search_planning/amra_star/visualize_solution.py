import argparse
from erl_search_planning.amra_star.solution import AMRAStarSolution


def main():
    parser = argparse.ArgumentParser(description="Visualize AMRA* solution")
    parser.add_argument("filename", type=str, help="filename of AMRA* solution")
    sol = AMRAStarSolution(parser.parse_args().filename)
    sol.visualize_paths()
    sol.visualize_opened_states()
    sol.visualize_closed_states()
    sol.visualize_inconsistent_states()


if __name__ == "__main__":
    main()

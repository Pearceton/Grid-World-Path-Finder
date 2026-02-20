from utils import *

# Breadth First Search
def breadth_first_search(problem):
    node = Node(problem.INITIAL)
    nodes_expanded = 0
    if problem.GOAL_TEST(node.STATE):
        return node, nodes_expanded
    frontier = Queue()
    frontier.push(node)
    reached = set()
    reached.add(node.STATE)
    while not frontier.isEmpty():
        node = frontier.pop()
        nodes_expanded += 1
        for child in node.expand(problem):
            s = child.STATE
            if problem.GOAL_TEST(s):
                return child, nodes_expanded
            if s not in reached:
                reached.add(s)
                frontier.push(child)
    return None, nodes_expanded


#Depth First Search
def is_cycle(node):
    current = node.PARENT
    while current is not None:
        if current.STATE == node.STATE:
            return True
        current = current.PARENT
    return False

def depth_first_search(problem):
    node = Node(problem.INITIAL)
    frontier = Stack()
    frontier.push(node)
    reached = set()
    reached.add(node.STATE)
    nodes_expanded = 0
    while not frontier.isEmpty():
        node = frontier.pop()
        nodes_expanded += 1
        if problem.GOAL_TEST(node.STATE):
            return node, nodes_expanded
        for child in node.expand(problem):
            if child.STATE not in reached:
                reached.add(child.STATE)
                frontier.push(child)
    return None, nodes_expanded


# Greedy Best First Search
def greedy_best_first_search(problem, h):
    node = Node(STATE=problem.INITIAL)
    frontier = PriorityQueue()
    frontier.push(node, h(node))
    reached = {}
    reached[node.STATE] = node
    nodes_expanded = 0

    while not frontier.isEmpty():
        node = frontier.pop()
        nodes_expanded += 1
        if problem.GOAL_TEST(node.STATE):
            return node, nodes_expanded
        for child in node.expand(problem):
            s = child.STATE
            if s not in reached or child.PATH_COST < reached[s].PATH_COST: 
                reached[s] = child
                frontier.push(child, h(child))
    return None, nodes_expanded

#A* Search
def a_star_search(problem, h):
    node = Node(STATE=problem.INITIAL)
    frontier = PriorityQueue()
    frontier.push(node, node.PATH_COST + h(node))
    reached = {}
    reached[node.STATE] = node
    nodes_expanded = 0

    while not frontier.isEmpty():
        node = frontier.pop()
        nodes_expanded += 1
        if problem.GOAL_TEST(node.STATE):
            return node, nodes_expanded
        for child in node.expand(problem):
            s = child.STATE
            if s not in reached or child.PATH_COST < reached[s].PATH_COST: 
                reached[s] = child
                f_value = child.PATH_COST + h(child)
                frontier.push(child, f_value)
    return None, nodes_expanded

def run_all(problem, h):

    bfs_goal, bfs_expanded = breadth_first_search(problem)
    dfs_goal, dfs_expanded = depth_first_search(problem)
    gbfs_goal, gbfs_expanded = greedy_best_first_search(problem, h)
    astar_goal, astar_expanded = a_star_search(problem, h)

    print("BFS:", bfs_goal.PATH_COST if bfs_goal else None, bfs_expanded)
    print("DFS:", dfs_goal.PATH_COST, dfs_expanded)
    print("GBFS:", gbfs_goal.PATH_COST, gbfs_expanded)
    print("A*:", astar_goal.PATH_COST, astar_expanded)
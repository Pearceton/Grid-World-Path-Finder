from utils import *

# Breadth First Search
def breadth_first_search(problem):
    node = Node(problem.INITIAL)
    if problem.GOAL_TEST(node.STATE):
        return node
    frontier = Queue()
    frontier.push(node)
    reached = set()
    reached.add(node.STATE)
    while not frontier.isEmpty():
        node = frontier.pop()
        for child in node.expand(problem):
            s = child.STATE
            if problem.GOAL_TEST(s):
                return child
            if s not in reached:
                reached.add(s)
                frontier.push(child)
    return None


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
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.GOAL_TEST(node.STATE):
            return node
        for child in node.expand(problem):
            if not is_cycle(child):
                frontier.push(child)
    return None


# Greedy Best First Search
def greedy_best_first_search(problem, h):
    node = Node(STATE=problem.INITIAL)
    frontier = PriorityQueue()
    frontier.push(node, h(node))
    reached = {}
    reached[node.STATE] = node

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.GOAL_TEST(node.STATE):
            return node
        for child in node.expand(problem):
            s = child.STATE
            if s not in reached or child.PATH_COST < reached[s].PATH_COST: 
                reached[s] = child
                frontier.push(child, h(child))
    return None

#A* Search
def a_star_search(problem, h):
    node = Node(STATE=problem.INITIAL)
    frontier = PriorityQueue()
    frontier.push(node, node.PATH_COST + h(node))
    reached = {}
    reached[node.STATE] = node

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.GOAL_TEST(node.STATE):
            return node
        for child in node.expand(problem):
            s = child.STATE
            if s not in reached or child.PATH_COST < reached[s].PATH_COST: 
                reached[s] = child
                f_value = child.PATH_COST + h(child)
                frontier.push(child, f_value)
    return None
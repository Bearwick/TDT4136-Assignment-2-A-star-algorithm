from Map import Map_Obj

class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    #When comparing two nodes with ==. It will compare their positions. 
    def __eq__(self, other):
        return self.position == other.position

#Implementation of the A* algorithm 
def astar(): #Implementation influenced by https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
    #Create start and end node objects. 
    start_node = Node(None, Samf.get_start_pos())
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, Samf.get_goal_pos())
    end_node.g = end_node.h = end_node.f = 0
    
    open_list = [] #List of discovered nodes.
    closed_list = [] #List of visited nodes.
    
    open_list.append(start_node)

    while len(open_list) > 0:
        #Finds the node with lowest f - total cost from start to end.
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item 
                current_index = index
        
        open_list.pop(current_index)
        closed_list.append(current_node)


        #Returns the path from start to end if the current node is the end node.
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent 
            return path[::-1] #Returns the path reversed, since the first position added is the end node, then its parent, i.e. second to last.

        
        children = [] #List for the current node´s children, i.e. its neighbours.
        for new_pos in [[-1, 0], [0, -1], [0, 1],[1, 0]]: #Coordinates to get the neighbour nodes.
            #Calculates the child position
            child_node_pos = [current_node.position[0] + new_pos[0], current_node.position[1] + new_pos[1]]

            #Checks that the child is within the map.
            if child_node_pos[0] > (len(Samf.int_map) - 1) or child_node_pos[0] < 0 or child_node_pos[1] > (len(Samf.int_map[len(Samf.int_map)-1]) -1) or child_node_pos[1] < 0:
                continue

            #Checks that the child is a walkable path, i.e. not an obstacle.
            if Samf.get_cell_value(child_node_pos) != -1:
                new_node = Node(current_node, child_node_pos)
                children.append(new_node)
        
        #Checks if the neighbours added in child is not allready visited or discovered.
        for child in children:
            checkUsed = False
            
            #Checks if child is allready visited
            for closed_child in closed_list:
                if child == closed_child:
                    checkUsed = True
            
            #Calculates g - cost from start to child, h - cost from child to end node, f - g + h. 
            child.g = current_node.g + Samf.get_cell_value(child.position)
            child.h = abs(end_node.position[0] - child.position[0] + end_node.position[1] - child.position[1]) #Manhatten heuristic.
            child.f = child.g + child.h
            
            #Checks if node is allready discovered.
            for open_node in open_list:
                if child == open_node:
                    checkUsed = True

            #Adds child to open_list if not allready visited or discovered.
            if checkUsed == False:
                open_list.append(child)
    return None #Returns None if open_list is empty, i.e. no more nodes to visit (No path). 

#Shows map with path printed. 
def show_map(path):
    path.pop(0)
    path.pop(len(path)-1) 
    Samf.show_map(path)


def main():
    path = astar() 
    print("\nNumber of steps: ", len(path)-1, "\n\n",path)
    show_map(path)
 

Samf = Map_Obj(1) #Choose task

if __name__ == "__main__":
    main()

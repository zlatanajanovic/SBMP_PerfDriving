%{ 
List of Files
 ReadMe.txt          : Contains List of Files and description
 Astar_tutorial.pdf  : Description of A* algorithm, examples
 A_star1.m           : This is the main file that contains the algorithm. This needs to be executed to
                                      run the program.
 costest.m           : This is a function that estimates cost between 2 nodes.
 min_fn.m              : This function takes the list OPEN as one of its arguments and returns the node
                                   with the smallest cost function.
 Node_index.m        : This function returns the index of the location of a node in the list OPEN.
 Expand_array.m      : This function takes a node and returns the expanded list of successors, with
                                  the calculated fn values. One of the criteria being none of the successors are on the
                                  CLOSED list.
 insert_open.m       : This function populates the list OPEN with values that have been passed to
                                 it. 
%}
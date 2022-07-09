Contributor: 

Biyao Liang @ biyaolia@usc.edu

Manoj Sunkara @ msunkara@usc.edu

In this project we have 7 major functions for our Trojanmap app. 

![image](https://user-images.githubusercontent.com/97922238/166183514-df1d5d99-4be0-4794-affc-96ba7e8b7ce4.png)


Autocomplete: 

Uses the 2 following functions.
auto_helper(std::string a, std::string b)
Autocomplete(std::string name)

  auto_helper:

    It takes in 2 string inputs a and b,convert both of them to lowercase and then compare the smaller string to the bigger one. If the bigger one contains the           smaller one in front, it returns true for a match. Otherwise, it returns false.


  Autocomplete:

    It drives auto_helper and stores the autocomplete match results.

![Capture](https://user-images.githubusercontent.com/97922238/166183614-d0c140da-dcf4-4006-a739-5ae54d032fad.PNG)


![image](https://user-images.githubusercontent.com/97922238/166183535-10c0ae97-c0f8-4b1b-8590-ac426cfd2f87.png)


Find the location:

Returns the latitude and longitude of a location


![Capture](https://user-images.githubusercontent.com/97922238/166183956-58210ad0-cd67-405c-a3e3-bf252dad2cbd.PNG)


3.Calculate Shortest Path 

We have implemented 2 different algorithms for this purpose: Dijkstra and Bellman-Ford. 
Dijkstra is an algorithm that returns a shortest path solution between 2 nodes very quickly, and its runtime increases as the distance between the 2 nodes increases.
On the other hand, Bellman-Ford is an algorithm that is very slow compared to Dijkstra when distance between nodes is short, but has a very stable runtime independent of the distance, but dependent on the size of the graph. 

![Capture](https://user-images.githubusercontent.com/97922238/166184069-c642663a-760a-4ea6-9c41-c22e3ab5ae87.PNG)


![Capture](https://user-images.githubusercontent.com/97922238/166184144-71904b47-0cda-498c-9fa8-f2d0184926d9.PNG)

4.Traveling Salesman Problem: 3 approaches
The Traveling Salesman Problem we have here is that we need to find the shortest way to go through all of the nodes.
We have implemented 3 approaches: Brute force, Early-tracking and 2opt a running video of three approaches for n = 6 can be found in 

Brute force:

We iterate through all the possible permutations of the node orderings and return the ordering with the shortest travel distance and exactly, what that shortest distance is.

![output0](https://user-images.githubusercontent.com/97922238/166184325-492191dc-5774-4822-9cec-488a33ae3c1d.gif)



Early-tracking:

Similar to brute force approach, but as we are going through all possible permutations, we compare the current path's current traveled distance to the shortest possible distance we have so far. If we already exceed that minimum distance before hitting the end of this possible path, we stop looking further and start looking at the next possible ordering.

![output0_backtracking](https://user-images.githubusercontent.com/97922238/166184336-015df4c8-1d55-4aa2-8204-82ad4f97eaa9.gif)

2opt:

We start with 1 possible path, and reverse the orders of some of the nodes in the path to see if this ordering can potentially give us a shorter travel distance. 

![Capture](https://user-images.githubusercontent.com/97922238/166184186-97d6d4d7-9ed8-48bf-a81f-e2dd25d6e51c.PNG)

One important characteristic of 2opt is, it is a heuristic which returns a good solution, but not necessarily the best solution.
The advantage of a heuristic is that, when the other approaches are taking too long to find the absolute shortest path, 2opt can give a good solution within a short amount of time that can be a good enough solution to the problem.

![output0_2opt](https://user-images.githubusercontent.com/97922238/166184352-28c36db9-56c7-493b-8422-7bb9a79ea265.gif)

![Capture](https://user-images.githubusercontent.com/97922238/166184217-75455fc1-dd61-490c-9747-35f87f462bbe.PNG)


![image](https://user-images.githubusercontent.com/97922238/166184235-247d55fe-6721-4ad5-8e11-1372b64d2774.png)



As we can see from the performance analysis, the 2opt path's distance is not too far away from the ground truth returned from brute force/early tracking.

5. Cycle Detection 

The definition of a cycle in a graph is when all the edges are directed, and every node has at least one parent node pointing to it and a child node it points at.In the scope of this project, we use this function to find any cycles present within a subgraph.

Is_cycle is a helper function.

![Capture](https://user-images.githubusercontent.com/97922238/166184486-2e38c3ce-3015-4527-9bef-b57b8fb78dbf.PNG)

sub graph with a cycle present

![Capture1](https://user-images.githubusercontent.com/97922238/166184622-67bd8bdc-7d1e-4915-a9db-f23feb3042ba.PNG)

sub graph without a cycle present

![Capture2](https://user-images.githubusercontent.com/97922238/166184644-1413aa10-8a9f-4d9f-953c-24c530cc51e9.PNG)


6.Topological sort

The delivering Trojan problem is that we need to travel to a list of nodes, but these nodes have dependencies meaning that we need to travel to some of the nodes first, then the others. In this problem, we need to output a list containing the order of the nodes we need to travel to to satisfy the dependencies between them. Note that if there's a cycle, there is no possible topological order. 

![image](https://user-images.githubusercontent.com/97922238/166195000-d1e1182a-0bb9-4299-a021-ac096d524d85.png)

![image](https://user-images.githubusercontent.com/97922238/166195012-98ddf054-a5d5-4604-8c2d-0c8e701d68b9.png)

![image](https://user-images.githubusercontent.com/97922238/166184700-34721ea4-95a6-4be2-84c2-787cefc4663d.png)

7. Find nearby

This function takes in a search radius, a location name, an attribute and a number x to return a list of x locations near the location entered within a radius. The locations are in order of distance from target location.

The target location is highlighted as a green dot

![Capture3](https://user-images.githubusercontent.com/97922238/166184849-c917f8d6-2073-4303-a92a-1a908f9ce2e9.PNG)


![Capture](https://user-images.githubusercontent.com/97922238/166183865-5a3b6570-e217-4843-a086-371d070c2881.PNG)



A UI for the code was created using ncurses :

![c4b9cbc66c03162f62d9a0880917c9e2266e3d5c](https://user-images.githubusercontent.com/97922238/166185495-729cb660-10d5-4de4-8eee-c873364e5c3f.gif)


Lessons learned:
We practiced our skills in using version control software in collaboration
Heuristics: sometimes a good solution is enough
Sometimes the problem is more than finding the 1 correct answer: efficiency, space, stability all play an important role in software engineering. We want to know exactly what we are looking for and not let our vision be clouded by focusing on finding the 1 absolute best answer(even though a lot of times, that is the way.)

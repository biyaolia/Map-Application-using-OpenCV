# TrojanMap: a map application for Trojans @ USC

Contributor:
Biyao Liang @ biyaolia@usc.edu
Manoj Sunkara @ msunkara@usc.edu
For EE538 Spring 2022 at USC Viterbi college of Engineering

In this project we have 7 major functions for our Trojanmap app.
1. Autocomplete
2. Find the location
3. Calculate Shortest Path
4. Travelling salesman problem
5. Cycle Detection
6. Topological Sort
7. Find Nearby

**Autocomplete**
It uses the 2 following functions.

    auto_helper(std::string a, std::string b)
It takes in 2 string inputs a and b,convert both of them to lowercase and then compare the smaller string to the bigger one. If the bigger one contains the smaller one in front, it returns true for a match. Otherwise, it returns false.

    Autocomplete(std::string name)

 It drives auto_helper and stores the autocomplete match results.

![](https://lh5.googleusercontent.com/Zu9LCh5rDn6b2GYvu3jqVGKBRc36VCYwAEs-wnq1VlYMNV77SYCJ0Z--OryA2litJcnCRmARAzS-H8LqMZuV2kE7H0BhjfWQ3n6_jJHC1s4D3rydAWxyCS63EVCdR0nIg_9jJKlRXOHL5nQWvA)

Figure 1. Autocomplete structure

![](https://lh3.googleusercontent.com/kb_bBLCc2MSgGFOy1Gn99JTgVAp7IlYObta96Z7MyhGI7Ci8iF56-m2zn6dCnMbxJ98Rjf3ZX0abDoXvfwJFKbeF4YIYx4giqgbagSEqIA8XE6cNfyFenW9njjEYTdF5upiwEeBtjgT4uDQ6OQ)

   Figure 2. Screenshot of Autocomplete output in terminal
  

**Find the location**

    std::string TrojanMap::GetID(const std::string& name)
Returns the ID of a location in the graph given its name.

    double TrojanMap::GetLon(const std::string& id)

Returns the longitude of a location given its ID.

    double TrojanMap::GetLat(const std::string& id)

Returns the latitude and latitude of a location.

![](https://lh5.googleusercontent.com/ZNdZE-ddI9kIMEZYdIisHQH__x3DlbfPvq_AJG_RrNtskZpk4vqYBX-VfyjbBgQldHHCPrVt_FKh2bzddJBk0_L-mAoZaZFRSiBHLWnI6fpHh1UYrsRHNXkJp1UJ6fGG9gKb1V9gkDNR4-wjcw)

Figure 3. Flowchart of Find_location

![](https://lh5.googleusercontent.com/OOpnZ45xeBe0pIy_WxATG_j3Syig4QrmGsIcvSSngjT-7mkQHJm-fM9CnTMJS6Tl3Va4T8iumjZEILPciU631rww_Fced-4ZXnUgrOoyPuI02W4A4m-x8myc1Paw6t2GUiaRLSmKaADjOD4GHg)

Figure 4. Screenshot of output of find the location


**Calculate Shortest Path**

We have implemented 2 different algorithms for this purpose: Dijkstra and Bellman-Ford.

    std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
        std::string location1_name, std::string location2_name)

Dijkstra is an algorithm that returns a shortest path solution between 2 nodes very quickly, and its runtime increases as the distance between the 2 nodes increases.

    std::vector<std::string> 
    `TrojanMap::CalculateShortestPath_Bellman_Ford(std::string location1_name, std::string location2_name)

On the other hand, Bellman-Ford is an algorithm that is very slow compared to Dijkstra when distance between nodes is short, but has a very stable runtime independent of the distance, but dependent on the size of the graph.

  

![](https://lh6.googleusercontent.com/PB573HMoAuWfXFGJO-e4QtMLQlKs2Mm2ZTcj8k7zSfZD2Z9wcinVTgCbieDVx5YiPyrJd8rNK-3URRq4bS7oGEhRq6mKXEchLsCWyvVNYM1LTagwjPG44Yba5okqNW_OUvqlq87BXiN687HfZQ)

Figure 5. Shortest path found by our program

![](https://lh6.googleusercontent.com/9q-qq5CMC_fx7hB9dXbkTCBL1nIi081yI3UhvvVqu5bV_SPqFUOSG6QvrpcJWAvhBfTXufIxQLt1IdX1l-FOg1rCnG3K0MCD2Xq7Hv5SOy_LYAE9EAUP8hwmAQgtwW9_aZFhkcpbTwGdNxHkTA)

Figure 6. Terminal output of Dijkstra

![](https://lh4.googleusercontent.com/JDItvSlNUmxhrMx7fPLIFawXLCa6Ws6wEnvGWFzc2MSgcgh1FGFRdwszrf7QgwC-IWYaYJeX0ZDAhX6lBgR0yGg3zeEZ6ERj0UIiVAiPqDuTrD83ndPeEvKanC7vEWizMQlaeyyi6oEjdeN_mg)

Figure 7. Terminal output of Bellman-Ford

![](https://lh3.googleusercontent.com/I-qdQvbOAPMM5qY9jOZAAnGV3y5V0SzoTvz9wNotc7dvnlLhl2Tegi41DgdXRzcHbfaigoeXc1fMHurcvlYSGIINPofAoBCVJr303t0TKnCYrz2fH5GMEQCphVAKy_fnHP3mnCpO3GszND5vlw)

Chart 1. Runtime comparison of Dijkstra and Bellman-Ford

![](https://lh6.googleusercontent.com/n3V6OazQajOmVd5gVHjZAd-bgezMNU1fB4lnocFD4iDRXgHgUMCcOppL3xG27tWLJyMaKlk_jfu1W7ozx8tLHyebBE6vtSB7LZMUP8eptnnCP3DCxbez5y9EErvWD0BpH93HcGPAU3njB3Hqxw)

Figure 8.  Dijkstra's runtime over number of nodes in path

![](https://lh5.googleusercontent.com/QJJ_19dMddSVsTGXMKh94ajrOViC8g8k1iskglqfO8NkMV-mhZ_sxpe7m6a3KLmKwQ_l0fPnyiNSm8j2dScKdNLXXGVDBd_jC_kkV_QdIMGueHcoJYaNbHDCC8-6rQsHb0yDoAXz_KTGGQIUIg)

Figure 9.  Bellman-Ford's runtime over number of nodes in path
  
**Traveling Salesman Problem**

The Traveling Salesman Problem we have here is that we need to find the shortest way to go through all of the nodes.

We have implemented 3 approaches: Brute force, Early-tracking and 2opt.

Brute force:

    std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(std::vector<std::string> location_ids)

We iterate through all the possible permutations of the node orderings and return the ordering with the shortest travel distance and exactly, what that shortest distance is.

  ![output0](https://user-images.githubusercontent.com/97922238/166184325-492191dc-5774-4822-9cec-488a33ae3c1d.gif)
  
  A gif of our Brute force approach

Early-tracking:

    void TrojanMap::TTB_Aux(std::string start,std::vector<std::string> location_ids,std::string cur_node,double cur_dist, std::vector<std::string> &cur_path, double &min_dist, std::vector<std::string> &min_path,std::vector<std::vector<std::string>> &paths)
    
    std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                        std::vector<std::string> location_ids) 

Similar to brute force approach, but as we are going through all possible permutations, we compare the current path's current traveled distance to the shortest possible distance we have so far. If we already exceed that minimum distance before hitting the end of this possible path, we stop looking further and start looking at the next possible ordering.

![output0_backtracking](https://user-images.githubusercontent.com/97922238/166184336-015df4c8-1d55-4aa2-8204-82ad4f97eaa9.gif)

A gif of our Early-tracking approach

2opt:

    std::vector<std::string> TrojanMap::two_opt_swap(std::vector<std::string> cur_path, int indexi, int indexk)
    std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
          std::vector<std::string> location_ids)

We start with 1 possible path, and reverse the orders of some of the nodes in the path to see if this ordering can  potentially give us a shorter travel distance.

![](https://lh3.googleusercontent.com/Nj27TAH8bTA46v50_Zvlw6k0G1F3yOUrk23DKFmZMf6ADh4H3cLN80Yvw7Oi7rfu-ECG-KJE1PHrmSHdnsuBDMMpmxjLZtzLJuWhGPqYNXnne2_EPOqGkVJ_PUDJXK2J6KukM9FFombQERrkow)


![output0_2opt](https://user-images.githubusercontent.com/97922238/166184352-28c36db9-56c7-493b-8422-7bb9a79ea265.gif)

A gif of our 2opt  approach


Figure 10. A demonstration of 2opt

One important characteristic of 2opt is, it is a heuristic which returns a good solution, but not necessarily the best solution.

The advantage of a heuristic is that, when the other approaches are taking too long to find the absolute shortest path, 2opt can give a good solution within a short amount of time that can be a good enough solution to the problem.

  

![](https://lh5.googleusercontent.com/FntEkJaXXPCC6PdBpztIKv8uIG1iEaCmefnUo64RRRnu82lzHUecni4ByQQ63VqhtAJamVh_ccPBVQZaEF05-ksF5YzaH-gtPxArPwNBRQnK8CJH_PBVSWYUX_K_kG0fCQzkU9NgPhNe-NHUrw)

Figure 11. Runtime comparison of the traveling salesman problem
  
  

![](https://lh6.googleusercontent.com/-yxTvCXBN36Hx7rtTU2wLy8GXF79WUdFtubZ88lH-j80Qk6nTEYKh-PFnZ0pPXxBzTZv6bH4B2rWk7xOrr9x5Zwq-mQkFmZR2KguN0j-4Wl5pCcCuSAcVq1ZhYzfVrTNTQbr8X573_AQDUdNeg)

  Figure 12: Runtime comparison of the traveling salesman problem

As we can see from the performance analysis, the 2opt path's distance is not too far away from the ground truth returned from brute force/early tracking.

  

**Cycle Detection**

The definition of a cycle in a graph is when all the edges are directed, and every node has at least one parent node pointing to it and a child node it points at.In the scope of this project, we use this function to find any cycles present within a subgraph. 

![](https://lh5.googleusercontent.com/60r14x4qMOywRiJQKJyWB9UcWmc_C3U55oTeHb_hbse-wdgZXLR_tiqk4u1tvxGNfGFuYM3t242RO88BG5oij6akxcl1PGlRfu1xn5fhngvj4FUPaQyZGT2pqwei761ZbI9Ve6qf70pytpz_lQ)

Figure 13. Flow chart of cycle detection

    bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) 
    bool TrojanMap::CycleDetectionHelper(std::string &id, std::unordered_map<std::string, bool> &isVisited, std::string parent, std::vector<std::string> &path) 

Is_cycle is a helper function.


**Topological sort**

The delivering Trojan problem is that we need to travel to a list of nodes, but these nodes have dependencies meaning that we need to travel to some of the nodes first, then the others. In this problem, we need to output a list containing the order of the nodes we need to travel to to satisfy the dependencies between them. Note that if there's a cycle, there is no possible topological order.

    void TrojanMap::Tsort(std::pair<std::string,bool> &m, std::vector<std::pair<std::string,bool>> &visited,std::vector<std::string> &stack, std::vector<std::vector<std::string>> &dependencies)
    
    std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                         std::vector<std::vector<std::string>> &dependencies)

  ![](https://lh6.googleusercontent.com/Ej3qV6eUa5m9xMp8s0pvU1Sf_0RC3U-w9Vvds0cMETA-1do_MpyJCH9vGDZ5yAEsVt6hV2WbgaUVoenCkq_7YyshqYa8UETxwBCy3xnZnNigH4tvirDo7l_zdtRD82UftO_bPVbArfmbtv7WDQ)

  Figure 14-1. An example dependency graph for topological sort
  
  
  ![](https://lh4.googleusercontent.com/jwY3UZBeVsNZzSUSiRkYm2j0ZeIbZSqA_icGnDIuczPeZSYBaektExHyKT6GVVvqEqHoI7fOnXGjfoXsfZ3woDYqp_S9V77AnJr32EX4HYMfkQX0Ahs2YOZCMztw-cZ41nNaec5qJOP4BArQ2w)
  
  Figure 14-2. The terminal output of the final topological order of such dependency 
  
  note that CFA = chick-fil-a, NES = Normandie Elementary School, TCA = The Caribbean Apartments
  
**Find nearby**

    std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) 
     
This function takes in a search radius, a location name, an attribute and a number x to return a list of x locations near the location entered within a radius.


**![](https://lh3.googleusercontent.com/RV0o4JWemSyJ4fnQ2k0wY_4L-jNbZ2roGiP94RYmMfyQ_ku7upxooQB7ZxS6VU5g-jS-4ZydiryiwJTtKkwilVoMdOD4dLS2miZ1czexggJxSpDz4PwnCnENbvfyXRodraFsRx-uOpd9K3edMg)**
  
  Figure 15. A full comparison of all functions

**UI**


![c4b9cbc66c03162f62d9a0880917c9e2266e3d5c](https://user-images.githubusercontent.com/97922238/166185495-729cb660-10d5-4de4-8eee-c873364e5c3f.gif)

A UI for the code was created using ncurses.

**Lessons Learned**

-   We practiced our skills in using version control software in collaboration
    
-   Heuristics: sometimes a good solution is enough
    
-   Sometimes the problem is more than finding the 1 correct answer: efficiency, space, stability all play an important role in software engineering. We want to know exactly what we are looking for and not let our vision be clouded by focusing on finding the 1 absolute best answer(even though a lot of times, that is the way.)


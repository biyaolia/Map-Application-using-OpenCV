#include "trojanmap.h"
#include <cctype>
#include <iostream>
#include <algorithm>
#include <float.h>
//#include "mapui.h"


//MapUI plot;
//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */

double TrojanMap::GetLat(const std::string& id) {
    std::unordered_map<std::string,Node>::const_iterator iter = this->data.find(id);
    if(iter == this->data.end()) return -1;
    else return iter->second.lat;

}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    std::unordered_map<std::string,Node>::const_iterator iter = this->data.find(id);
    if(iter == this->data.end()) return -1;
    else return iter->second.lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
    std::unordered_map<std::string,Node>::const_iterator iter = this->data.find(id);
    if(iter == this->data.end()) return "NULL";
    else return iter->second.name;
    return "NULL";
}

/*double TrojanMap::GetLat(const std::string& id) {
    std::unordered_map<std::string,Node>::const_iterator iter = this->data.find(id);
    if(iter == this->data.end()) return -1;
    else return iter->second.lat;
}
*/
/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    std::vector<std::string> results;
    std::unordered_map<std::string,Node>::const_iterator iter = this->data.find(id);
    if(iter == this->data.end()) return {};
    else return iter->second.neighbors;
    return {};
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */

std::string TrojanMap::GetID(const std::string& name) {
  std::string res = "";
   for (auto n : data){
      if (n.second.name == name ){
        return n.second.id;
      }
    }
    return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> emptyresults(-1, -1);
     for (auto n : this->data){
      if (n.second.name == name ){
        return std::pair<double,double> (n.second.lat, n.second.lon);
      }
    }
    return emptyresults;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
    int lena = a.length();
    int lenb = b.length();
    int **memo = new int*[lena+1]; // dynamic programming
    for(int i = 0; i<= lena;i++){
      memo[i] = new int [lenb+1]; // initializing memo array
    } 
    memo[0][0] = 0;
    for(int i = 1; i < lena+1; i++){
        memo[i][0] = i;
    }
    for (int i = 1; i < lenb + 1; i++) {
        memo[0][i] = i;
    }
    for (int i = 1; i < lena + 1; i++) {
      for (int j = 1; j < lenb + 1; j++) {
        if (a[lena - i] == b[lenb - j]) memo[i][j] = memo[i-1][j-1];
        else {
          memo[i][j] = 1 + std::min(memo[i][j - 1], std::min(memo[i - 1][j], memo[i - 1][j - 1]));
        }
      }
  }
  return memo[lena][lenb];
}
/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name){
  int difference = INT_MAX;
  std::string tmp = "";
  for (auto n : this->data){
        if(difference > CalculateEditDistance(n.second.name,name)){
            tmp = n.second.name;
            difference = CalculateEditDistance(n.second.name,name);
        }
  }
  return tmp;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
 bool auto_helper(std::string a, std::string b){ 
  
  for( int i = 0 ; i < a.size() ; i++){
       a[i] = tolower(a[i]);
      }
      
  for( int i = 0 ; i < b.size() ; i++){
       b[i] = tolower(b[i]);
      }

  int size = std::min(a.size(),b.size());
  
  for(int i = 0; i < size ; i++){
    if (a[i] != b[i] && int(a[i] - b[i])!= 32 && int(b[i] - a[i])!= 32){
        return false;
        }
  }
  return true;
}

std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  bool match = 0;
  for (auto n : this->data){
    if (n.second.name.size() >= name.size()){ //filter shorter results
        match = auto_helper(n.second.name,name);
        if(match)results.push_back(n.second.name);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}
struct Edge {
    std::string src, dst;
    int weight;
};

struct path_dist {
    std::vector<std::string> path;
    int weight;
};
int getindex(std::string &name,std::vector<std::string> &V){
  for (int i = 0 ; i<V.size(); i++){
    if(V[i] == name){
      return i;
    }
  }
  return -1;
}
/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
 
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::unordered_map<std::string, std::string> predecessor_map;
  std::unordered_map<std::string, double> distance_map;
  std::unordered_map<std::string, bool> visited; 
  std::string src = GetID(location1_name);
  std::string dst = GetID(location2_name);
  typedef std::pair<double, std::string> iPair;
  std::priority_queue< iPair, std::vector <iPair> , std::greater<iPair> > pq;
  std::string top_node;
  if ((src == "") | (dst == "")) {
    return path;
  }
  for (auto d : data){
    distance_map[d.second.id] = DBL_MAX;
    visited[d.second.id] = false;
    predecessor_map[d.second.id] = "null";
  }
  distance_map[src] = 0;
  pq.push({0,src});
 int i = 0;
  while(!pq.empty()){
    top_node = pq.top().second;
    visited[top_node] = true;
    pq.pop();
    i = i+1;
    for( auto n:GetNeighborIDs(top_node)){
      if(!visited[n]){
      double distance = distance_map[top_node] + CalculateDistance(n, top_node);
      //std::cout<<"src:\t"<<top_node<<"\t destination:\t"<<n<<"\tdistance\t"<<distance<<"\t distance_current:"<<CalculateDistance(n, top_node)<<"\n";
      if(distance_map[n] > distance){
        distance_map[n] = distance;
        predecessor_map[n] = top_node;
        pq.push({distance,n});
      }
      }
    }
    //if(i == 100){break;}
    if(top_node == dst){
      break;
    }
    //std::cout<<top_node<<"\n";
  }
  //std::cout<<predecessor_map[dst]<<"\n";
  
  if (distance_map[dst] == DBL_MAX) {
    return path;
  } 
  //std::cout<<predecessor_map[dst]<<"\n"<<predecessor_map[predecessor_map[dst]]<<"\n";
  for (auto parent = dst; parent != src; parent = predecessor_map[parent]) {
    path.push_back(parent);
  }
  path.push_back(src);
  std::reverse(path.begin(), path.end());

  return path;
}
/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */

std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(std::string location1_name, std::string location2_name){
  bool flag;
  std::string node;
  std::vector<std::string> path;
  std::unordered_map<std::string, std::string> parent_map;
  std::unordered_map<std::string, double> distance_map; 
  std::string start = GetID(location1_name);
  std::string goal = GetID(location2_name);
  
  if (start == "" || goal == "") {
    return path;
  }

  for (auto d : data){
    distance_map[d.first] = DBL_MAX;
  }

  distance_map[start] = 0; 
  for (int i = 0; i < data.size() - 1; i++) { 
    flag = false;
    for (auto &d : data) {
      node = d.first;
      for (auto &n : d.second.neighbors) { 
        auto distance = distance_map[n] + CalculateDistance(n, d.second.id);
        if (distance_map[node] >  distance) {
          distance_map[node] = distance;
          parent_map[node] = n;
          flag = true;
        }
      }
    }
    if (!flag) {
      break;
    }
  }

  if (distance_map[goal] == DBL_MAX) {
    return path;
  } 

  for (auto parent = goal; parent != start; parent = parent_map[parent]) {
    path.push_back(parent);
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());

  return path;
}
/*void TrojanMap::bellman_ford_helper(std::unordered_map < std::string, int> &distance_map, std::unordered_map < std::string, std::string> &parent_map, std::string &start){
for (auto d : data){
    distance_map[d.first] = DBL_MAX;
  }

  distance_map[start] = 0; 
  for (int i = 0; i < data.size() - 1; i++) { 
    bool flag = false;
    for (auto &d : data) {
      std::string node = d.first;
      for (auto &neighbor : d.second.neighbors) { 
        auto distance = distance_map[neighbor] + CalculateDistance(neighbor, d.second.id);
        if (distance_map[node] >  distance) {
          distance_map[node] = distance;
          parent_map[node] = neighbor;
          flag = true;
        }
      }
    }
    if (!flag) {
      break;
    }
  }
   }
 
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
  std::string location1_name, std::string location2_name){
  std::vector<std::string> path;
  std::unordered_map < std::string, int> distance;
  std::unordered_map < std::string, std::string> predecesor;
  std::string src = GetID(location1_name);
  std::string dst = GetID(location2_name);
  if( (src == "") | (dst == "")){
    return {};
  }
  
  bellman_ford_helper(distance,predecesor,src);

  if(distance[dst] == DBL_MAX){
    return {};
  }
  for(auto parent = dst; parent !=src ; parent = predecesor[parent]){
    path.push_back(parent);
  }
  path.push_back(src);
  std::reverse(path.begin(),path.end());
  return path;
}
*/
/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::vector<std::vector<std::string>> paths;
  std::vector<std::string> temp;
  std::vector<std::string> path_fin;
  std::vector<std::string> location;
  std::pair<double, std::vector<std::vector<std::string>>> records;
    if(location_ids.size()<2){
    return {};
  }
  double distance = INT_MAX;
  location = location_ids;
  location.erase(location.begin());

  std::sort(location.begin(),location.end());
  do {
        temp = location;
        temp.insert(temp.begin(),location_ids.front());
        temp.push_back(location_ids.front());
        paths.push_back(temp);
    } while (std::next_permutation(location.begin(), location.end()));
        
    for(auto n:paths){
      if(CalculatePathLength(n)<distance){
        distance = CalculatePathLength(n);
        path_fin = n;
      }
    }
    paths.push_back(path_fin);
  records = {distance,paths};
  return records;
}


void TrojanMap::TTB_Aux(std::string start,std::vector<std::string> location_ids,std::string cur_node,
  double cur_dist, std::vector<std::string> &cur_path, double &min_dist, std::vector<std::string> &min_path,std::vector<std::vector<std::string>> &paths){
  
  if(cur_path.size() == location_ids.size()){
  // full path = ABCA for locations ABC
  //if (cur_path.size() == distances.size()){
    int final_distance = cur_dist + CalculateDistance(start,cur_node);
    if (final_distance < min_dist){
      min_dist = final_distance;
      min_path = cur_path;
      paths.push_back(cur_path);
    }
    return;
  }
  if (cur_dist >= min_dist){
    return; // path is too long
  }
  for(auto n: location_ids){
    if(std::find(cur_path.begin(),cur_path.end(),n)==cur_path.end()){ // node isnt visited yet
        cur_path.push_back(n);
        TTB_Aux(cur_node,location_ids,n,cur_dist+ CalculateDistance(start,cur_node),cur_path,min_dist,min_path,paths);
        cur_path.pop_back();
    }    
  }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
std::vector<std::string>cur_path = {}; // {start}
std::vector<std::string>min_path;
std::vector<std::vector<std::string>> paths;
std::vector<std::vector<std::string>> paths_all;
std::vector<std::string> path_fin;
std::vector<std::string> temp;
double min_dist = DBL_MAX;
std::pair<double, std::vector<std::vector<std::string>>> results;
  if(location_ids.size()<2){
    return {};
  }
TTB_Aux(location_ids[0],location_ids,location_ids[0],0,cur_path,min_dist,min_path,paths);

min_dist = DBL_MAX;
  for(auto n : paths){
    temp = n;
    temp.push_back(location_ids[0]);
    paths_all.push_back(temp);
  }

  for(auto n:paths_all){
      if(CalculatePathLength(n) < min_dist){
        //std::cout<<CalculatePathLength(n)<<"z\n";
        min_dist = CalculatePathLength(n);
        path_fin = n;
      }
    }

  paths_all.push_back(path_fin);
  results = {min_dist, paths_all};
  return results;
}

std::vector<std::string> TrojanMap::two_opt_swap(std::vector<std::string> cur_path, int indexi, int indexk){
  std::vector<std::string> new_path;
  for (int i = 0; i <= indexi - 1; i++){
    new_path.push_back(cur_path[i]);
  }
  for (int i = indexk; i>= indexi; i--){
    new_path.push_back(cur_path[i]);
  }
  if (indexk+1 <= cur_path.size()-1){
     for (int i = indexk+1; i< cur_path.size(); i++){
      new_path.push_back(cur_path[i]);
      }
  }
  return new_path;
}
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::vector<std::string>cur_path;
  std::vector<std::vector<std::string>> paths;
  std::vector<std::vector<std::string>> paths_all;
  std::vector<std::string> path_fin;
  std::vector<std::string> temp;
  std::vector<std::string>min_path;
std::pair<double, std::vector<std::vector<std::string>>> results;
  double min_dist = 0;
  double new_dist = 0;
  if(location_ids.size()<2){
    return {};
  }
  if(location_ids.size()==2){
    temp.push_back(location_ids[0]);
    temp.push_back(location_ids[1]);
    temp.push_back(location_ids[0]);
    paths_all.push_back(temp);
    min_dist = CalculatePathLength(temp);
    results = {min_dist,paths_all};
    return results;
    
  }
  cur_path.push_back(location_ids[0]);
  for (int i = 1; i < location_ids.size(); i++){
    cur_path.push_back(location_ids[i]);
    min_dist += CalculateDistance(location_ids[i-1],location_ids[i]);
  }
min_dist += CalculateDistance(location_ids[location_ids.size()-1],location_ids[0]); // add distance from last node back to start
cur_path.push_back(location_ids[0]); // now we have a path to work with
  // initializations done
    for (int i = 1; i <= location_ids.size()-1;i++){
      for (int k = i + 1; k <= location_ids.size()-1; k++){
          std::vector<std::string>new_path;
          new_dist = 0;
          new_path = two_opt_swap(cur_path,i,k);
          for (int j = 0; j < new_path.size()-2; j++){
            new_dist += CalculateDistance(new_path[j], new_path[j+1]);
          } 
          if (new_dist < min_dist){
            cur_path.assign(new_path.begin(),new_path.end());
            paths.push_back(cur_path);
            min_dist = new_dist; 
          }
      }
    }
 min_dist = DBL_MAX;
  
  for(auto n : paths){
    temp = n;
    //std::cout<<"\n";
    //temp.insert(temp.begin(),location_ids[0]);
    for(auto m : temp){
      //std::cout<<m<<"\n";
    }
    paths_all.push_back(temp);
  }

 for(auto n:paths_all){
      if(CalculatePathLength(n) < min_dist){
        min_dist = CalculatePathLength(n);
        path_fin = n;
      }
    }
    paths_all.push_back(path_fin);
    results = {min_dist,paths_all};
return results;
}


std::vector<std::string> TrojanMap::three_opt_swap(std::vector<std::string> cur_path, int indexi, int indexk){
  std::vector<std::string> new_pathA;
  std::vector<std::string> new_pathB;
  std::vector<std::string> new_pathC;
  std::vector<std::string> new_pathD;
  std::vector<std::string> Bback;
  std::vector<std::string> Bforw;
  std::vector<std::string> Cback;
  std::vector<std::string> Cforw;
  
  double cost[4];
  
  for (int i = 0; i <= indexi - 1; i++){
    new_pathA.push_back(cur_path[i]);
    new_pathB.push_back(cur_path[i]);
    new_pathC.push_back(cur_path[i]);
    new_pathD.push_back(cur_path[i]);
  }
  
  for (int i = indexk; i>= indexi; i--){
    Bback.push_back(cur_path[i]);
    Bforw.push_back(cur_path[i]);
  }
  
  std::reverse(Bforw.begin(),Bforw.end());

  for (int i = indexk+1; i<cur_path.size(); i++){
    Cforw.push_back(cur_path[i]);
    Cback.push_back(cur_path[i]);  
  }
  std::reverse(Cback.begin(),Cback.end());
  
  new_pathA.insert(new_pathA.end(),Bback.begin(),Bback.end());
  new_pathA.insert(new_pathA.end(),Cback.begin(),Cback.end());
  
  new_pathB.insert(new_pathB.end(),Cforw.begin(),Cforw.end());
  new_pathB.insert(new_pathB.end(),Bforw.begin(),Bforw.end());
  
  new_pathC.insert(new_pathC.end(),Cforw.begin(),Cforw.end());
  new_pathC.insert(new_pathC.end(),Bback.begin(),Bback.end());
  
  new_pathD.insert(new_pathC.end(),Cback.begin(),Cback.end());
  new_pathD.insert(new_pathC.end(),Bforw.begin(),Bforw.end());

  for (int i = 0; i < new_pathA.size()-2;i++){
    cost[0] += CalculateDistance(new_pathA[i], new_pathA[i+1]);
    cost[1] += CalculateDistance(new_pathB[i], new_pathB[i+1]);
    cost[2] += CalculateDistance(new_pathC[i], new_pathC[i+1]);
    cost[3] += CalculateDistance(new_pathD[i], new_pathD[i+1]);
  }
  
  if (cost[0]<= cost[1] && cost[0]<= cost[2] && cost[0]<= cost[3]) return new_pathA;
  else if(cost[1]<= cost[0] && cost[1]<= cost[2] && cost[1]<= cost[3]) return new_pathB;
  else if(cost[2]<= cost[0] && cost[2]<= cost[1] && cost[2]<= cost[3]) return new_pathC;
  else if(cost[3]<= cost[0] && cost[3]<= cost[1] && cost[3]<= cost[2])return new_pathD;
  else return {};

}
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
        std::vector<std::string> location_ids){
 std::vector<std::string>cur_path;
  std::vector<std::vector<std::string>> paths;
  std::vector<std::vector<std::string>> paths_all;
  std::vector<std::string> path_fin;
  std::vector<std::string> temp;
  double min_dist = 0;
  double new_dist = 0;
  cur_path.push_back(location_ids[0]);
  for (int i = 1; i < location_ids.size(); i++){
    cur_path.push_back(location_ids[i]);
    min_dist += CalculateDistance(location_ids[i-1],location_ids[i]);
  }
min_dist += CalculateDistance(location_ids[location_ids.size()-1],location_ids[0]); // add distance from last node back to start
cur_path.push_back(location_ids[0]); // now we have a path to work with
std::vector<std::string>min_path;
std::pair<double, std::vector<std::vector<std::string>>> results;
  // initializations done
    for (int i = 1; i <= location_ids.size()-1;i++){
      for (int k = i + 1; k <= location_ids.size()-1; k++){
          std::vector<std::string>new_path;
          new_dist = 0;
          new_path = three_opt_swap(cur_path,i,k);
          for (int j = 0; j < new_path.size()-2; j++){
            new_dist += CalculateDistance(new_path[j], new_path[j+1]);
          } 
          if (new_dist < min_dist){
            cur_path.assign(new_path.begin(),new_path.end());
            paths.push_back(cur_path);
            min_dist = new_dist; 
          }
        }  
      }
 min_dist = DBL_MAX;
  
  for(auto n : paths){
    temp = n;
    //std::cout<<"\n";
    //temp.insert(temp.begin(),location_ids[0]);
    for(auto m : temp){
      //std::cout<<m<<"\n";
    }
    paths_all.push_back(temp);
  }

    results = {min_dist,paths_all};
return results;
}

/*
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */

std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::fstream inputfile;
  std::string line;

  inputfile.open(locations_filename,std::ios::in);
  while(std::getline(inputfile,line)){
    location_names_from_csv.push_back(line);
  }
 
  location_names_from_csv.erase(location_names_from_csv.begin());
  return location_names_from_csv;
}
/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;  
  std::fstream inputfile;
  inputfile.open(dependencies_filename,std::ios::in);
  std::string line,field,word;

  while(std::getline(inputfile,line)){
    std::istringstream s(line);
    std::vector<std::string> entry;
    while(std::getline(s,word,',')){
      entry.push_back(word);
    }
    dependencies_from_csv.push_back(entry);
  }
 
  dependencies_from_csv.erase(dependencies_from_csv.begin());
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */

void TrojanMap::Tsort(std::pair<std::string,bool> &m, std::vector<std::pair<std::string,bool>> &visited,std::vector<std::string> &stack, std::vector<std::vector<std::string>> &dependencies){
  m.second = true;
  //std::cout<<"Tsort\n";
  for (auto n: dependencies){
    if (n[0] == m.first){
      for (auto k:visited){
        if(k.first == n[1] && k.second == false){
          Tsort(k,visited,stack,dependencies);
        }
      }
    }
  }
  if ( (std::find(stack.begin(), stack.end(), m.first) == stack.end()) ){
  stack.push_back(m.first);
  //std::cout<<m.first<<std::endl;
  }
}
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::vector<std::string> stack;
  std::vector<std::pair<std::string,bool>> visited;
  //mark all locations as unvisited
  for(auto n:locations){
    visited.push_back(std::pair<std::string,bool>{n,false});
  }
  for(auto m:visited){
    if(m.second == false){
      Tsort(visited[0],visited,stack,dependencies);
    }
  }
  //while(stack.empty()== false){
  //  result.push_back(stack.top());
    //stack.erase();
  //}
  std::reverse(stack.begin(),stack.end());
  return stack;                                                     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  auto lat = GetLat(id);
  auto lon = GetLon(id);
  return square[3] < lat && square[2] > lat && square[0] < lon && square[1] > lon;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph; // lat, lon, lat lon
  double minlat,maxlat,minlon,maxlon;
  minlat = (square[2]>square[3]) ? (square[3]):(square[2]);
  maxlat = (square[2]>square[3]) ? (square[2]):(square[3]);
  minlon = (square[0]>square[1]) ? (square[1]):(square[0]);
  maxlon = (square[0]>square[1]) ? (square[0]):(square[1]);
  for (auto n : this->data){
    if (n.second.lat <= maxlat && n.second.lat >= minlat){
      if (n.second.lon <= maxlon && n.second.lon >= minlon){
        subgraph.push_back(n.second.id);
      }
    }
  }
  //std::cout<<"min lat\t"<<minlat<<"\tmax lat\t"<<maxlat<<"\tmin lon\t"<<minlon<<"\tmax lat\t"<<maxlon<<"\n";
  return subgraph;
}
bool TrojanMap::CycleDetectionHelper(std::string &id, std::unordered_map<std::string, bool> &isVisited, std::string parent, std::vector<std::string> &path) {
  isVisited[id] = true;
  for (auto neighbor : GetNeighborIDs(id)) {
    if (isVisited.find(neighbor) != isVisited.end()) {
      if (!isVisited[neighbor]) {
        path.push_back(neighbor);
        if (CycleDetectionHelper(neighbor, isVisited, id, path)) {
          return true;
        }
        path.pop_back();
      }
      else if (isVisited[neighbor] && neighbor != parent) {
        path.push_back(neighbor);
        return true;
      }
    }
  }
  return false;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  std::unordered_map<std::string, bool> isVisited;
  std::vector<std::string> path; 
  for (auto id : data) {
    if (inSquare(id.first, square)) {
      isVisited[id.first] = false;
    }
  }
  for (auto id : subgraph) {
    if (!isVisited[id]) {
      std::string parent;
      path.push_back(id);
      if (CycleDetectionHelper(id, isVisited, parent, path)) {
        return true;
      }
    }
  }
  return false;
}
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square,  std::vector<std::string> &path) {
  std::unordered_map<std::string, bool> isVisited; 
  for (auto id : data) {
    if (inSquare(id.first, square)) {
      isVisited[id.first] = false;
    }
  }
  for (auto id : subgraph) {
    if (!isVisited[id]) {
      std::string parent;
      path.push_back(id);
      if (CycleDetectionHelper(id, isVisited, parent, path)) {
        for(auto n:path){
          //std::cout<<n<<std::endl;
        }
        return true;
      }
    }
  }
  return false;
}

/*
/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
/*
bool TrojanMap::iscycle(std::string src,std::string parent,std::unordered_map<std::string,bool> &visited,std::unordered_map<std::string,std::vector<std::string>> &adj,std::vector<std::string> &cycle_nodes){
  visited[src] = true;
  //std::cout<<src<<"\tsrc\t"<<parent<<"\tparent\n";
  for(auto i : adj[src]){
      //std::cout<<i<<"\n";
    if(i!=parent){
      if(visited[i]){
        //std::cout<<"true 1\n";
        cycle_nodes.push_back(i);
        return true;

      }
      if(!visited[i] & iscycle(i,src,visited,adj,cycle_nodes)){
        //std::cout<<"true 1\n";
        cycle_nodes.push_back(i);
        return true;
      }
    }
  return false;
  }
}

bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
   std::vector<std::string> neighbours;
   std::unordered_map<std::string,bool> visited;
   std::vector<std::string> cycle_nodes;
   std::vector<std::string> temp;
   std::unordered_map<std::string,std::vector<std::string>> adj;
   for(auto n : subgraph){
     neighbours = IsInSubgraph(square,GetNeighborIDs(n));
     //map[subgraph[i]].neighbors = GetNeighborIDs(subgraph[i]);
     adj[n] =  neighbours;
     visited[n] = false;
    }

     for(auto n : subgraph){
       //std::cout<<n<<"\tn\t"<<visited[n]<<"\n";
       if(!visited[n]){
         for(auto m:subgraph){
           visited[m] = false;
         }
         if(iscycle(n,"null",visited,adj,cycle_nodes)){
           cycle_nodes.push_back(n);
           //for(auto n : cycle_nodes){
           // std::cout<<n<<std::endl;
           //}
           //plot.PlotPathandSquare(cycle_nodes,square);
           return true;
         }
       }
     }
//plot.PlotPointsandEdges(cycle_nodes, square);
return false;
}
*/
/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  std::unordered_set<std::string> attributes;
  typedef std::pair<double, std::string> iPair;
  std::priority_queue< iPair, std::vector <iPair> , std::greater<iPair> > pq;

  for(auto n : this->data){
    if(!n.second.attributes.empty() & n.second.name != name){
      if (n.second.attributes.find(attributesName) != n.second.attributes.end()) {
        if(CalculateDistance(GetID(name),n.second.id) < r ){
          pq.push({CalculateDistance(GetID(name),n.second.id),n.second.id});
        }
      }
    }
  }

  while(!pq.empty()&& res.size()<k){
    res.push_back(pq.top().second);
    pq.pop();
  }
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

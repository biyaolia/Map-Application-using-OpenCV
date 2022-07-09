#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"
/*TEST(ReadLocationsFromCSVFile,Test1){
  TrojanMap *test = new TrojanMap();
  std::vector<std::string> testing,testing2;
  //testing.push_back("Name");
  testing.push_back("Ralphs");
  testing.push_back("KFC");
  testing.push_back("Chick-fil-A");
  testing2=test->ReadLocationsFromCSVFile("/home/manoj/Desktop/fin_proj/final-project-ManojSunkara97/input/topologicalsort_locations.csv");
  EXPECT_EQ(testing,testing2);
}
TEST(ReadDependenciesFromCSVFile,Test1){
  TrojanMap *test = new TrojanMap();
  std::vector<std::vector<std::string>> testing,testing2;
  //testing.push_back("Name");
  testing.push_back({"Ralphs","Chick-fil-A"});
  testing.push_back({"Ralphs","KFC"});
  testing.push_back({"Chick-fil-A","KFC"});
  testing2 =test->ReadDependenciesFromCSVFile("/home/manoj/Desktop/fin_proj/final-project-ManojSunkara97/input/topologicalsort_dependencies.csv");
  EXPECT_EQ(testing,testing2);
}
TEST(threeopt,Test1){
  TrojanMap *test = new TrojanMap();
  std::vector<std::vector<std::string>> testing,testing2;
  testing.push_back({"Ralphs","Chick-fil-A"});
  testing.push_back({"Ralphs","KFC"});
  testing.push_back({"Chick-fil-A","KFC"});
  testing2 =test->ReadDependenciesFromCSVFile("/home/manoj/Desktop/fin_proj/final-project-ManojSunkara97/input/topologicalsort_dependencies.csv");
  EXPECT_EQ(testing,testing2);
}
*/

TEST(TrojanMapStudentTest, Test1) {
  TrojanMap *test = new TrojanMap();
  double lat,lon;
  std::string ID;
  std::string name;
  std::string closename = "";
  std::pair<double,double> location;
  lat=test->GetLat("732641023");
  lon=test->GetLon("732641023");
  name = test->GetName("732641023");
  ID = test->GetID("Chipotle");
  location = test->GetPosition(name);
  EXPECT_EQ(name,"Chipotle");
  EXPECT_EQ(ID,"732641023");
  EXPECT_EQ(location.first,lat);
  EXPECT_EQ(location.second,lon);
}

TEST(TrojanMapTest, Autocomplete1) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Chi");
  std::unordered_set<std::string> gt = {"Chick-fil-A", "Chipotle", "Chinese Street Food"}; // groundtruth for "Ch"
  int success = 0;
  for (auto n: names) {
    EXPECT_EQ(gt.count(n) > 0, true);
    if (gt.count(n) > 0){
      success++;
    }
  }
}

TEST(TrojanMapTest, Autocomplete2) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete(" ");
  std::unordered_set<std::string> gt = {};
  int success = 0;
  for (auto n: names) {
    EXPECT_EQ(gt.count(n) > 0, true);
    if (gt.count(n) > 0){
      success++;
    }
  }
}

TEST(TrojanMapTest, Autocomplete3) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("TAR");
  std::unordered_set<std::string> gt = {"Target"};
  int success = 0;
  for (auto n: names) {
    EXPECT_EQ(gt.count(n) > 0, true);
    if (gt.count(n) > 0){
      success++;
    }
  }
}

TEST(TrojanMapTest, Autocomplete4) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("c");
  std::unordered_set<std::string> gt = {"Chinese Street Food","Chase","Cognoscenti Coffee Roastery","Chucks Chicken & Waffles","Cheebos Burger","Cindys Shoes","Cosmo Plaza","Corwin Denney Research Center","Central Church of Seventh Day Adventists","Callejon All Star","Chevron","Chase Plaza Heliport","CAVA","California Family Pharmacy","Community of Christ","Chipotle","Chevron 2","Cover the Homeless Ministry","Church of Christ","CVS Pharmacy","City Tacos","Chevron 1","Central Adult Senior High","Carson Center","Chick-fil-A","Crosswalk","CorePower Yoga","City of Angels Independent Studies School","Cal Mart Beer & Wine Food Store","Crosswalk 1"}; // groundtruth for "Ch"
  int success = 0;
  for (auto n: names) {
    EXPECT_EQ(gt.count(n) > 0, true);
    if (gt.count(n) > 0){
      success++;
    }
  }
}
TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName(""), "");
}
TEST(TrojanMapTest, FindClosestName2) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("targat"), "Target");
}
TEST(TrojanMapTest, FindClosestName3) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("ChicfilA"), "Chick-fil-A");
}

TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698","4015377690","4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229","122719216","6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
   auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698","4015377690","4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229","122719216","6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Chipotle");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919","6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145","6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785","6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809","4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483","7221497057","6352865695","4872897510","6915916494","6819179782","1732243889","6820935945","1732243586","6820935946","6814620861","2738732233","2700459348","4399697326","1732243587","4399697313","2613066338","4540763383","1673644081","2613066358","2613066270","6813379551","6814916524","1732243544","6813405275","348121996","348121864","6813405280","1472141024","6813411590","216155217","6820935908","9446678100","732641023"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Chipotle", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford2) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
   auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Chipotle");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919","6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145","6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785","6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809","4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483","7221497057","6352865695","4872897510","6915916494","6819179782","1732243889","6820935945","1732243586","6820935946","6814620861","2738732233","2700459348","4399697326","1732243587","4399697313","2613066338","4540763383","1673644081","2613066358","2613066270","6813379551","6814916524","1732243544","6813405275","348121996","348121864","6813405280","1472141024","6813411590","216155217","6820935908","9446678100","732641023"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Chipotle", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra3) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Target","Parking Center");
  std::vector<std::string> gt{
      "5237417650","6814769289","6813379584","6813360961","6813379480","6813360960","6814620882","6813360954","6813360952","6813379420","6813360951","6813360936","6813379467","6813379466","21306060","6813379469","6813379427","123005255","6807200376","6807200380","6813379451","6813379463","123327639","6813379460","4141790922","4015423963","9559739235","1286136447","1286136422","4015423962","6813379494","63068643","6813379493","4015423961","4015423960","7860380185","4015423959","9591448314","1841835270","2193435032","1378231753","9591449436","123292100","123292045","6939732877","123292047","4818896620","4818921126","6816355554","4818921121","732642214"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Parking Center","Target");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford3) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
   auto path = m.CalculateShortestPath_Dijkstra("Target","Parking Center");
  std::vector<std::string> gt{
      "5237417650","6814769289","6813379584","6813360961","6813379480","6813360960","6814620882","6813360954","6813360952","6813379420","6813360951","6813360936","6813379467","6813379466","21306060","6813379469","6813379427","123005255","6807200376","6807200380","6813379451","6813379463","123327639","6813379460","4141790922","4015423963","9559739235","1286136447","1286136422","4015423962","6813379494","63068643","6813379493","4015423961","4015423960","7860380185","4015423959","9591448314","1841835270","2193435032","1378231753","9591449436","123292100","123292045","6939732877","123292047","4818896620","4818921126","6816355554","4818921121","732642214"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Parking Center","Target");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.290, 34.020, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

    // Test case 2
  std::vector<double> square2 = {-118.320, -118.250, 34.000, 34.040};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, true);

  // Test case 2
  std::vector<double> square3 = {-118.290, -118.289, 34.025, 34.023};
  auto sub3 = m.GetSubgraph(square3);
  bool result3 = m.CycleDetection(sub3, square3);
  EXPECT_EQ(result3, false);
}
TEST(TrojanMapTest, TSP_BF1) {
  TrojanMap m;
  
  std::vector<std::string> input{"8858977768","1832241490"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"8858977768","1832241490","8858977768"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP_BF2) {
  TrojanMap m;
  
  std::vector<std::string> input{"1630951196","4399914038","123322116","4273778142"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1630951196","123322116","4273778142","4399914038","1630951196"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP_BF3) {
  TrojanMap m;
  
  std::vector<std::string> input{"1855169983","1862286775","6820982916","7281805575","3285242854"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1855169983","7281805575","6820982916","1862286775","3285242854","1855169983"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_bf_bt1) {
  TrojanMap m;
  
  std::vector<std::string> input{"354063321","8157392447"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"354063321","8157392447","354063321"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_bf_bt2) {
  TrojanMap m;
  
  std::vector<std::string> input{"4063107778","6818390168","7823283216","2700467936"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4063107778","6818390168","2700467936","7823283216","4063107778"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP_bf_bt3) {
  TrojanMap m;
  
  std::vector<std::string> input{"8344672873","6788017719","4400281327","1838283855","6278441227"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"8344672873","6788017719","4400281327","1838283855","6278441227","8344672873"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2opt1) {
  TrojanMap m;
  
  std::vector<std::string> input{"354063321","8157392447"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"354063321","8157392447","354063321"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP2opt2) {
  TrojanMap m;
  
  std::vector<std::string> input{"4063107778","6818390168","7823283216","2700467936"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4063107778","7823283216","6818390168","2700467936","4063107778"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP2opt3) {
  TrojanMap m;
  
  std::vector<std::string> input{"8344672873","6788017719","4400281327","1838283855","6278441227"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"8344672873","4400281327","6788017719","1838283855","6278441227","8344672873"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TopologicalSort1) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {
    "Ralphs",
    "KFC",
"Chick-fil-A",
"Chipotle",
"Parking Center",
"Normandie Elementary School",
"Leavey Library",
"USC Village Gym",
"USC Parking",
"The Caribbean Apartments"
  };
  std::vector<std::vector<std::string>> dependencies = {
    {"Ralphs","Chick-fil-A"},
{"Ralphs","KFC"},
{"Chick-fil-A","KFC"},
{"KFC","Chipotle"},
{"Chipotle","USC Parking"},
{"USC Parking","Normandie Elementary School"},
{"Chipotle","Normandie Elementary School"},
{"Normandie Elementary School","Leavey Library"},
{"USC Parking","USC Village Gym"},
{"Leavey Library","USC Village Gym"},
{"USC Village Gym","Parking Center"},
{"Parking Center","The Caribbean Apartments"}
  };
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs",
"Chick-fil-A",
"KFC",
"Chipotle",
"USC Parking",
"Normandie Elementary School",
"Leavey Library",
"USC Village Gym",
"Parking Center",
"The Caribbean Apartments"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSort2) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {
    "Ralphs",
    "KFC",
"Chick-fil-A",
"Chipotle",
"Parking Center",
"Normandie Elementary School",
"Leavey Library",
"USC Village Gym",
"USC Parking",
"The Caribbean Apartments"
  };
  std::vector<std::vector<std::string>> dependencies = {
    {"Ralphs","Chick-fil-A"},
{"Ralphs","KFC"},
{"Chick-fil-A","KFC"},
{"Chipotle","USC Parking"},
{"USC Parking","Normandie Elementary School"},
{"Chipotle","Normandie Elementary School"},
{"USC Parking","USC Village Gym"},
{"Leavey Library","USC Village Gym"},
{"USC Village Gym","Parking Center"},
{"Parking Center","The Caribbean Apartments"}
  };
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs","Chick-fil-A", "KFC"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSort3) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {
    "Ralphs",
    "KFC",
"Chick-fil-A",
"Chipotle",
"Parking Center",
"Normandie Elementary School",
"Leavey Library",
"USC Village Gym",
"USC Parking",
"The Caribbean Apartments"
  };
  std::vector<std::vector<std::string>> dependencies = {
{"Ralphs","Chick-fil-A"},
{"Chick-fil-A","KFC"},
{"KFC","Chipotle"},
{"Chipotle","USC Parking"},
{"USC Parking","Normandie Elementary School"},
{"Chipotle","Normandie Elementary School"},
{"Normandie Elementary School","Leavey Library"},
{"USC Parking","USC Village Gym"},
{"Leavey Library","USC Village Gym"},
{"USC Village Gym","Parking Center"},
{"Parking Center","The Caribbean Apartments"}
//,{"Normandie Elementary School","Chipotle"}
  };
   auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs",
"Chick-fil-A",
"KFC",
"Chipotle",
"USC Parking",
"Normandie Elementary School",
"Leavey Library",
"USC Village Gym",
"Parking Center",
"The Caribbean Apartments"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, FindNearby1) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 10);
  std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby2) {
  TrojanMap m;
  
  auto result = m.FindNearby("school", "Target", 3, 10);
  std::vector<std::string> ans{ m.GetID("Divine Providence Kindergarten and Day Nursery"),
m.GetID("Vermont Elementary School"),
m.GetID("Saint Agnes Elementary School"),
m.GetID("National Schools"),
m.GetID("Washington Boulevard School"),
m.GetID("Central Adult Senior High"),
m.GetID("Trinity Elementary School"),
m.GetID("Twenty-Eight Street Elementary School"),
m.GetID("Foshay Learning Center"),
m.GetID("West Vernon Elementary School"),
};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby3) {
  TrojanMap m;
  auto result = m.FindNearby("restaurant", "Ralphs", 5, 0);
  std::vector<std::string> ans{};
  EXPECT_EQ(result, ans);
}
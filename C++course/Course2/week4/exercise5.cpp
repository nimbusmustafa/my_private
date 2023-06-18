#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line

vector<string> data;

try {
  ifstream file;
  string read;
  file.open(path);
  if (!file) {
    throw runtime_error("File failed to open.");
  }
  while (getline(file, read)) {
    stringstream ss(read);
    while (getline(ss, read, ',')) {
      data.push_back(read);
    }
  }
  file.close();
}
  
catch (exception& e) {
  cerr << e.what() << endl;
}
  
string cities;
cout << "The following cities are in the Southern Hemisphere: ";
  
for (int i = 6; i < data.size(); i+=4) {
  if (stoi(data.at(i)) < 0) {
    cities += (data.at(i - 2) + ", ");
  }
}
  
cities.pop_back();
cities.pop_back();
cities += ".";
  
cout << cities;


  //add code above this line
  
  return 0;
  
}

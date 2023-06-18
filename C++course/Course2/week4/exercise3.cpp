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
    throw runtime_error("File failed to open.");}
  while (getline(file, read)) {
    stringstream ss(read);
    while (getline(ss, read)) {
      data.push_back(read);
    }
  }
  file.close();
}
  catch (exception& e) {
  cerr << e.what() << endl;}
  
for (int i = data.size() - 1; i >= 0; i--) {
  cout << data.at(i) << endl;
}



  //add code above this line
  
  return 0;
  
}

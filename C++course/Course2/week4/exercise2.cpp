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
  
int col1 = 0;
int col2 = 0;
int col3 = 0;
int col4 = 0;
for (int i = 0; i < data.size(); i++) {
  if (i == 0 || i == 4 | i == 8) {
    col1 += stoi(data.at(i));
  }
  if (i == 1 || i == 5 | i == 9) {
    col2 += stoi(data.at(i));
  }
  if (i == 2 || i == 6 | i == 10) {
    col3 += stoi(data.at(i));
  }
  if (i == 3 || i == 7 | i == 11) {
    col4 += stoi(data.at(i));
  }
}
  
cout << col1 / 3 << " ";
cout << col2 / 3 << " ";
cout << col3 / 3 << " ";
cout << col4 / 3;


  //add code above this line
  
  return 0;
  
}

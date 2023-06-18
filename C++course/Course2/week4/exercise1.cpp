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

try {
  ifstream file;
  string read;
  int lines = 0;
  int chars = 0;
  file.open(path);
  if (!file) {
    throw runtime_error("File failed to open.");
  }
  while (getline(file, read)) {
    lines++;
    chars += read.length();
  }
  file.close();
  cout << lines << " line(s)" << endl;
  cout << chars << " character(s)"; 
}
  
catch (exception& e) {
  cerr << e.what() << endl;
}
  
  //add code above this line
  
  return 0;
  
}
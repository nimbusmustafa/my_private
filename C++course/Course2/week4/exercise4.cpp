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
    while (getline(ss, read, '\t')) {
      data.push_back(read);
    }
  }
  file.close();
}
  
catch (exception& e) {
  cerr << e.what() << endl;
}
  
int max = 0;
string person;
  
for (int i = 1; i < data.size(); i+=3) {
  if (stoi(data.at(i)) > max) {
    max = stoi(data.at(i));
    person = data.at(i - 1);
  }
}
  
cout << "The oldest person is " << person << ".";


  //add code above this line
  
  return 0;
  
}

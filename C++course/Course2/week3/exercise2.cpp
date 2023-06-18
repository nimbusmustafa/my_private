#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line
int l= my_string.length();
for (int i=0;i<l;i++){
  for(int j=0;j<l;j++){
    cout << my_string ;
  }
  cout << endl;
}


  //add code above this line
  
  return 0;
  
}
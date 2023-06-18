#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

char first = my_string.at(0);
int l =my_string.length();
char last = my_string.at(l-1);
cout << first;
cout << " is the first character and ";
cout << last ;
cout << " is the last character" << endl;


  //add code above this line
  
  return 0;
  
}
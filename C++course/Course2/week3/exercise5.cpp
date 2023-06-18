#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

for(int i=0;i< my_string.length();i++){
if(i%2==0){
cout << my_string.at(i+1);
}
else{
cout<< my_string.at(i-1);
}}

  //add code above this line
  
  return 0;
  
}
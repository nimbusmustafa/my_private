#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string original = (argv[1]);
  string modified;
  char ch;
  
  //add code below this line
for(int i=0; i< original.length();i++){
if( char(original.at(i)) >= 'a' && char(original.at(i)) <= 'z'){
modified.at(i)="l";
cout << modified;}
if( char(original.at(i)) >= 'A' && char(original.at(i)) <= 'Z'){
modified.push_back('u');
cout << modified;}
else{
modified.push_back('-');
cout<< modified;}}


  //add code above this line
  
  return 0;
  
}
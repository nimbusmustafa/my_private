#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line
 int l= my_string.length();
  int i;
  int k=l/2;
if(l %2==0){
  for( i=0;i<k;i++){
cout << my_string.at(i) ;
}
cout << endl;
for(i=k;i<l;i++){
  cout << my_string.at(i);
}
}
if(l%2!=0){
  for (i=0;i<k;i++){
    cout << my_string.at(i) ;
}
cout << endl;
for(i=k;i<l;i++){
  cout << my_string.at(i);
}
  }



  //add code above this line
  
  return 0;
  
}
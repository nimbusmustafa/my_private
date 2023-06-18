#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  int a = stoi(argv[1]);
  int b = stoi(argv[2]);
  
  if (a > b) {
    int c = b;
    b = a;
    a = c;
  }
  
  //add code below this line
int sum=0,i;
  for (i=a;i<=b;i++)
  {
    sum+=i;
  }
  cout << sum;


  //add code above this line
  
  return 0;
  
}

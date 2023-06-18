#include <iostream>
using namespace std;

//add function definitions below this line

int RecursiveSum(int num){
  if(num==1){
    return 1;
  }
  else if(num==0){
    return 0;
  }
  else{
  int x= num+ RecursiveSum(num-1);
  return x;
}}


//add function definitions above this line

int main(int argc, char** argv) {
  cout << RecursiveSum(stoi(argv[1])) << endl;
  return 0;
}

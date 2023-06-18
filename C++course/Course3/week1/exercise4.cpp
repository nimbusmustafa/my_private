#include <iostream>
#include <vector>
using namespace std;

//add code below this line
bool IsPalindrome(string x) {
  bool palindrome = false;
  string y;
  for (int i = x.length() - 1; i >= 0; i--) {
    y += x.at(i);
  }
  if (x == y) {
    palindrome = true;
  }
  return palindrome;
}


//add code above this line

int main(int argc, char** argv) {
  string x = argv[1];
  cout << boolalpha << IsPalindrome(x) << endl;
}
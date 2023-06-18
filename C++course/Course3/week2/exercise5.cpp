#include <iostream>
#include <vector>
#include<algorithm>
using namespace std;

//add function definitions below this line
int GetMax(vector<int>& nums) {
  if (nums.size() == 1) {
    return nums.at(0);
  }
  else {
    vector<int> new_vector(nums.begin() + 1, nums.begin() + nums.size());
    return max(nums.at(0), GetMax(new_vector));
  }
}



//add function definitions above this line

int main(int argc, char** argv) {
  vector<int> nums;
  for (int i = 1; i < argc; i++) {
    nums.push_back(stoi(argv[i]));
  }
  cout << GetMax(nums) << endl;
  return 0;
}

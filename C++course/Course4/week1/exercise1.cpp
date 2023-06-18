#include <iostream>
using namespace std;

//add class definitions below this line
class Fruit {
  public:
    Fruit(string n, string c) {
      name = n;
      color = c;
    }
  
    string GetName() {
      return name;
    }
  
    void SetName(string new_name) {
      name = new_name;
    }
  
    string GetColor() {
      return color;
    }
  
    void SetColor(string new_color) {
      color = new_color;
    }
  
  private:
    string name;
    string color;
};


//add class definitions above this line


int main() {
  
  //DO NOT EDIT CODE BELOW THIS LINE

  Fruit fruit("apple", "red");
  cout << fruit.GetName() << endl;
  cout << fruit.GetColor() << endl;
  fruit.SetName("orange");
  fruit.SetColor("orange");
  cout << fruit.GetName() << endl;
  cout << fruit.GetColor() << endl;

  //DO NOT EDIT CODE ABOVE THIS LINE
  
  return 0;
  
}
#include <iostream>
using namespace std;

//add class definitions below this line

class Watch {
  public:
    Watch(string mf, string mo, int d, int w, string mt) {
      manufacturer = mf;
      model = mo;
      diameter = d;
      water_resistance = w;
      material = mt;
    }
  
    string GetManufacturer() {
      return manufacturer;
    }

    void SetManufacturer(string new_manufacturer) {
      manufacturer = new_manufacturer;
    }

    string GetModel() {
      return model;
    }

    void SetModel(string new_model) {
      model = new_model;
    }

    int GetDiameter() {
      return diameter;
    }

    void SetDiameter(int new_diameter) {
      diameter = new_diameter;
    }

    int GetWaterResistance() {
      return water_resistance;
    }

    void SetWaterResistance(int new_water_resistance) {
      water_resistance = new_water_resistance;
    }

    string GetMaterial() {
      return material;
    }

    void SetMaterial(string new_material) {
      material = new_material;
    }

    void Summary() {
      cout << "Manufacturer: " << manufacturer << endl;
      cout << "Model: " << model << endl;
      cout << "Diameter: " << diameter << " mm" << endl;
      cout << "Water Resistance: " << water_resistance << " m" << endl;
      cout << "Material: " << material << endl;
    }
  
  private:
    string manufacturer;
    string model;
    int diameter;
    int water_resistance;
    string material;
};

//add class definitions above this line


int main() {
  
  //DO NOT EDIT CODE BELOW THIS LINE

  Watch my_watch("Omega", "Speedmaster", 42, 50, "steel");
  cout << my_watch.GetManufacturer() << endl;
  cout << my_watch.GetModel() << endl;
  cout << my_watch.GetDiameter() << endl;
  cout << my_watch.GetWaterResistance() << endl;
  cout << my_watch.GetMaterial() << endl;
  my_watch.SetManufacturer("Rolex");
  my_watch.SetModel("Explorer");
  my_watch.SetDiameter(36);
  my_watch.SetWaterResistance(60);
  my_watch.SetMaterial("gold");
  my_watch.Summary();

  //DO NOT EDIT CODE ABOVE THIS LINE
  
  return 0;
  
}
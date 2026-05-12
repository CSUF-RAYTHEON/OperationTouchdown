#include <iostream>
#include <stdexcept>

#include "akida/hw_version.h"
#include "host/hardware_drivers.h"

#include "TEST_NAME/test.h"

#define SUCCESS 0
#define FAILURE 1

int main() {
  bool result = false;
  // Get the list of drivers available on this host (each driver match one
  // device)
  const auto& drivers = akida::get_drivers();
  if (drivers.size() == 0) {
    std::cerr << "Unable to instantiate any driver" << std::endl;
    return FAILURE;
  }
  // Let's run the test for each driver on this host
  for (const auto& driver : drivers) {
    try {
      std::cout << "Running TEST_NAME with driver instance '" << driver->desc()
                << '\'' << std::endl;
      if (!TEST_NAME(driver.get())) {
        return FAILURE;
      }
    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
      return FAILURE;
    }
  }
  std::cout << "TEST_NAME successfully ran" << std::endl;
  return SUCCESS;
}

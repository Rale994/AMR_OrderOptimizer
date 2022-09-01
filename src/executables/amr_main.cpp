#include <iostream>

#include "amr.hpp"
#include "mosquitto.h"

int main(int argc, char *argv[]) {
  if (argc == 2) {
    mosquitto_lib_init();

    AMR::AmrUnit myAmrUnit{std::string(argv[1])};
    myAmrUnit.run();

    mosquitto_lib_cleanup();
    return 0;
  } else {
    std::cout << "Call: '" << argv[0]
              << " working_dir', where working_dir is a directory containing "
                 "configuration and orders subdirectories."
              << std::endl;
    return 1;
  }
}

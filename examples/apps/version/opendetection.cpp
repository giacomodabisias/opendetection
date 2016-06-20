//
// Created by sarkar on 03.06.15.
//

#include "od_version.h"
#include <iostream>

int main (int argc, char *argv[])
{
  std::cout << "OpenDetection: " << std::endl;
  std::cout << "You are using OpenDetection Version " << OD_MAJOR_VERSION << "." << OD_MINOR_VERSION << std::endl;
  return 0;
}

# """
# ===================================================================
# |                                                                 |
# |  COPYRIGHT NOTICE                                               |
# |                                                                 |
# |  This code is developed by OpenIoT - Open Platforms and         |
# |  Enabling Technologies for the Internet of Things.              |
# |                                                                 |
# |  Developer: Farhad Shamsfakhr, PhD                              |
# |  Affiliation: Fondazione Bruno Kessler (FBK)                    |
# |  Address: via Sommarive, 18 38123 Povo, Trento (Italy)          |
# |                                                                 |
# |  This project is sponsored by the Agri-Food project,            |
# |  coordinated by FONDAZIONE BRUNO KESSLER.                       |
# |  Funded under the Digital Europe Programme (DIGITAL).           |
# |                                                                 |
# |  © [2024] Farhad Shamsfakhr, PhD. All rights reserved.          |
# |                                                                 |
# ===================================================================
# """
cmake_minimum_required(VERSION 3.5.0)
project(uwb_positioning)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/main.py
  scripts/dwm1001_apiCommands.py
  scripts/dwm1001_systemDefinitions.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

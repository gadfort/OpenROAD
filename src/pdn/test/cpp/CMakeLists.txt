# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, The OpenROAD Authors

include("openroad")

set(TEST_LIBS
  GTest::gtest
  GTest::gtest_main
  GTest::gmock
  odb
  pdn
  tst
)

add_executable(TestPDNVia TestPDNVia.cpp stubs.cpp)
target_link_libraries(TestPDNVia ${TEST_LIBS})
# via.h is an internal header in src/pdn/src that isn't on pdn's public
# include path, so add it here for the unit test.
target_include_directories(TestPDNVia PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
gtest_discover_tests(TestPDNVia WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(TestPDNTechLayer TestPDNTechLayer.cpp stubs.cpp)
target_link_libraries(TestPDNTechLayer ${TEST_LIBS})
target_include_directories(TestPDNTechLayer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
gtest_discover_tests(TestPDNTechLayer WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(TestPDNShape TestPDNShape.cpp stubs.cpp)
target_link_libraries(TestPDNShape ${TEST_LIBS})
target_include_directories(TestPDNShape PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
gtest_discover_tests(TestPDNShape WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(TestPDNDomain TestPDNDomain.cpp stubs.cpp)
target_link_libraries(TestPDNDomain ${TEST_LIBS})
target_include_directories(TestPDNDomain PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
gtest_discover_tests(TestPDNDomain WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_dependencies(build_and_test
  TestPDNVia
  TestPDNTechLayer
  TestPDNShape
  TestPDNDomain
)

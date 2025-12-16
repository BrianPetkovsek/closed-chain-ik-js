cmake_minimum_required(VERSION 3.16)
project(ccik LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CCIK_DIST_DIR "${CMAKE_SOURCE_DIR}/../../dist" CACHE PATH "Directory for generated wasm artifacts")
file(MAKE_DIRECTORY ${CCIK_DIST_DIR})

add_library(ccik STATIC
	src/Chain.cpp
	src/IKSolver.cpp
)
target_include_directories(ccik PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (EMSCRIPTEN)
	add_executable(ccik_wasm bindings/embind.cpp)
	target_include_directories(ccik_wasm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
	target_link_libraries(ccik_wasm PRIVATE ccik)
	set_target_properties(ccik_wasm PROPERTIES
		OUTPUT_NAME "ccik"
		RUNTIME_OUTPUT_DIRECTORY "${CCIK_DIST_DIR}"
		ARCHIVE_OUTPUT_DIRECTORY "${CCIK_DIST_DIR}"
		LIBRARY_OUTPUT_DIRECTORY "${CCIK_DIST_DIR}"
	)
	target_link_options(ccik_wasm PRIVATE
		"-O3"
		"-s" "WASM=1"
		"-s" "MODULARIZE=1"
		"-s" "EXPORT_NAME=CCIKModule"
		"-s" "EXPORT_ES6=1"
		"-s" "ENVIRONMENT=web,worker,node"
		"--bind"
	)
endif()

include(CTest)
if (BUILD_TESTING)
	include(FetchContent)
	FetchContent_Declare(
		googletest
		URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
	)
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)

	add_executable(ccik_tests
		tests/chain_tests.cpp
		tests/solver_tests.cpp
	)
	target_include_directories(ccik_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
	target_link_libraries(ccik_tests PRIVATE ccik GTest::gtest_main)
	add_test(NAME ccik_tests COMMAND ccik_tests)
endif()

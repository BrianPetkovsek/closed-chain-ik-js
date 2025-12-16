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
		"-sWASM=1"
		"-sMODULARIZE=1"
		"-sEXPORT_NAME=CCIKModule"
		"-sEXPORT_ES6=1"
		"-sENVIRONMENT=web,worker,node"
		"--bind"
	)
endif()

include(CTest)
if (BUILD_TESTING)
	include(FetchContent)
	FetchContent_Declare(
		googletest
		URL https://github.com/google/googletest/archive/refs/tags/v1.15.0.zip
		URL_HASH SHA256=ed98258b96c6dc8af33eeb03c8d94a05b185866604382e12980f576595bdc509
		DOWNLOAD_EXTRACT_TIMESTAMP TRUE
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

cmake_minimum_required(VERSION 3.30)
project(graph-library-gummersbach)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -fexperimental-library")
set(CMAKE_OSX_DEPLOYMENT_TARGET "15.4")

find_package(Boost REQUIRED)


add_executable(graph-library-gummersbach
        main.cpp
        graph/AdjacentMatrixGraph.cpp
        graph/AdjacentListGraph.cpp
        graph/EdgeListGraph.cpp
        graph/SuperGraph.cpp
        helper/UnionFind.cpp
        helper/UnionFind.h
        edge/SuperEdge.h
        edge/WeightedEdge.h
        0x05.cpp
        helper/AugmentingPathFinder.h
        helper/BfsAugmentingPathFinder.cpp
        helper/BfsAugmentingPathFinder.h
)

target_include_directories(graph-library-gummersbach PRIVATE ${Boost_INCLUDE_DIRS})
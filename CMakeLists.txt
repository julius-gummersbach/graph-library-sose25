cmake_minimum_required(VERSION 3.30)
project(graph-library-gummersbach)

set(CMAKE_CXX_STANDARD 26)

add_executable(graph-library-gummersbach
        main.cpp
        graph/AdjacentMatrixGraph.cpp
        graph/AdjacentListGraph.cpp
        graph/EdgeListGraph.cpp
        graph/SuperGraph.cpp
)

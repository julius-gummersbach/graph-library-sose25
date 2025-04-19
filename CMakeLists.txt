cmake_minimum_required(VERSION 3.30)
project(graph-library-gummersbach)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -fexperimental-library")
set(CMAKE_OSX_DEPLOYMENT_TARGET "15.4")


add_executable(graph-library-gummersbach
        main.cpp
        graph/AdjacentMatrixGraph.cpp
        graph/AdjacentListGraph.cpp
        graph/EdgeListGraph.cpp
        graph/SuperGraph.cpp
)

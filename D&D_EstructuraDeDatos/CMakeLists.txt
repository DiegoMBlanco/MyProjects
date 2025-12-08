cmake_minimum_required(VERSION 3.29)
project(prueba_monstruos)

set(CMAKE_CXX_STANDARD 20)

add_executable(prueba_monstruos main.cpp
        linkedList.cpp
        linkedList.h
        Node.cpp
        Node.h
        Powers.cpp
        Powers.h
        Player.cpp
        Player.h
        HashTable.cpp
        HashTable.h
        Inventory.cpp
        Inventory.h
        Dice.cpp
        Dice.h
        Graph.cpp
        Graph.h
        Monster.cpp
        Monster.h
)

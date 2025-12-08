//
// Created by stanl on 16/11/2024.
//

#ifndef GRAPH_H
#define GRAPH_H
#include "linkedList.h"


class Graph {
private:
    int vertices;
    int** adjMatriz;
    int minDistance(int* distances, bool* visited);
public:
    Graph(int vertices): vertices(vertices) {
        adjMatriz = new int*[vertices];
        for(int i=0;i< vertices;i++) {
            adjMatriz[i]= new int[vertices];
            for(int j=0;j<vertices;j++) {
                adjMatriz[i][j]=0;
            }
        }
    }
    ~Graph() {
        for(int i=0;i<vertices;i++) {
            delete[] adjMatriz[i];
        }
        delete[] adjMatriz;
    }
    void addEdge(int src,int dest, int weight);
    void displayMatrix();
    void BFS(int startIndex);
    void DFS(int startIndex);
    void dijkstra(int startIndex);
    void vecinos(int& posicion);

};



#endif //GRAPH_H
//
// Created by stanl on 16/11/2024.
//
#include <iostream>
#include "Graph.h"
#include<vector>
#include<queue>
#include<stack>
#include <limits>

using namespace std;
void Graph::addEdge(int src,int dest, int weight) {
    adjMatriz[src][dest]=1;
}

void Graph::displayMatrix() {
    std::cout<<"Matrix: "<<endl;
    for(int i=0;i<vertices;i++) {
        for(int j=0;j<vertices;j++) {
            std::cout<<adjMatriz[i][j]<<" ";
        }
        std::cout<<std::endl;
    }
}


void Graph::BFS(int startVertex) {
    vector<bool> visited(vertices, false); // Vector para marcar los nodos visitados
    queue<int> q; // Cola para manejar los nodos a visitar en BFS

    visited[startVertex] = true; // Marcamos el nodo inicial como visitado
    q.push(startVertex); // Encolamos el nodo inicial

    std::cout << "BFS starting from vertex " << startVertex << ": ";
    while (!q.empty()) {
        int vertex = q.front(); // Tomamos el nodo actual
        std::cout << vertex << " "; // Procesamos el nodo
        q.pop(); // Lo eliminamos de la cola

        // Recorremos todos los vecinos del nodo actual
        for (int i = 0; i < vertices; i++) {
            if (adjMatriz[vertex][i] == 1 && !visited[i]) {
                visited[i] = true; // Marcamos el vecino como visitado
                q.push(i); // Encolamos el vecino
            }
        }
    }
    std::cout << std::endl;
}

void Graph::DFS(int startVertex) {
    vector<bool> visited(vertices, false); // Vector para marcar los nodos visitados
    stack<int> s; // Pila para manejar los nodos a visitar en DFS

    s.push(startVertex); // Apilamos el nodo inicial

    std::cout << "DFS starting from vertex " << startVertex << ": ";
    while (!s.empty()) {
        int vertex = s.top(); // Tomamos el nodo actual
        s.pop(); // Lo eliminamos de la pila

        // Si el nodo no ha sido visitado, lo procesamos
        if (!visited[vertex]) {
            std::cout << vertex << " "; // Procesamos el nodo
            visited[vertex] = true; // Marcamos el nodo como visitado
        }

        // Recorremos todos los vecinos del nodo actual
        for (int i = 0; i < vertices; i++) {
            if (adjMatriz[vertex][i] == 1 && !visited[i]) {
                s.push(i); // Apilamos el vecino si no ha sido visitado
            }
        }
    }
    std::cout << std::endl;
}

void Graph::dijkstra(int startIndex) {
    int* distances= new int[vertices];
    bool* visited= new bool[vertices];

    for(int i=0; i<vertices;i++) {
        distances[i]= INT_MAX;
        visited[i]=false;
    }

    distances[startIndex]=0;
    visited[startIndex]=true;
    for(int i=0;i<vertices;i++) {
        if(i!=startIndex) {
            if(adjMatriz[startIndex][i]) {
                distances[i]=adjMatriz[startIndex][i];
            }
        }
    }
    for (int i=0;i<vertices-1;i++) {
        int index=minDistance(distances,visited);
        visited[index]= true;
        for(int j=0;j<vertices;j++) {
            if(!visited[j]
                && adjMatriz[index][j]
                && distances[index] != INT_MAX
                && distances[index] + adjMatriz[j][index] < distances[j]) {
                distances[j]= distances[index]+  adjMatriz[index][j];
            }
        }
    }

    cout<<"Dijsktra from "<<startIndex<<endl;
    for(int i=0; i<vertices;i++) {
        cout<<i<<"\t "<<distances[i]<<endl;
    }
    cout<<"Visited from "<<startIndex<<endl;
    for(int i=0; i<vertices;i++) {
        cout<<i<<"\t "<<visited[i]<<endl;
    }
    delete[] distances;
    delete[] visited;
}

int Graph::minDistance(int* distances, bool* visited) {
    int min= INT_MAX, minIndex;
    for(int i=0; i<vertices;i++) {
        if(!visited[i] && distances[i] <= min) {
            min= distances[i];
            minIndex= i;
        }
    }
    return minIndex;
}

void Graph::vecinos(int& posicion) {

    int contador = 0;

    cout<<"Estas en el cuarto [" <<  posicion << "]. Que camino eliges?"<< endl;
    for(int i=0;i<vertices;i++){
        if(adjMatriz[posicion][i]!=0) {
            contador++;
        }
    }

    int misVecinos[contador];
    int index = 0;
    for(int i=0;i<vertices;i++){
        if(adjMatriz[posicion][i]!=0) {
            misVecinos[index] = i;
            index++;
            cout<<"Puedes visitar: "<<i <<endl<<endl;;
        }
    }

    int camino;

    bool flag = false;
    while(flag== false) {
        cout<<"Camino a elegir: ";

        if (cin >> camino) {
            cout<<"Confirma: ";
            cin>>camino;
            for(int i = 0; i<contador; i++) {
                if(misVecinos[i] == camino) {
                    flag = true;
                    break;
                }
            }
        }else {
            cin.clear(); // Limpia el error de tipo
            cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Ignora lo que queda en el buffer

        }


    }
    posicion = camino;
}
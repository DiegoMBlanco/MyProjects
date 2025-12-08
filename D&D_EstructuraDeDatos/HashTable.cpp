//
// Created by stanl on 12/11/2024.
//

#include "Hashtable.h"
#include "Powers.h"
#include <iostream>
using namespace std;

#include "Hashtable.h"

template <typename T>
int Hashtable<T>::hash(int key) {
    return key%14;
}

template <typename T>
void Hashtable<T>::insert(T data, int num) {
    int index= hash(num);
    Node<T>* newNode= new Node<T>(data);
    //implementar colisión
    newNode->setNext(table[index]);
    table[index]= newNode;
}

template <typename T>
Node<T>* Hashtable<T>::search(T data, int num) {
    int index= hash(num);
    //implementar colisión
    return table[index];
}

template <typename T>
void Hashtable<T>::remove(T data, int num) {
    int index= hash(num);
    //implementar colision
    table[index]= nullptr;
}

template <typename T>
void Hashtable<T>::showTable() {
    cout<<"--------Table---------"<<endl;
    for (int i=0; i< capacity; i++) {
        if(table[i]!=nullptr){
            Node<T>* temp= table[i];
            while(temp!=nullptr) {
                cout<<"["<<i+1<<"]: "<<temp->getData()<< " ";
                temp= temp->getNext();
            }
            cout<<endl;
        }else {
            cout<<" NULL "<<endl;
        }
    }
    cout<<"--------END---------"<<endl;
}

template <typename T>
void Hashtable<T>::showPower(int i) {
    int index = i;
    Node<T>* temp = table[index];
    cout<<temp->getData();
}


template class Hashtable<Powers>;
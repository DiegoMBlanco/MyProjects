//
// Created by stanl on 10/11/2024.
//

#include "Node.h"
#include <iostream>
#include "Monster.h"
#include "Powers.h"
#include "Inventory.h"


template<typename T>
Node<T>::Node (T data) {
    this->data = data;
    next = nullptr;
}
template<typename T>
T Node<T>::getData() {
    return this->data;
}
template<typename T>
void Node<T>::setData(T data) {
    this->data=data;
}
template<typename T>
Node<T>* Node<T>::getNext() {
    return this->next;
}
template<typename T>
void Node<T>::setNext(Node<T> *next) {
    this->next=next;
}

template class Node<Monster>;
template class Node<Powers>;
template class Node<Inventory>;
template class Node<string>;
template class Node<int>;
//
// Created by stanl on 10/11/2024.
//

#ifndef LINKEDLIST_H
#define LINKEDLIST_H
#include "Node.h"
#include "Monster.h"
#include "inventory.h"


template <typename T>
class LinkedList {
private:
    Node<T>* head;
public:
    LinkedList();
    void insertAtStart(T value);
    void insertAtFinish(T value);
    void displayList();
    int findElement(T value);
    void deleteElement(T value);
    void read_record();
    void displayDefeatedMonsters();
    Monster getRandomMonster();
    Inventory getRandomItem();
    ~LinkedList() {
        Node<T>* current=head;
        while(current!=nullptr) {
            Node<T>* temp= current;
            current=current->getNext();
            delete temp;
        }
    }
};

#endif
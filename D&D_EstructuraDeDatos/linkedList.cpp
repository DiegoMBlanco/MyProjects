//
// Created by stanl on 10/11/2024.
//
#include "linkedList.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "Inventory.h"

using namespace std;
template<typename T>
LinkedList<T>::LinkedList() {
    head=nullptr;
}

template<typename T>
void LinkedList<T>::insertAtStart(T value) {
    Node<T>* n= new Node(value);
    if(head==nullptr) {
        head= n;
    }else {
        n->setNext(head);
        head=n;
    }
}

template<typename T>
void LinkedList<T>::insertAtFinish(T value) {
    Node<T>* n= new Node(value);
    if(head==nullptr) {
        head=n;
    }else {
        Node<T>* current= head;
        while(current->getNext()!= nullptr) {
            current= current->getNext();
        }
        current->setNext(n);
    }
}



template<typename T>
int LinkedList<T>::findElement(T value) {
    Node<T>* current=head;
    int index=0;
    while(current!= nullptr) {
        if(current->getData()==value) {
            return index;
        }
        current= current->getNext();
        index+=1;
    }
    return -1;
}
template<typename T>
void LinkedList<T>::deleteElement(T value) {
    if (head==nullptr) {
        return;
    }
    if(head->getData()==value) {
        Node<T>* temp= head;
        head= head->getNext();
        delete temp;
        return;
    }
    Node<T>* current= head;
    while(current->getNext()!= nullptr && current->getNext()->getData()!=value) {
        current=current->getNext();
    }
    if(current->getNext()!=nullptr) {
        Node<T>* temp= current->getNext();
        current->setNext(current->getNext()->getNext());
        delete temp;
    }
}

template<>
void LinkedList<Monster>::read_record()
{
    // File pointer
    fstream fin;

    // Open an existing file
    fin.open("monsters_patched.csv", ios::in);
    if(fin.fail()){
        cout<<"No se pudo abrir el archivo.";
        exit(1);
    }

    // Read the Data from the file
    // as String Vector
    string line, name, type,size, align;
    int ac, lp, cr;
    vector<string> row;
    getline(fin, line);

    while (getline(fin, line))
    {

        row.clear();

        stringstream s(line); // To process the line
        string word;

        // Split line by comma
        while (getline(s, word, ',')) {
            row.push_back(word);
        }

        // Check if row has the correct number of columns
        if (row.size() < 7) continue;

        // Parse the columns into appropriate data types
        name = row[0];
        cr = stoi(row[1]);
        type = row[2];
        size = row[3];
        ac = stoi(row[4]);
        lp = stoi(row[5]);
        align = row[6];

        Monster monster(name,cr,type,align,size,ac,lp);
        insertAtFinish(monster);

    }
}


template<>
Monster LinkedList<Monster>::getRandomMonster() {
    srand(time(NULL));
    int random=rand()%(760);
    Node<Monster>* current=head;
    int index = 0;
    while(current!= nullptr) {
        if(random == index) {
            Monster value = current->getData();
            return value;
        }else {
            current= current->getNext();
            index+=1;
        }
    }
}

template<>
Inventory LinkedList<Inventory>::getRandomItem() {
    srand(time(NULL));
    int random=rand()%(4);
    Node<Inventory>* current=head;
    int index = 0;
    while(current!= nullptr) {
        if(random == index) {
            Inventory value = current->getData();
            return value;
        }else {
            current= current->getNext();
            index+=1;
        }
    }
}

template<typename T>
void LinkedList<T>::displayList() {
    Node<T>* current=head;
    cout<<"[";
    while(current!= nullptr) {
        cout<<current->getData()<<" \n";
        current= current->getNext();
    }
    cout<<"]"<<endl;
}



template class LinkedList<Monster>;
template class LinkedList<Inventory>;
template class LinkedList<string>;
template class LinkedList<int>;
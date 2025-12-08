//
// Created by stanl on 12/11/2024.
//
#include <iostream>
#include <string>
using namespace std;
#ifndef PLAYER_H
#define PLAYER_H



class Player {
public:
    Player() :  lp(100), race(""),name(""){};
    Player(int lp, string race, string name) :  lp(lp), race(race),name(name){};
    ~Player(){}
    //seters y geters
    void setLp(int lp);
    int getLp();
    void setRace(string race);
    string getRace();
    void setName(string name);
    string getName();
    void display();


private:
    int  lp;
    string race, name; //lista de monstruos faltante
};



#endif //PLAYER_H
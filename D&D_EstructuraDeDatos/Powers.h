//
// Created by stanl on 12/11/2024.
//
#include <iostream>
#include <string>
#include "Monster.h"
#include "Player.h"
#include "Dice.h"
using namespace std;

#ifndef POWERS_H
#define POWERS_H

class Powers {
public:
    Powers(): name(""), description(""){};
    Powers(string name, string description) : name(name), description(description){};
    string getName();
    string getDescription();
    void setName(string name);
    void setDescription(string name);
    void accion(int n, Monster& m, Player& p, int& turnoint, int& conteoN10, int daño, int vida, int dañoM,bool& invisibilidad, int& dañoDobleHechizo, int dañoDado);
    void aplicarTurnoPlayer(Monster& m, Player& p, int daño, bool& invisibilidad);
    void aplicarTurnoMonster(Monster& m, Player& p, int dañoM, bool& invisibilidad, int& conteoN10);

    ~Powers(){};
    friend std::ostream& operator<<(std::ostream& os,const Powers& P) {
        os<<"Name: "<<P.name<<", "<<" Description: "<<P.description<<" ";
        return os;
    }
    bool operator==(const Powers& other) const {
        return (name==other.name && description==other.description);
    }
    bool operator!=(const Powers& other) const {
        return (name!=other.name || description!=other.description);
    }
private:
    string name, description;
    Dice dado;
};
#endif //POWERS_H

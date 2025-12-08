//
// Created by stanl on 16/11/2024.
//
#include "Dice.h"
#include <iostream>
#include <string>

using namespace std;

void Dice::setFace1(int face1) {
    this->face1=face1;
}

int Dice::getFace1() {
    return this->face1;
}

void Dice::setFace2(int face2) {
    this->face2=face2;
}

int Dice::getFace2() {
    return this->face2;
}
float Dice::getRandomDice1() {

    float random1=rand()%(8-1+1)+1;
    return random1;
}

float  Dice::getRandomDice2() {

    float random2=rand()%(10-1+1)+1;
    return random2;
}
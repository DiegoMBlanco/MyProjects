//
// Created by stanl on 16/11/2024.
//
#ifndef DICE_H
#define DICE_H

#include <string>
#include <cstdlib>
#include <ctime>
using namespace std;


class Dice {
public:
    Dice (): face1(8), face2(10){srand(time(NULL));};
    Dice(int face1, int face2):face1(face1), face2(face2){srand(time(NULL));};

    ~Dice(){}
    void setFace1(int face1);
    int getFace1();
    void setFace2(int face2);
    int getFace2();
    float getRandomDice1();
    float getRandomDice2();
private:
    int face1, face2;
};



#endif //DICE_H
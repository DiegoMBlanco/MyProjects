//
// Created by stanl on 12/11/2024.
//

#include "Player.h"

void Player:: setLp(int lp){
    this->lp=lp;
}

int Player::getLp(){
    return this->lp;
}

void Player::setRace(string race){
    this->race=race;
}

string Player::getRace(){
    return this->race;
}

void Player::setName(string name){
    this->name=name;
}

string Player::getName(){
    return this->name;
}

void Player::display() {
    cout<<"YOU -> "<<"Name: "<<name<<", "<<"Race: "<<race<<", "<<"Life Points: "<<lp<<endl;
}
//
// Created by stanl on 12/11/2024.
//

#include "Inventory.h"

void Inventory::setName(string name) {
    this->name = name;
}
string Inventory::getName() {
    return this->name;
}
void Inventory::setDescription(string description) {
    this->description = description;
}
string Inventory::getDescription() {
    return this->description;
}
void Inventory::setAmount(int amount) {
    this->amount = amount;
}
int Inventory::getAmount() {
    return this->amount;
}
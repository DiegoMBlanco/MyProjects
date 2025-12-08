
#ifndef INVENTORY_H
#define INVENTORY_H

#include <string>
#include <iostream>
using namespace std;

class Inventory {
public:
    Inventory(): name(""), description(""), amount(0){};
    Inventory(string name, string description, int amount): name(name), description(description), amount(amount){};

    ~Inventory(){}
    void setName(string name);
    string getName();
    void setDescription(string description);
    string getDescription();
    void setAmount(int amount);
    int getAmount();
    friend std::ostream& operator<<(std::ostream& os,const Inventory& I) {
        os<<"Name: "<<I.name<<", "<<" description: "<<I.description<<", "<<"Amount: "<<I.amount<<" ";
        return os;
    }
    bool operator==(const Inventory& other) const {
        return (name==other.name && description==other.description && amount == other.amount);
    }
    bool operator!=(const Inventory& other) const {
        return (name!=other.name || description!=other.description || amount != other.amount);
    }
private:
    string name, description;
    int amount;
};

#endif //INVENTORY_H

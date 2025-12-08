#include <iostream>
#include "Monster.h"
#include "LinkedList.h"
#include "Powers.h"
#include "HashTable.h"
#include "Inventory.h"
#include "Dice.h"
#include "Graph.h"
#include <cstdlib>



void createMonster();
void createPlayer(Player& you);
void createInventory();
void createGraph(Graph& calabozo);
void createPowers(Hashtable<Powers>& HashP, Powers& p1);
void assignPowers( Hashtable<Powers>& HashP, int arrPowers[], int randomDicePowers[]);
int combate(Monster& monster, Player& you, Powers& p1, int arrPowers[], int& conteoN10, int& turno, Dice dado, int randomDicePowers[],Graph& calabozoRoom, int& posicion, bool& danioPredeterminado,int& dadoDanioPredeterminado);
int juego(Monster& monster, Player& you, Powers& p1, int arrPowers[],LinkedList<Monster> monster_list, int& conteoN10, Dice dado, int randomDicePowers[], LinkedList<string>& defeated, LinkedList<Inventory>& backpack,Graph& calabozoRoom, int& posicion,LinkedList<Inventory>& inventario, Inventory& item, bool& danioPredeterminado,int& dadoDanioPredeterminado);
void crearArregloDadosPowers(int randomDicePowers[]);
bool checarDado(int randomDicePowers[], int randomN);
void calabozo(Graph& calabozoRoom, int& posicion, int turno);
int conteoN10 = 0;
int daño = 0;
int vida = 0;
int dañoM = 0;
int dañoDobleHechizo = 0;
int dañoDado = 0;
int turno = 0;
bool invisibilidad = false;
int arrPowers[5];
int randomDicePowers[5] = {0};
int posicion = 1;
bool danioPredeterminado;
int dadoDanioPredeterminado;


int main() {
    //creación del calabozo
    Graph calabozoRoom(21);
    createGraph(calabozoRoom);



    //Creación del mosntruo
    LinkedList<Monster> monster_list;
    monster_list.read_record();
    Monster monster;
    monster = monster_list.getRandomMonster();

    //Creación del jugador
    Player you;
    createPlayer(you);
    you.display();

    //creación del dado
    Dice dado;

    //creación inventario
    cout<<"--------------------------------- Lista inventario -------------------------------------"<<endl;
    LinkedList<Inventory> inventario;// de aqui se van a sacar los objetos random tras cada victoria
    LinkedList<Inventory> backpack; // Aquí se van a guardarv los objetos que suelten los monstruos
    Inventory item1("Llave","Esto abre algo?",1);
    Inventory item2("Mapa","El One Piece esta muy cerca ....",1);
    Inventory item3("Nullptr","Esto no es cero",1);
    Inventory item4("Posión del dinero maldito","Estas en bancarrota .-.",1);
    Inventory item5("Carta","Tus familiares quieren que llegues a salvo <3 ",1);
    inventario.insertAtFinish(item1);
    inventario.insertAtFinish(item2);
    inventario.insertAtFinish(item3);
    inventario.insertAtFinish(item4);
    inventario.insertAtFinish(item5);
    Inventory item = inventario.getRandomItem();
    //Creación poderes
    Hashtable<Powers> HashP;
    Powers p1;
    createPowers(HashP, p1);
    HashP.showTable();
    assignPowers(HashP, arrPowers, randomDicePowers);

    //Mostrar monstruos derrotados mediante una lista ligada
    LinkedList<string> defeated; //Está bien, creas la lista llamada "defeated"






    calabozo(calabozoRoom,posicion,turno);
    juego(monster,you,p1,arrPowers,monster_list, conteoN10,dado,randomDicePowers,defeated, backpack,calabozoRoom,posicion,inventario,item,danioPredeterminado,dadoDanioPredeterminado);

}

void createPlayer(Player& you) {
    string usuario;
    string raza;
    bool razaValida = false;
    cout << "Ingrese su usuario: ";
    getline(cin, usuario);
    you.setName(usuario);
    you.setLp(100);
    while (!razaValida) {
        cout << "Ingresa tu raza (Elf, Human, Dwarf): ";
        getline(cin, raza);
        if (raza == "Elf" || raza == "Human" || raza == "Dwarf") {
            razaValida = true;
            you.setRace(raza); // Mover aquí
        } else {
            cout << "Eres de una raza alienigena?" << endl;
            cout << "Por favor ingresa una raza valida." << endl;
        }
        fflush(stdin);
    }
}

void createGraph(Graph& calabozo) {
    calabozo.addEdge(1, 2, 0);
    calabozo.addEdge(1, 4, 0);
    calabozo.addEdge(1, 3, 0);
    calabozo.addEdge(2, 6, 0);
    calabozo.addEdge(2, 4, 0);
    calabozo.addEdge(3, 4, 0);
    calabozo.addEdge(3, 7, 0);
    calabozo.addEdge(4, 8, 0);
    calabozo.addEdge(4, 9, 0);
    calabozo.addEdge(4, 7, 0);
    calabozo.addEdge(6, 10, 0);
    calabozo.addEdge(6, 8, 0);
    calabozo.addEdge(7, 9, 0);
    calabozo.addEdge(7, 11, 0);
    calabozo.addEdge(7, 14, 0);
    calabozo.addEdge(8, 10, 0);
    calabozo.addEdge(8, 12, 0);
    calabozo.addEdge(8, 15, 0);
    calabozo.addEdge(8, 13, 0);
    calabozo.addEdge(9, 13, 0);
    calabozo.addEdge(9, 16, 0);
    calabozo.addEdge(9, 14, 0);
    calabozo.addEdge(10, 12, 0);
    calabozo.addEdge(11, 14, 0);
    calabozo.addEdge(12, 15, 0);
    calabozo.addEdge(13, 15, 0);
    calabozo.addEdge(13, 17, 0);
    calabozo.addEdge(13, 16, 0);
    calabozo.addEdge(14, 16, 0);
    calabozo.addEdge(15, 17, 0);
    calabozo.addEdge(16, 17, 0);
    calabozo.addEdge(16, 19, 0);
    calabozo.addEdge(17, 18, 0);
    calabozo.addEdge(17, 19, 0);
    calabozo.addEdge(18, 20, 0);
    calabozo.addEdge(19, 20, 0);
}



void createPowers(Hashtable<Powers>& HashP, Powers& p1) {
    p1.setName("Eldritch Blast");
    p1.setDescription("Este hechizo conjura una rafaga de energia magica oscura que se lanza contra un enemigo. HP = 1.5 * d(10) ");
    HashP.insert(p1,1);
    Powers p2("Invisibility", "Eres invisible durante el proximo turno, por lo que no te pueden hacer dano");
    HashP.insert(p2,2);
    Powers p3("Fireball", "Disparas una bola de fuego de tu pecho que inflige daño instantaneo. HP = 2 * d(10)");
    HashP.insert(p3,3);
    Powers p4("Healing Word", "Recuperas puntos de vida LP = 20 + d(10)");
    HashP.insert(p4,4);
    Powers p5("Mystic Shot", "Este hechizo permite multiplicar x10 el dano asignado a tu dado (solo aplicable en el primer turno de cada combate). HP = 10*d(10)");
    HashP.insert(p5,5);
    Powers p6("Aura of Vitality", "Recibes puntos de vida. LP = 2 * d(10) ");
    HashP.insert(p6,6);
    Powers p7("Slicker Shot", "El dano que hagas en esta tirada d(10) se suma al valor de dano que el monstruo recibio en el turno pasado");
    HashP.insert(p7,7);
    Powers p8("Celestial Will", "Los dioses han decidido ayudarte con un ataque letal que inflige dano verdadero. HP = 300 ");
    HashP.insert(p8,8);
    Powers p9("Personal Jesus", "La lagrima de un gnomo multiversal ha caido sobre tu hermosa cabellera, otorgandote una cantidad considerable de vida. LP = 500 ");
    HashP.insert(p9,9);
    Powers p10("Weakness", "Provoca que el dano de los siguientes 2 ataques del monstruo se reduzcan a la mitad. ");
    HashP.insert(p10,10);
    Powers p11("Acid Splash", "Este hechizo inflige dano instananeo. HP = 50 + d(10) ");
    HashP.insert(p11,11);
    Powers p12("Finger of Death", "Obtienes acceso a la lista de mosntruos, y usas el método deleteNode() para borrar a tu oponente de esta realidad. (instakill)");
    HashP.insert(p12,12);
    Powers p13("Vampiric Touch", "Controlas el alma de la bestia, robando puntos de vida a tu favor. LP = d(10) , HP = d(10)");
    HashP.insert(p13,13);
    Powers p14("Compound seizure", "Infliges dano proporcional al numero de turno actual por combate. HP = d(10) * #turno");
    HashP.insert(p14,14);

}

void assignPowers(Hashtable<Powers>& HashP, int arrPowers[], int randomDicePowers[]) {
    int nPower;
    for (int i= 0; i<5; i++) {
        bool validInput = false;
        while (!validInput) {
            cout<<"Ingrese el numero del poder que quiere agregar a su repertorio["<<i+1<<"]: ";
            cin>>nPower;
            if(nPower >= 1 && nPower <= 14) {
                arrPowers[i]=nPower;
                validInput=true;
            } else {
                cout <<"Aun no eres capaz de usar ese hechizo"<<endl;
                cout<<"Ingresa un hechizo que ya conoces"<<endl;
            }
        }
    }
    cout<<"\n\n\n\nTus hechizos: "<<endl;
    for(int i = 0; i<5; i++) {
        cout<<"["<<arrPowers[i]<<"] ";
        HashP.showPower(i);
        cout<<endl;
    }

    cout<<"---------------arreglo dados--------------------"<<endl;
    crearArregloDadosPowers(randomDicePowers);

    cout<<"\n Sus hechizos: ";
    for(int i = 0; i<5; i++) {
        cout<<arrPowers[i]<<", ";
    }
    cout<<"Tienen asignados los siguientes numeros del dado d(10): ";
    for(int i = 0; i<5; i++) {
        cout<<randomDicePowers[i]<<", ";
    }
    cout<<endl;
}

void crearArregloDadosPowers(int randomDicePowers[]) {
    int randomN;
    Dice dado;
    for (int i = 0; i < 5; i++) {
        do {
            randomN = dado.getRandomDice2(); // dado de 10
        } while (checarDado(randomDicePowers,randomN)); // Verifica si el número ya existe

        randomDicePowers[i] = randomN; // Si no se repite, lo asigna
    }
}

bool checarDado(int randomDicePowers[], int randomN) {
    for (int i = 0; i < 5; i++) {
        if (randomDicePowers[i] == randomN) {
            return true; // Número ya existe
        }
    }
    return false; // Número no está en el arreglo
}
int combate(Monster& monster, Player& you, Powers& p1, int arrPowers[], int& conteoN10, int& turno, Dice dado, int randomDicePowers[], Graph& calabozoRoom, int& posicion, bool& danioPredeterminado, int& dadoDanioPredeterminado) {
    fflush(stdin);
    cout << "---------------------------- Combate ------------------------" << endl << endl;

    string opcion;


    if (monster.getLp() <= 0) {
        cout << "¡El monstruo ya ha sido derrotado!" << endl;
        return 45; // monstruo muerto
    }

    while (monster.getLp() > 0) {
        cout << "-------------------- Inicio de turno --------------------------------" << endl;
        turno++;
        bool teclaValida = false;
        while (!teclaValida) {
            cout << "Deseas lanzar el dado (Y): " << endl;
            cin >> opcion;
            if (opcion == "y" || opcion == "Y") {
                teclaValida = true;
            } else {
                cout << "Que tipo de hechiceria es esta. Por favor conjura algo correcto" << endl;
            }
        }

        int index = -1;
        int valorDado = dado.getRandomDice2();
        cout << "Buena suerte guerrero" << endl << endl;

        cout << "Dado = " << valorDado << endl << endl;

        for (int i = 0; i < 5; i++) {
            if (randomDicePowers[i] == valorDado) {
                index = i;
                break;
            }
        }

        if (index != -1) { //encontró el valor en el arreglo de poderes. Esa cara tiene un poder asignado
            cout << "La suerte te beneficia guerrero" << endl;
            p1.accion(arrPowers[index], monster, you, turno, conteoN10, daño, vida, dañoM, invisibilidad, dañoDobleHechizo, dañoDado);
        } else {
            // Si no encontramos el valor, aplicar daño predeterminado
            dadoDanioPredeterminado = valorDado; // Guardar el valor del dado
            int dañoRecibido = dado.getRandomDice1();
            monster.setLp(max(0, monster.getLp() - dadoDanioPredeterminado));
            you.setLp(you.getLp() - dañoRecibido);

            you.display();
            monster.display();
            cout << "Hiciste " << dadoDanioPredeterminado << " de daño" << endl;
            cout << "Recibiste " << dañoRecibido << " de daño" << endl;
        }

        if (you.getLp() <= 0) {
            cout << "¡Has caído en batalla, valiente guerrero!" << endl;
            return 76; // jugador muerto
        }

        if (monster.getLp() <= 0) {
            cout << "¡Has derrotado al monstruo!" << endl;
            return 45;
        }
    }
}



int juego(Monster& monster, Player& you, Powers& p1, int arrPowers[], LinkedList<Monster> monster_list, int& conteoN10, Dice dado, int randomDicePowers[], LinkedList<string>& defeated, LinkedList<Inventory>& backpack, Graph& calabozoRoom, int& posicion, LinkedList<Inventory>& inventario, Inventory& item, bool& danioPredeterminado, int& dadoDanioPredeterminado) {
    fflush(stdin);
    bool juegoActivo = true;

    while (juegoActivo) {
        monster.display();
        int condicion = combate(monster, you, p1, arrPowers, conteoN10, turno, dado, randomDicePowers, calabozoRoom, posicion, danioPredeterminado, dadoDanioPredeterminado);

        if (condicion == 45) {//el monstruo es derrotado
            turno = 0;
            if (posicion == 20) {
                cout << "\n\n---------------------------------Ganaste---------------------";
                cout << "\n-------------------- Resumen del juego --------------------" << endl;
                cout << "Monstruos derrotados: " << endl;
                defeated.displayList();

                cout << "\nObjetos recolectados: " << endl;
                backpack.displayList();

                cout << "\nEstadísticas del jugador: " << endl;
                you.display();
                cout << "----------------------------------------------------------" << endl;
                cout<<"Hecho por:\nGuillermo\nCarolina\nDiego\n";
                return 0;
            } else {

                defeated.insertAtFinish(monster.getName());

                monster = monster_list.getRandomMonster();

                if (monster.getLp() <= 0) {
                    monster.setLp(10);
                }

                backpack.insertAtFinish(item);
                item = inventario.getRandomItem();
                cout<<"---------------Tu mochila: ---------------------";
                backpack.displayList();
                cout<<"\n\n";
                calabozo(calabozoRoom, posicion, turno);

            }

        } else if (condicion == 76) {//el jugador muere
            cout << "Game over" << endl;
            cout << "\n-------------------- Resumen del juego --------------------" << endl;
            cout << "Monstruos derrotados: " << endl;
            defeated.displayList(); // Mostrar monstruos derrotados

            cout << "\nObjetos recolectados: " << endl;
            backpack.displayList(); // Mostrar objetos recolectados

            cout << "\nEstadísticas del jugador: " << endl;
            you.display(); // Mostrar información del jugador final

            cout << "----------------------------------------------------------" << endl;
            cout<<"Hecho por:\nGuillermo\nCarolina\nDiego\n";
            return 0;
        } 
    }
}


void calabozo(Graph& calabozo, int& posicion, int turno) {
    fflush(stdin);
    cout<<"Te puedes mover a los siguiente cuartos: ";
    calabozo.vecinos(posicion);

}






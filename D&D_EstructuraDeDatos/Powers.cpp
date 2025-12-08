//
// Created by stanl on 12/11/2024.
//

#include "Powers.h"
#include "Dice.h"


void Powers::setName(string name){
    this->name=name;
}

string Powers::getName(){
    return name;
}

void Powers::setDescription(string description){
    this->description=description;
}

string Powers::getDescription(){
    return name;
}
void Powers::aplicarTurnoPlayer(Monster& m, Player& p, int daño, bool& invisibilidad) {
    if (daño > 0) {
        m.setLp(m.getLp() - daño);
        cout << "Daño infligido al monstruo: " << daño << endl;
    }
}

void Powers::aplicarTurnoMonster(Monster& m, Player& p, int dañoM, bool& invisibilidad, int& conteoN10) {
    if (invisibilidad) {
        cout << "Monstruo: Grsss, ¿en dónde estás?" << endl;
        invisibilidad = false;
    } else if (m.getLp() > 0) {
        int dañoRecibido = (conteoN10 > 0) ? dañoM / 2 : dañoM;
        p.setLp(p.getLp() - dañoRecibido);
        if (conteoN10 > 0) conteoN10--;
        cout << "Daño recibido del monstruo: " << dañoRecibido << endl;
    }
}

void Powers::accion(int n, Monster& m, Player& p, int& turno, int& conteoN10, int daño, int vida, int dañoM, bool& invisibilidad, int& dañoDobleHechizo, int dañoDado) {
    cout << "\nTurno: " << turno << endl;

    switch (n) {
        case 1: // Conjura una ráfaga de energía mágica oscura
            daño = 1.5*dado.getRandomDice2();
            dañoDobleHechizo = daño;
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);
            break;

        case 2: // Invisibilidad durante el próximo turno
            cout << "Si no me muevo, no me ven jijijija" << endl;
            invisibilidad = true;
            break;

        case 3: // Bola de fuego
            daño = 2 * dado.getRandomDice2();
            dañoDobleHechizo = daño;
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);
            break;

        case 4: // Recupera vida
            vida = 20 + dado.getRandomDice2();
            p.setLp(p.getLp() + vida);
            cout << "Vida recuperada: " << vida << endl;
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);
            break;

        case 5: // Hechizo especial
            cout << "Hechizo especial activado." << endl;
            daño = dado.getRandomDice2();
            dañoDobleHechizo = daño;
            if(turno == 1) {
                m.setLp(m.getLp()-(10*daño));
                dañoDobleHechizo = 10*daño;
            }
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);
            break;

        case 6: //Recibes puntos erde vida. LP = 2 * d(10)
            cout<< endl;
            vida = 2*dado.getRandomDice2();
            dañoDobleHechizo = 0;
            p.setLp(p.getLp()+(vida));
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);

            break;

        case 7:
            daño = dado.getRandomDice2();
            cout << "Un rayo eléctrico golpea al monstruo, haciendo daño progresivo." << endl;
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            conteoN10 = 2; // Daño reducido para el monstruo en los próximos 2 turnos
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);

            break;

        case 8: // Refleja el daño recibido este turno
            cout << "Reflejas el ataque del monstruo hacia él." << endl;
            daño = 300;
            dañoDobleHechizo = daño;
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);

            break;

        case 9: // Incrementa el daño de tus ataques futuros
            cout << "otorgandote una cantidad considerable de vida. LP = 500" << endl;
            vida = 500;
            dañoDobleHechizo = 0;
            p.setLp(p.getLp() + vida);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);


            break;

        case 10: // Reduce a la mitad el daño del monstruo por 3 turnos
            if(conteoN10 == 0) {
                conteoN10 = conteoN10 + 2;
            }
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);

            break;

        case 11: // Crea un escudo que bloquea daño por un turno
            cout << "Crea un escudo mágico que bloquea el daño del próximo ataque." << endl;
            daño = 50 + dado.getRandomDice2();
            dañoDobleHechizo = daño;
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);

            break;

        case 12: // Recupera vida adicional si tu vida está baja
            dañoDobleHechizo = 0;
            if(invisibilidad == true) {
                dañoM = 0;
                cout<<"Monstruo: Grsss, en donde estas?"<<endl;
                invisibilidad = false;
            }
            cout<<"¡¿Pero qué esta pasando?!, el mosnstruo se esta desintegrando"<<endl;

            m.setLp(max(0,m.getLp() - 999));

            cout<<"Supongo que esto es una victoria para mi"<<endl;


            m.display();
            p.display();
            break;

        case 13: // Golpe crítico al monstruo con alta probabilidad de daño
            daño = dado.getRandomDice2();
            cout << "Golpe crítico lanzado con gran fuerza." << endl;
            p.setLp(p.getLp()+daño);
            aplicarTurnoPlayer(m, p, daño, invisibilidad);
            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m,p,dañoM,invisibilidad,conteoN10);

            break;

        case 14: // Daña al monstruo y recupera parte de tu vida
            daño = dado.getRandomDice2() * turno;
            dañoDobleHechizo = daño;
            aplicarTurnoPlayer(m, p, daño, invisibilidad);

            dañoM = dado.getRandomDice1();
            aplicarTurnoMonster(m, p, dañoM, invisibilidad, conteoN10);

            break;

        default:
            cout << "Opción no válida" << endl;
            return;
    }
    
    // Mostrar el estado actualizado del juego
    m.display();
    p.display();
}

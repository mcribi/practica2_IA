#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

/////////////////////////////////////////////////  N0  //////////////////////////////////////////////
class stateN0{
    public:
    ubicacion jugador;
    ubicacion sonambulo;

    
    virtual bool operator==(const stateN0 &st) const{
        return (jugador.f == st.jugador.f && jugador.c == st.jugador.c &&
        sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c &&
        jugador.brujula == st.jugador.brujula && sonambulo.brujula == st.sonambulo.brujula);
    }

    virtual bool operator<(const stateN0 &st) const
    {
        return (jugador.f < st.jugador.f ||
               (jugador.f == st.jugador.f && jugador.c < st.jugador.c) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula < st.jugador.brujula));
    }
};


struct nodeN0{
  stateN0 st;
  list<Action> secuencia;

  bool operator==(const nodeN0 &nd) const{
    return (st == nd.st);
  }

  bool operator<(const nodeN0 &nd) const
    {
        return (st < nd.st);
    }
};


/////////////////////////////////////////////////  N1  //////////////////////////////////////////////
class stateN1 : public stateN0 {
    
    public:
    bool operator==(const stateN1 &st) const{
        return (jugador.f == st.jugador.f && jugador.c == st.jugador.c &&
        sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c &&
        jugador.brujula == st.jugador.brujula && sonambulo.brujula == st.sonambulo.brujula);
    }

    bool operator<(const stateN1 &st) const
    {
        return ((jugador.f < st.jugador.f) ||
               (jugador.f == st.jugador.f && jugador.c < st.jugador.c) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula < st.jugador.brujula) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && sonambulo.f < st.sonambulo.f)||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && sonambulo.f == st.sonambulo.f && sonambulo.c < st.sonambulo.c)||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c && sonambulo.brujula < st.sonambulo.brujula)   
               );
    }
};


struct nodeN1{
  stateN1 st;
  list<Action> secuencia;

  bool operator==(const nodeN1 &nd) const{
    return (st == nd.st);
  }

  bool operator<(const nodeN1 &nd) const
    {
        return (st < nd.st);
    }
};


/////////////////////////////////////////////////  N2  //////////////////////////////////////////////
class stateN2 : public stateN1 {//cada nivel deriva del anterior
    
    public:

    bool bikini=false;
    bool zapatillas=false;

    bool operator==(const stateN2 &st) const{ //metido bikini/zapatillas
        return (jugador.f == st.jugador.f && jugador.c == st.jugador.c &&
        sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c &&
        jugador.brujula == st.jugador.brujula && sonambulo.brujula == st.sonambulo.brujula && st.bikini==bikini && st.zapatillas==zapatillas);
    }

    bool operator<(const stateN2 &st) const  //metido bikini/zapatillas
    {
        return ((jugador.f < st.jugador.f) ||
               (jugador.f == st.jugador.f && jugador.c < st.jugador.c) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula < st.jugador.brujula) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini<st.bikini) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas<st.zapatillas)
               );
    }
};


struct nodeN2{
  stateN2 st;
  int coste;
  int heuristica; //Planteo un A* 
  list<Action> secuencia;

  bool operator==(const nodeN2 &nd) const{
    return (st == nd.st);
  }

  bool operator<(const nodeN2 &nd) const
    {
        return (st < nd.st);
    }

  bool operator>(const nodeN2 &nd) const
    {
        return (coste+heuristica> nd.coste+nd.heuristica);
    }
};

/////////////////////////////////////////////////  N3  //////////////////////////////////////////////
class stateN3 : public stateN2 {//cada nivel deriva del anterior
    
    public:

    bool bikini_son=false;
    bool zapatillas_son=false;
    
    bool operator==(const stateN3 &st) const{ //metido bikini/zapatillas
        return (jugador.f == st.jugador.f && jugador.c == st.jugador.c &&
        sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c &&
        jugador.brujula == st.jugador.brujula && sonambulo.brujula == st.sonambulo.brujula && st.bikini==bikini && st.zapatillas==zapatillas && st.bikini_son==bikini_son && st.zapatillas_son==zapatillas_son);
    }

    bool operator<(const stateN3 &st) const  //metido bikini/zapatillas
    {
        return ((jugador.f < st.jugador.f) ||
               (jugador.f == st.jugador.f && jugador.c < st.jugador.c) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula < st.jugador.brujula) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini<st.bikini) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas<st.zapatillas) ||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas== st.zapatillas && sonambulo.f < st.sonambulo.f)||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas== st.zapatillas && sonambulo.f == st.sonambulo.f && sonambulo.c < st.sonambulo.c)||
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas== st.zapatillas && sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c && sonambulo.brujula < st.sonambulo.brujula) || 
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas== st.zapatillas && sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c && sonambulo.brujula == st.sonambulo.brujula && bikini_son<st.bikini_son) || 
               (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini==st.bikini && zapatillas== st.zapatillas && sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c && sonambulo.brujula == st.sonambulo.brujula && bikini_son==st.bikini_son && zapatillas_son<st.zapatillas_son)
               );
    }
};


struct nodeN3{
  stateN3 st;
  int coste;
  int heuristica; //Planteo un A*
  list<Action> secuencia;

  bool operator==(const nodeN3 &nd) const{
    return (st == nd.st);
  }

  bool operator<(const nodeN3 &nd) const
    {
        return (st < nd.st);
    }

  bool operator>(const nodeN3 &nd) const
    {
        return (coste+heuristica> nd.coste+nd.heuristica);
    }
};



class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) { //para nivel 4
      // Inicializar Variables de Estado
      hayPlan=false;
      hayPlan_jug=false;
      hayPlan_son=false;
      primer_mov=true;
      segundo_mov=false;
      this->size=size;
      last_action=actIDLE;
      vuelve_a_visualizar=false;
      voy_sonambulo=false;
      he_recargado=false;
      en_recarga=false;
      ciclos=2999;
      ultima_recarga=0;
      no_encuentro_bateria=true;
      sonambulo_sin_sol=false;
      sonambuloInalcanzable=false;
      ya_he_encontrado_recarga=false;
      //Sabemos que hay 3x3 de precipicio
      for (int i=0; i<size; i++){
        for (int j=0; j<size; j++){
          if (i<3 or j<3 or j>(size-4) or i>(size-4))
            mapaResultado[i][j]='P';
        }
      }
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) { //para nivel 0,1,2,3
      // Inicializar Variables de Estado
      hayPlan=false;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);


  private:
    // Declarar Variables de Estado
    list<Action> plan;  //Almacena el plan en ejecución
    list<Action> plan_son;  //Almacena el plan en ejecución
    list<Action> plan_jug;  //Almacena el plan en ejecución
    bool hayPlan; //Si verdad infica que se está siguiendo un plan
    bool hayPlan_jug;
    bool  hayPlan_son;
    //stateN0 c_state;
    bool sonambulo_sin_sol;
    stateN3 c_state;
    ubicacion goal_jug;
    ubicacion goal_son;
    ubicacion goal;
    bool primer_mov;
    bool segundo_mov;
    int size;
    bool voy_sonambulo;
    Action last_action;
    bool vuelve_a_visualizar;
    bool he_recargado;
    bool en_recarga;
    int ciclos;
    int ultima_recarga;
    bool no_encuentro_bateria;
    bool sonambuloInalcanzable;
    bool ya_he_encontrado_recarga;



void VisualizaPlan(const stateN0 &st, const list<Action> &plan);

};


bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa);
ubicacion NextCasilla(const ubicacion &pos);
stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa);
stateN1 apply(const Action &a, const stateN1 &st, const vector<vector<unsigned char>> &mapa);
stateN2 apply(const Action &a, const stateN2 &st, const vector<vector<unsigned char>> &mapa);
stateN3 apply(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa);
stateN3 applyN4(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa);
//bool Find(const stateN0 &item, const list<stateN0> &lista);
//bool Find(const stateN0 &item, const list<nodeN0> &lista);
void AnularMatriz(vector<vector<unsigned char>> &matriz);
bool veoSonambulo(const ubicacion &jugador, const ubicacion &sonambulo);
//Nivel 0
bool AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
//Nivel 1
bool AnchuraSonambulo(const stateN1 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
//Nivel 2
bool a_estrella_manhattan(const stateN2 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
int distanciaManhattan(const ubicacion &inicio, const ubicacion &final);
int distanciaChebychev(const ubicacion &inicio, const ubicacion &final);
int costeActualN2(Action accion, const ubicacion &inicio, const vector<vector<unsigned char>> &mapa, const stateN2 &estado);
int costeActualN3(Action accion, const ubicacion &inicio, const vector<vector<unsigned char>> &mapa, const stateN3 &estado);
void actualizarObjetos(nodeN2 &current_node, const vector<vector<unsigned char>> &mapa);
void actualizarObjetosJugador(nodeN3 &current_node, const vector<vector<unsigned char>> &mapa);
void actualizarObjetosSonambulo(nodeN3 &current_node, const vector<vector<unsigned char>> &mapa );
bool a_estrella_chebychev(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);


//Para el nivel 4
void PonerTerrenoEnMatriz (const stateN3 &st, vector<vector<unsigned char>>&matriz, Sensores sensores);
bool costeUniforme(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa);
//bool CasillaTransitableN4(const ubicacion &x, const vector<vector<unsigned char>> &mapa); 
bool nivel4Jug(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
stateN3 apply_son(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa);
bool nivel4(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
ubicacion buscarEnMatriz (vector<vector<unsigned char>>&matriz, unsigned char letra, const ubicacion &jugador);
bool nivel4Sonambulo(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
bool veoSonambuloN4(const ubicacion &jugador, const ubicacion &sonambulo);
bool libreDelanteSonambulo (const stateN3 &st, vector<vector<unsigned char>>&matriz, Sensores sensores);
char casillaDelanteSonambulo (const stateN3 &st, vector<unsigned char>&terreno);
int mirar_terreno(char casilla, const vector<unsigned char> &terreno);
bool a_estrella_sonambulo(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa);
#endif

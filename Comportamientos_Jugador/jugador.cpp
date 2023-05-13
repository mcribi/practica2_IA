#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"

Action ComportamientoJugador::think(Sensores sensores){
    Action accion = actIDLE;
	if (sensores.nivel!=4){	//NIvel 0,1,2,3
		if (!hayPlan){
			cout << "Calculando plan..." << endl;
			c_state.jugador.f = sensores.posF;
			c_state.jugador.c = sensores.posC;
			c_state.jugador.brujula = sensores.sentido;
			c_state.sonambulo.f = sensores.SONposF;
			c_state.sonambulo.c = sensores.SONposC;
			c_state.sonambulo.brujula = sensores.SONsentido;
			goal.f = sensores.destinoF;
			goal.c = sensores.destinoC;
			//para el sonambulo
			ubicacion ubi_sonam;
			ubi_sonam.brujula= sensores.SONsentido;
			ubi_sonam.f = sensores.SONposF;
			ubi_sonam.c = sensores.SONposC;

			switch(sensores.nivel){
				case 0: //PLAN NIVEL 0
					hayPlan=AnchuraSoloJugador(c_state, goal, plan, mapaResultado);
					break;
				case 1://PLAN NIVEL 1
					hayPlan=AnchuraSonambulo(c_state, goal, plan, mapaResultado);
					break;
				case 2://PLAN NIVEL 2
					hayPlan=a_estrella_manhattan(c_state, goal, plan, mapaResultado);
					break;
				case 3:
					//PLAN NIVEL 3
					hayPlan=a_estrella_chebychev(c_state, goal, plan, mapaResultado);
					break;
			}
			if (plan.size()>0){
				VisualizaPlan(c_state, plan);
			}
		}
		if (hayPlan and plan.size()>0){
			cout << "Ejecutando siguietne acción del plan" << endl;
			accion = plan.front();
			plan.pop_front();
		}

		if (plan.size()== 0){
			cout << "Se completó el plan" << endl;
			hayPlan = false;
		}
	}else{	
		if (primer_mov){
			accion=actWHEREIS;
			primer_mov=false;
			segundo_mov=true;

			if (sensores.terreno[0]=='B'){
					c_state.bikini=true;
					c_state.zapatillas=false;
			}else if (sensores.terreno[0]=='K'){
					c_state.zapatillas=true;
					c_state.bikini=false;
			}
		}
		else{
			if (segundo_mov){	//el actWHEREIS sabemos las posiciones en el segundo movimiento
				c_state.jugador.f = sensores.posF;
				c_state.jugador.c = sensores.posC;
				c_state.jugador.brujula = sensores.sentido;
				c_state.sonambulo.f = sensores.SONposF;
				c_state.sonambulo.c = sensores.SONposC;
				c_state.sonambulo.brujula = sensores.SONsentido;
				//goal.f = sensores.destinoF;
				//goal.c = sensores.destinoC;
				goal.f = c_state.sonambulo.f;
				goal.c = c_state.sonambulo.c;
				segundo_mov=false;
				
				/*PonerTerrenoEnMatriz(c_state, mapaResultado, sensores );
 				hayPlan=nivel4(c_state, goal, plan, mapaResultado);
				
				
				if (plan.size()>0){
					VisualizaPlan(c_state, plan);
				}

				if (hayPlan and plan.size()>0){
					cout << "Ejecutando siguietne acción del plan" << endl;
					accion = plan.front();
					plan.pop_front();
					c_state =applyN4(accion,c_state,mapaResultado);
				}

				if (plan.size()== 0){
					cout << "Se completó el plan" << endl;
					hayPlan = false;
				}*/
				//if (sensores.terreno[0]=='B'){
				if (mapaResultado[c_state.jugador.f][c_state.jugador.c]=='K'){
					c_state.bikini=true;
					c_state.zapatillas=false;
					cout<<"cojo bikini"<<endl;
				//}else if (sensores.terreno[0]=='D'){
				}else if (mapaResultado[c_state.jugador.f][c_state.jugador.c]=='D'){
					c_state.zapatillas=true;
					c_state.bikini=false;
					cout<<"cojo zapas"<<endl;
				}
				accion=actIDLE;
			}
			else{	//a partir del 3 mov
				if(!sensores.colision)	//solo actualizo si no se ha chocado
					c_state = applyN4(last_action, c_state, mapaResultado);
				
				//if (sensores.terreno[0]=='B'){
				if (mapaResultado[c_state.jugador.f][c_state.jugador.c]=='K'){
					c_state.bikini=true;
					c_state.zapatillas=false;
					cout<<"cojo bikini"<<endl;
				//}else if (sensores.terreno[0]=='D'){
				}else if (mapaResultado[c_state.jugador.f][c_state.jugador.c]=='D'){
					c_state.zapatillas=true;
					c_state.bikini=false;
					cout<<"cojo zapas"<<endl;
				}

				if (!hayPlan and !sensores.colision){
					
					//si no veo al sonmabulo y no estoy yendo a por el
					/*if (!veoSonambulo(c_state.jugador, c_state.sonambulo)){
						//cout<<"veo al sonambulo"<<endl;
						
						if(!voy_sonambulo){	//si no estoy yendo
							cout<<"acabo de ver al sonmabulo"<<endl;
							//mi destino es el sonmabulo
							goal.f = c_state.sonambulo.f;
							goal.c = c_state.sonambulo.c;
							//plan.clear();
							hayPlan=AnchuraSonambulo(c_state, goal, plan, mapaResultado);
							voy_sonambulo=true;
						}
						if(voy_sonambulo){
							cout<<"estoy yendo a por el sonmabulo"<<endl;

						}
						
					}else if (veoSonambulo(c_state.jugador, c_state.sonambulo)){//si veo al sonambulo le doy instrucciones

					}*/
					
					
					//default
					
					//plan.clear();
					PonerTerrenoEnMatriz(c_state, mapaResultado, sensores);
					hayPlan=nivel4(c_state, goal, plan, mapaResultado);
					//mando al sonmabulo
					//hayPlan=nivel4Sonambulo(c_state, goal, plan, mapaResultado);
					

					if (plan.size()>0 ){
						VisualizaPlan(c_state, plan);
					}
					
				}

				//si veo a un aldeano o a un lobo recalculo
				if ((sensores.superficie[2]=='a' or sensores.superficie[2]=='l') and !sensores.colision){
					if (plan.size()>0)
						plan.clear();
					hayPlan=nivel4(c_state, goal, plan, mapaResultado);
					//hayPlan=false;
				}

				//si veo precipicio recalculo
				if((sensores.terreno[2]=='P' or sensores.terreno[2]=='M' or (sensores.terreno[2]=='B' and !c_state.zapatillas) or (sensores.terreno[2]=='A' and !c_state.bikini) or (c_state.jugador.f==c_state.sonambulo.f and c_state.jugador.c==c_state.sonambulo.c)) and last_action==actFORWARD){
					if (plan.size()>0)
						plan.clear();
					//hayPlan=false;
					hayPlan=nivel4(c_state, goal, plan, mapaResultado);	//AQUI ERROR
				}
				

				//quedan pocos instantes vamos lo mas directos posibles (con anchura)
				if (ciclos<85){
					cout<<"quedan pocos instantes"<<endl,
					plan.clear();
					hayPlan=AnchuraSoloJugador(c_state, goal, plan, mapaResultado);
				}

				//me hace falta recargar y la ultima recarga no ha sido hace poco
				if (((ciclos-sensores.bateria)>2000 or sensores.bateria<1000) and abs(ultima_recarga-ciclos)>=100 and ciclos>300){
					cout<<"necesito recarga"<<endl;
					ya_he_encontrado_recarga=false;
					if (!ya_he_encontrado_recarga){//voy a por recarga
						ubicacion recarga = buscarEnMatriz(mapaResultado, 'X', c_state.jugador);
						if (recarga.f>0 and recarga.c>0){	//si encuentro recarga
							//goal.f=recarga.f;
							//goal.c=recarga.c;
							plan.clear();
							hayPlan=nivel4(c_state, recarga, plan, mapaResultado);
							//hayPlan=false;
							ya_he_encontrado_recarga=true;
						}
					}
					//encuentro recarga
					if (mapaResultado[c_state.jugador.f][c_state.jugador.c]=='X'){
						cout<<"estoy recargando"<<endl;
						for (int i=0; i<130; i++){
								plan.push_back(actIDLE);//obligamos a que recargue un poco
						}
						ultima_recarga=ciclos;
						he_recargado=true;
						
					}
				}else
					ya_he_encontrado_recarga=false;
				

				//si se ha llegado al objetivo
				if (plan.size()== 0){
					cout << "Se completó el plan" << endl;
					hayPlan = false;
					goal.f = sensores.destinoF;
					goal.c = sensores.destinoC;
				}

				//si ha colisionado recalculamos
				if (sensores.colision){	//me ha dado un lobo
					cout<<"lobo diria"<<endl;
					plan.clear();
					accion=actWHEREIS;
					segundo_mov=true;	//volvemos a posicionarnos
					//hayPlan=nivel4(c_state, goal, plan, mapaResultado);
					hayPlan=false;
				}
				
				

				if(sensores.bateria<600 and mapaResultado[c_state.jugador.f][c_state.jugador.c]=='X'){
					cout<<"recargo a al desesperada"<<endl;
					//accion=actIDLE;
					if (plan.size()>0)
						plan.clear();
					//hayPlan=false;

					for (int i=0; i<100; i++){
						plan.push_back(actIDLE);//obligamos a que recargue un poco
					}
				}
				

				if (hayPlan and plan.size()>0){
					cout << "Ejecutando siguietne acción del plan" << endl;
					VisualizaPlan(c_state, plan);
					accion = plan.front();
					
					//si me voy a chocar recalculo
					if((sensores.terreno[2]=='M' or sensores.terreno[2]=='P') and accion==actFORWARD){
						cout<<"delante lmuro/precicipio y quiero avanzar"<<endl;
						if (plan.size()>0)
							plan.clear();
						
						hayPlan=nivel4(c_state, goal, plan, mapaResultado);
						accion = plan.front();
						plan.pop_front();
					}else if ((sensores.superficie[2]=='a' or sensores.superficie[2]=='l') and accion==actFORWARD){
						cout<<"delante lobo/aldeano y quiero avanzar"<<endl;
						accion=actIDLE;
					}else{
						plan.pop_front();
					}
					PonerTerrenoEnMatriz(c_state, mapaResultado, sensores);
				}
			}
		}
	}
	//if(!he_recargado)
	//	ultima_recarga=ciclos;
	last_action=accion;
	ciclos--;
    return accion;
}


int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

/** Devuelve si una ubicación en el mapa es transitable para el agente */
bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa)
{
	return (mapa[x.f][x.c] != 'P' and mapa[x.f][x.c] != 'M' and x.f>2 and x.f<(mapa.size()-3) and x.c>2 and x.c<(mapa.size()-3));
}


/** Devuelve la ubicación siguiente según el avance del agente*/
ubicacion NextCasilla(const ubicacion &pos)
{
	ubicacion next = pos;
	switch (pos.brujula)
	{
	case norte:
		next.f = pos.f - 1;
		break;
	case noreste:
		next.f = pos.f - 1;
		next.c = pos.c + 1;
		break;
	case este:
		next.c = pos.c + 1;
		break;
	case sureste:
		next.f = pos.f + 1;
		next.c = pos.c + 1;
		break;
	case sur:
		next.f = pos.f + 1;
		break;
	case suroeste:
		next.f = pos.f + 1;
		next.c = pos.c - 1;
		break;
	case oeste:
		next.c = pos.c - 1;
		break;
	case noroeste:
		next.f = pos.f - 1;
		next.c = pos.c - 1;
		break;
	default:
		break;
	}

	return next;
}

/** Devuelve el estado que se genera si el agente puede avanzar. EN EL NIVEL 0
 * Si no puede avanzar, se devuelve como salida el mismo estado de entrada.
 */
stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa)
{
  stateN0 st_result = st;
  ubicacion sig_ubicacion;
  switch (a){
  case actFORWARD: // si casilla delante es transitable y no está ocupada por el sonámbulo
    sig_ubicacion = NextCasilla(st.jugador);
    if (CasillaTransitable(sig_ubicacion, mapa) and !(sig_ubicacion.f == st.sonambulo.f and sig_ubicacion.c == st.sonambulo.c )){
      st_result.jugador = sig_ubicacion;
    }
    break;
  case actTURN_L:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
    break;

  case actTURN_R:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
    break;
  }
  return st_result;
}

/** Devuelve el estado que se genera si el agente puede avanzar. EN EL NIVEL 1
 * Si no puede avanzar, se devuelve como salida el mismo estado de entrada.
 */
stateN1 apply(const Action &a, const stateN1 &st, const vector<vector<unsigned char>> &mapa)
{
  stateN1 st_result = st;
  ubicacion sig_ubicacion;
  switch (a){
  case actFORWARD: // si casilla delante es transitable y no está ocupada por el sonmabulo
    sig_ubicacion = NextCasilla(st.jugador);
    if (CasillaTransitable(sig_ubicacion, mapa) and !(sig_ubicacion.f == st.sonambulo.f and sig_ubicacion.c == st.sonambulo.c )){
      st_result.jugador = sig_ubicacion;
    }
    break;
  case actTURN_L:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
    break;

  case actTURN_R:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
    break;

	//acciones para el sonambulo
  case actSON_FORWARD: // si casilla delante es transitable y no está ocupada por el jugaodor
    sig_ubicacion = NextCasilla(st.sonambulo);//solo si veo al sonmabulo le doy ordenes
    if (CasillaTransitable(sig_ubicacion, mapa) and veoSonambulo(st.jugador, st.sonambulo) and !(sig_ubicacion.f == st.jugador.f and sig_ubicacion.c == st.jugador.c )){
     st_result.sonambulo = sig_ubicacion;
    }
    break;
  case actSON_TURN_SL:
    if (veoSonambulo(st.jugador, st.sonambulo))//solo si veo al sonmabulo le doy ordenes
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 7) % 8);
    break;

  case actSON_TURN_SR:
  if (veoSonambulo(st.jugador, st.sonambulo))//solo si veo al sonmabulo le doy ordenes
      st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 1) % 8);
    break;
  }
  return st_result;
}

/** Devuelve el estado que se genera si el agente puede avanzar. EN EL NIVEL 2
 * Si no puede avanzar, se devuelve como salida el mismo estado de entrada.
 */
stateN2 apply(const Action &a, const stateN2 &st, const vector<vector<unsigned char>> &mapa)
{
  stateN2 st_result = st;
  ubicacion sig_ubicacion;
  switch (a){
  case actFORWARD: // si casilla delante es transitable y no está ocupada por el sonámbulo
    sig_ubicacion = NextCasilla(st.jugador);
    if (CasillaTransitable(sig_ubicacion, mapa) and !(sig_ubicacion.f == st.sonambulo.f and sig_ubicacion.c == st.sonambulo.c )){
      st_result.jugador = sig_ubicacion;
    }
    break;
  case actTURN_L:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
    break;

  case actTURN_R:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
    break;
  }
  return st_result;
}

/** Devuelve el estado que se genera si el agente puede avanzar. EN EL NIVEL 3
 * Si no puede avanzar, se devuelve como salida el mismo estado de entrada.
 */
stateN3 apply(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa)
{
  stateN3 st_result = st;
  ubicacion sig_ubicacion;
  switch (a){
  case actFORWARD: // si casilla delante es transitable y no está ocupada por el sonmabulo
    sig_ubicacion = NextCasilla(st.jugador);
    if (CasillaTransitable(sig_ubicacion, mapa) and !(sig_ubicacion.f == st.sonambulo.f and sig_ubicacion.c == st.sonambulo.c )){
      st_result.jugador = sig_ubicacion;
    }
    break;
  case actTURN_L:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
    break;

  case actTURN_R:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
    break;

	//acciones para el sonambulo
  case actSON_FORWARD: // si casilla delante es transitable y no está ocupada por el jugaodor
    sig_ubicacion = NextCasilla(st.sonambulo);//solo si veo al sonmabulo le doy ordenes
    if (CasillaTransitable(sig_ubicacion, mapa) and veoSonambulo(st.jugador, st.sonambulo) and !(sig_ubicacion.f == st.jugador.f and sig_ubicacion.c == st.jugador.c )){
     st_result.sonambulo = sig_ubicacion;
    }
    break;
  case actSON_TURN_SL:
    if (veoSonambulo(st.jugador, st.sonambulo))//solo si veo al sonmabulo le doy ordenes
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 7) % 8);
    break;

  case actSON_TURN_SR:
  if (veoSonambulo(st.jugador, st.sonambulo))//solo si veo al sonmabulo le doy ordenes
      st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 1) % 8);
    break;
  }
  return st_result;
}

/** Devuelve el estado que se genera si el agente puede avanzar. EN EL NIVEL 4
 * Si no puede avanzar, se devuelve como salida el mismo estado de entrada.
 */
stateN3 applyN4(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa)
{
  stateN3 st_result = st;
  ubicacion sig_ubicacion;
  switch (a){
  case actFORWARD: // si casilla delante es transitable y no está ocupada por el sonámbulo
    sig_ubicacion = NextCasilla(st.jugador);
    if (CasillaTransitable(sig_ubicacion,mapa) and !(sig_ubicacion.f == st.sonambulo.f and sig_ubicacion.c == st.sonambulo.c )){
      st_result.jugador = sig_ubicacion;
    }
    break;
  case actTURN_L:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
    break;

  case actTURN_R:
    st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
    break;
	
  //acciones para el sonambulo
  case actSON_FORWARD: // si casilla delante es transitable y no está ocupada por el jugaodor
    sig_ubicacion = NextCasilla(st.sonambulo);//solo si veo al sonmabulo le doy ordenes
    if (CasillaTransitable(sig_ubicacion, mapa) and veoSonambulo(st.jugador, st.sonambulo) and !(sig_ubicacion.f == st.jugador.f and sig_ubicacion.c == st.jugador.c )){
     st_result.sonambulo = sig_ubicacion;
    }
    break;
  case actSON_TURN_SL:
    if (veoSonambulo(st.jugador, st.sonambulo))//solo si veo al sonmabulo le doy ordenes
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 7) % 8);
    break;

  case actSON_TURN_SR:
  if (veoSonambulo(st.jugador, st.sonambulo))//solo si veo al sonmabulo le doy ordenes
      st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 1) % 8);
    break;
  }
  return st_result;
}


/** Permite pintar sobre el mapa del simulador el plan partiendo desde el estado st */
void ComportamientoJugador::VisualizaPlan(const stateN0 &st, const list<Action> &plan)
{
  AnularMatriz(mapaConPlan);
  stateN0 cst = st;

  auto it = plan.begin();
  while (it != plan.end()){
    switch (*it){
      case actFORWARD:
        cst.jugador = NextCasilla(cst.jugador);
	  mapaConPlan[cst.jugador.f][cst.jugador.c] = 1;
	  break;
	case actTURN_R:
        cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 2) % 8);
	  break;
	case actTURN_L:
	  cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 6) % 8);
	  break;
	case actSON_FORWARD:
	  cst.sonambulo = NextCasilla(cst.sonambulo);
	  mapaConPlan[cst.sonambulo.f][cst.sonambulo.c] = 2;
	  break;
	case actSON_TURN_SR:
	  cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 1) % 8);
	  break;
	case actSON_TURN_SL:
	  cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 7) % 8);
	  break;
    }
    it++;
  }
}

/** pone a cero todos los elementos de una matriz */
void AnularMatriz(vector<vector<unsigned char>> &matriz)
{
  for (int i = 0; i < matriz.size(); i++)
    for (int j = 0; j < matriz[0].size(); j++)
      matriz[i][j] = 0;
}

/** tercera aproximación a la implimentación de la busqueda en anchura */
bool AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa)
{
	nodeN0 current_node;
	current_node.st = inicio;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	bool SolutionFound = (current_node.st.jugador.f == final.f and current_node.st.jugador.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWARD
		nodeN0 child_forward = current_node; 
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (child_forward.st.jugador.f == final.f and child_forward.st.jugador.c == final.c)
		{
			child_forward.secuencia.push_back(actFORWARD);  
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward)==explored.end())
		{
			child_forward.secuencia.push_back(actFORWARD);
			frontier.push_back(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hijo actTURN_L
			nodeN0 child_turnl = current_node;  
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}
			// Generar hijo actTURN_R
			nodeN0 child_turnr = current_node; 
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while(!frontier.empty() and explored.find(current_node)!=explored.end()){
				frontier.pop_front();
				if(!frontier.empty())
					current_node=frontier.front();
			}
		}
	}

	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;
}

/** aproximación a la implimentación de la busqueda en anchura */
bool AnchuraSonambulo(const stateN1 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa)
{
	nodeN1 current_node;
	current_node.st = inicio;
	list<nodeN1> frontier;
	set<nodeN1> explored;
	bool SolutionFound = (current_node.st.sonambulo.f == final.f and current_node.st.sonambulo.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWARD SONAMBULO
		nodeN1 child_forward_son = current_node; 
		child_forward_son.st = apply(actSON_FORWARD, current_node.st, mapa);
		if (child_forward_son.st.sonambulo.f == final.f and child_forward_son.st.sonambulo.c == final.c)
		{
			SolutionFound = true;
			child_forward_son.secuencia.push_back(actSON_FORWARD);  
			current_node = child_forward_son;
			
		}
		else if (explored.find(child_forward_son)==explored.end())
		{
			child_forward_son.secuencia.push_back(actSON_FORWARD);
			frontier.push_back(child_forward_son);
		}
		
		

		if (!SolutionFound)
		{	
			// Generar hijo actFORWARD JUGADOR
			nodeN1 child_forward = current_node; 
			child_forward.st = apply(actFORWARD, current_node.st, mapa);
		
			if (explored.find(child_forward)==explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				frontier.push_back(child_forward);
			}

			// Generar hijo actTURN_L SONAMBULO
			nodeN1 child_turnl_son = current_node;  
			child_turnl_son.st = apply(actSON_TURN_SL, current_node.st, mapa);
			if (explored.find(child_turnl_son)==explored.end())
			{
				child_turnl_son.secuencia.push_back(actSON_TURN_SL);
				frontier.push_back(child_turnl_son);
			}
			// Generar hijo actTURN_R SONAMBULO
			nodeN1 child_turnr_son = current_node; 
			child_turnr_son.st = apply(actSON_TURN_SR, current_node.st, mapa);
			if (explored.find(child_turnr_son)==explored.end())
			{
				child_turnr_son.secuencia.push_back(actSON_TURN_SR);
				frontier.push_back(child_turnr_son);
			}
			// Generar hijo actTURN_L JUGADOR
			nodeN1 child_turnl = current_node;  
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}
			// Generar hijo actTURN_R JUGADOR
			nodeN1 child_turnr = current_node; 
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while(!frontier.empty() and explored.find(current_node)!=explored.end()){
				frontier.pop_front();
				if(!frontier.empty())
					current_node=frontier.front();
			}
		}
	}

	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;
}
/** funcion para el nivel 4 */
bool nivel4(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa)
{	//solo es con jugador
	nodeN3 objetivo;
	objetivo.st.jugador=final;
	nodeN3 current_node;
	current_node.st = inicio;
	current_node.coste=0;	//inicializo el coste
	actualizarObjetosJugador(current_node, mapa);//por si empieza en bikini/zapas
	priority_queue<nodeN3, vector<nodeN3>, greater<nodeN3>> frontier;
	set<stateN3> explored;
	bool SolutionFound = (current_node.st.jugador.f == final.f and current_node.st.jugador.c == final.c);
	frontier.push(current_node);//meto en abiertos el nodo inicial (no miro coste pq es el unico que hay)

	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop();	//saca el de menor coste
		explored.insert(current_node.st);//lo meto en cerrados y ahora tengo que expandir:

		if (!SolutionFound)
		{
			// Generar hijo actFORWARD
			nodeN3 child_forward = current_node; 

			actualizarObjetosJugador(child_forward, mapa);
			child_forward.st = applyN4(actFORWARD, current_node.st, mapa);
			child_forward.coste+=costeActualN3(actFORWARD, current_node.st.jugador,mapa,child_forward.st);
			child_forward.heuristica=distanciaManhattan(child_forward.st.jugador, final);
			if (explored.find(child_forward.st)==explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				actualizarObjetosJugador(child_forward, mapa);
				frontier.push(child_forward);
			}

			// Generar hijo actTURN_R
			nodeN3 child_turnr = current_node; 
			actualizarObjetosJugador(child_turnr, mapa);
			child_turnr.st = applyN4(actTURN_R, current_node.st, mapa);
			child_turnr.coste+=costeActualN2(actTURN_R, current_node.st.jugador,mapa,child_turnr.st);
			child_turnr.heuristica=distanciaManhattan(child_turnr.st.jugador, final);
			

			if (explored.find(child_turnr.st)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				actualizarObjetosJugador(child_turnr, mapa);
				frontier.push(child_turnr);
			}

			// Generar hijo actTURN_L
			nodeN3 child_turnl = current_node;  
			actualizarObjetosJugador(child_turnl, mapa);
			child_turnl.st = applyN4(actTURN_L, current_node.st, mapa);
			child_turnl.coste+=costeActualN2(actTURN_L, current_node.st.jugador,mapa,child_turnl.st);
			child_turnl.heuristica=distanciaManhattan(child_turnl.st.jugador, final);

			if (explored.find(child_turnl.st)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				actualizarObjetosJugador(child_turnl, mapa);
				frontier.push(child_turnl);
			}	

		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.top();
			while(!frontier.empty() and explored.find(current_node.st)!=explored.end()){
				frontier.pop();
				if(!frontier.empty()){
					current_node=frontier.top();
					actualizarObjetosJugador(current_node, mapa);
				}
			}
		}

		if (current_node.st.jugador.c==final.c && current_node.st.jugador.f==final.f){
			actualizarObjetosJugador(current_node, mapa);	
			SolutionFound = true;
		}
	}

	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;
}


/** funcion de A* para calcular el minimo coste con la distancia Manhattan (NIVEL 2) */
bool a_estrella_manhattan(const stateN2 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa)
{	//solo es con jugador
	nodeN2 objetivo;
	objetivo.st.jugador=final;
	nodeN2 current_node;
	current_node.st = inicio;
	current_node.coste=0;	//inicializo el coste
	//current_node.heuristica=0;
	actualizarObjetos(current_node, mapa);//por si empieza en bikini/zapas
	priority_queue<nodeN2, vector<nodeN2>, greater<nodeN2>> frontier;
	set<stateN2> explored;
	bool SolutionFound = (current_node.st.jugador.f == final.f and current_node.st.jugador.c == final.c);
	frontier.push(current_node);//meto en abiertos el nodo inicial (no miro coste pq es el unico que hay)

	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop();	//saca el de menor coste
		explored.insert(current_node.st);//lo meto en cerrados y ahora tengo que expandir:

		if (!SolutionFound)
		{
			// Generar hijo actFORWARD
			nodeN2 child_forward = current_node; 

			actualizarObjetos(child_forward, mapa);
			child_forward.st = apply(actFORWARD, current_node.st, mapa);
			child_forward.coste+=costeActualN2(actFORWARD, current_node.st.jugador,mapa,child_forward.st);
			child_forward.heuristica=distanciaManhattan(child_forward.st.jugador, final);
			if (explored.find(child_forward.st)==explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				actualizarObjetos(child_forward, mapa);
				frontier.push(child_forward);
			}

			// Generar hijo actTURN_R
			nodeN2 child_turnr = current_node; 
			actualizarObjetos(child_turnr, mapa);
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			child_turnr.coste+=costeActualN2(actTURN_R, current_node.st.jugador,mapa,child_turnr.st);
			child_turnr.heuristica=distanciaManhattan(child_turnr.st.jugador, final);
			

			if (explored.find(child_turnr.st)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				actualizarObjetos(child_turnr, mapa);
				frontier.push(child_turnr);
			}

			// Generar hijo actTURN_L
			nodeN2 child_turnl = current_node;  
			actualizarObjetos(child_turnl, mapa);
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			child_turnl.coste+=costeActualN2(actTURN_L, current_node.st.jugador,mapa,child_turnl.st);
			child_turnl.heuristica=distanciaManhattan(child_turnl.st.jugador, final);

			if (explored.find(child_turnl.st)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				actualizarObjetos(child_turnl, mapa);
				frontier.push(child_turnl);
			}	

		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.top();
			while(!frontier.empty() and explored.find(current_node.st)!=explored.end()){
				frontier.pop();
				if(!frontier.empty()){
					current_node=frontier.top();
					actualizarObjetos(current_node, mapa);
				}
			}
		}

		if (current_node.st.jugador.c==final.c && current_node.st.jugador.f==final.f){
			actualizarObjetos(current_node, mapa);	
			SolutionFound = true;
		}
	}

	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;
}

/** funcion de A* para calcular el minimo coste con la distancia de Chebycheb (NIVEL 3) */
bool a_estrella_chebychev(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa)
{
	nodeN3 current_node;
	current_node.st = inicio;
	current_node.coste=0;	//inicializo el coste
	actualizarObjetosJugador(current_node, mapa);//por si empieza en bikini/zapas
	actualizarObjetosSonambulo(current_node, mapa);//por si empieza en bikini/zapas
	priority_queue<nodeN3, vector<nodeN3>, greater<nodeN3>> frontier;
	set<stateN3> explored;
	bool SolutionFound = (current_node.st.sonambulo.f == final.f and current_node.st.sonambulo.c == final.c);
	frontier.push(current_node);

	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop();	//saco el de menor coste y lo meto en cerrados
		explored.insert(current_node.st);
		actualizarObjetosJugador(current_node, mapa);//por si empieza en bikini/zapas
		actualizarObjetosSonambulo(current_node, mapa);//por si empieza en bikini/zapas
		if (!SolutionFound)
		{
			// Generar hijo actFORWARD SONAMBULO
			nodeN3 child_forward_son = current_node; 
			child_forward_son.st = apply(actSON_FORWARD, current_node.st, mapa);
			child_forward_son.coste+=costeActualN3(actSON_FORWARD, current_node.st.sonambulo,mapa,child_forward_son.st);
			child_forward_son.heuristica=distanciaChebychev(child_forward_son.st.sonambulo, final);
		
			if (explored.find(child_forward_son.st)==explored.end())
			{
				child_forward_son.secuencia.push_back(actSON_FORWARD);
				frontier.push(child_forward_son);
				actualizarObjetosSonambulo(child_forward_son, mapa);
			}

			
		
			// Generar hijo actFORWARD JUGADOR
			nodeN3 child_forward = current_node; 
			child_forward.st = apply(actFORWARD, current_node.st, mapa);
			child_forward.coste+=costeActualN3(actFORWARD, current_node.st.jugador,mapa,child_forward.st);
			child_forward.heuristica=distanciaChebychev(child_forward.st.sonambulo, final);
			if (explored.find(child_forward.st)==explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				frontier.push(child_forward);
				actualizarObjetosJugador(child_forward, mapa);
			}
			// Generar hijo actTURN_L SONAMBULO
			nodeN3 child_turnl_son = current_node;  
			child_turnl_son.st = apply(actSON_TURN_SL, current_node.st, mapa);
			child_turnl_son.coste+=costeActualN3(actSON_TURN_SL, current_node.st.sonambulo,mapa,child_turnl_son.st);
			child_turnl_son.heuristica=distanciaChebychev(child_turnl_son.st.sonambulo, final);
			if (explored.find(child_turnl_son.st)==explored.end())
			{
				child_turnl_son.secuencia.push_back(actSON_TURN_SL);
				frontier.push(child_turnl_son);
				actualizarObjetosSonambulo(child_turnl_son, mapa);
			}
			// Generar hijo actTURN_R SONAMBULO
			nodeN3 child_turnr_son = current_node; 
			child_turnr_son.st = apply(actSON_TURN_SR, current_node.st, mapa);
			child_turnr_son.coste+=costeActualN3(actSON_TURN_SR, current_node.st.sonambulo,mapa,child_turnr_son.st);
			child_turnr_son.heuristica=distanciaChebychev(child_turnr_son.st.sonambulo, final);
			if (explored.find(child_turnr_son.st)==explored.end())
			{
				child_turnr_son.secuencia.push_back(actSON_TURN_SR);
				frontier.push(child_turnr_son);
				actualizarObjetosSonambulo(child_turnr_son, mapa);
			}

			
			// Generar hijo actTURN_L JUGADOR
			nodeN3 child_turnl = current_node; 
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			child_turnl.coste+=costeActualN3(actTURN_L, current_node.st.jugador,mapa,child_turnl.st);
			child_turnl.heuristica=distanciaChebychev(child_turnl.st.sonambulo, final); 
			if (explored.find(child_turnl.st)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push(child_turnl);
				actualizarObjetosJugador(child_turnl, mapa);
			}
			// Generar hijo actTURN_R JUGADOR
			nodeN3 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			child_turnr.coste+=costeActualN3(actTURN_R, current_node.st.jugador,mapa,child_turnr.st);
			child_turnr.heuristica=distanciaChebychev(child_turnr.st.sonambulo, final);
			
			if (explored.find(child_turnr.st)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push(child_turnr);
				actualizarObjetosJugador(child_turnr, mapa);
			}

			
		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.top();
			while(!frontier.empty() and explored.find(current_node.st)!=explored.end()){
				frontier.pop();
				if(!frontier.empty()){
					current_node=frontier.top();
					actualizarObjetosSonambulo(current_node, mapa);	
					actualizarObjetosJugador(current_node, mapa);
				}
			}
		}

		if (current_node.st.sonambulo.c==final.c && current_node.st.sonambulo.f==final.f){
			actualizarObjetosSonambulo(current_node, mapa);	
			actualizarObjetosJugador(current_node, mapa);
			SolutionFound = true;
		}
	}
	

	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;
}

/** funcion de A* para calcular el minimo coste con la distancia de Chebycheb (NIVEL 4) */
bool nivel4Sonambulo(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa)
{
	nodeN3 current_node;
	current_node.st = inicio;
	current_node.coste=0;	//inicializo el coste
	actualizarObjetosJugador(current_node, mapa);//por si empieza en bikini/zapas
	actualizarObjetosSonambulo(current_node, mapa);//por si empieza en bikini/zapas
	priority_queue<nodeN3, vector<nodeN3>, greater<nodeN3>> frontier;
	set<stateN3> explored;
	bool SolutionFound = (current_node.st.sonambulo.f == final.f and current_node.st.sonambulo.c == final.c);
	frontier.push(current_node);
	int nodos_generados=0;

	while (!frontier.empty() and !SolutionFound and nodos_generados<800000)
	{
		frontier.pop();	//saco el de menor coste y lo meto en cerrados
		explored.insert(current_node.st);
		actualizarObjetosJugador(current_node, mapa);//por si empieza en bikini/zapas
		actualizarObjetosSonambulo(current_node, mapa);//por si empieza en bikini/zapas
		if (!SolutionFound)
		{		
		
			// Generar hijo actFORWARD JUGADOR
			nodeN3 child_forward = current_node; 
			nodos_generados++;
			child_forward.st = apply(actFORWARD, current_node.st, mapa);
			child_forward.coste+=costeActualN3(actFORWARD, current_node.st.jugador,mapa,child_forward.st);
			child_forward.heuristica=distanciaChebychev(child_forward.st.sonambulo, final);
			if (explored.find(child_forward.st)==explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				frontier.push(child_forward);
				actualizarObjetosJugador(child_forward, mapa);
			}

			//solo creo hijos del sonambulo si lo veo
			if (veoSonambuloN4(current_node.st.jugador,current_node.st.sonambulo)){
				// Generar hijo actFORWARD SONAMBULO
				nodeN3 child_forward_son = current_node; 
				nodos_generados++;
				child_forward_son.st = apply(actSON_FORWARD, current_node.st, mapa);
				child_forward_son.coste+=costeActualN3(actSON_FORWARD, current_node.st.sonambulo,mapa,child_forward_son.st);
				child_forward_son.heuristica=distanciaChebychev(child_forward_son.st.sonambulo, final);
			
				if (explored.find(child_forward_son.st)==explored.end())
				{
					child_forward_son.secuencia.push_back(actSON_FORWARD);
					frontier.push(child_forward_son);
					actualizarObjetosSonambulo(child_forward_son, mapa);
				}

				// Generar hijo actTURN_L SONAMBULO
				nodeN3 child_turnl_son = current_node; 
				nodos_generados++; 
				child_turnl_son.st = apply(actSON_TURN_SL, current_node.st, mapa);
				child_turnl_son.coste+=costeActualN3(actSON_TURN_SL, current_node.st.sonambulo,mapa,child_turnl_son.st);
				child_turnl_son.heuristica=distanciaChebychev(child_turnl_son.st.sonambulo, final);
				if (explored.find(child_turnl_son.st)==explored.end())
				{
					child_turnl_son.secuencia.push_back(actSON_TURN_SL);
					frontier.push(child_turnl_son);
					actualizarObjetosSonambulo(child_turnl_son, mapa);
				}
				// Generar hijo actTURN_R SONAMBULO
				nodeN3 child_turnr_son = current_node; 
				nodos_generados++;
				child_turnr_son.st = apply(actSON_TURN_SR, current_node.st, mapa);
				child_turnr_son.coste+=costeActualN3(actSON_TURN_SR, current_node.st.sonambulo,mapa,child_turnr_son.st);
				child_turnr_son.heuristica=distanciaChebychev(child_turnr_son.st.sonambulo, final);
				if (explored.find(child_turnr_son.st)==explored.end())
				{
					child_turnr_son.secuencia.push_back(actSON_TURN_SR);
					frontier.push(child_turnr_son);
					actualizarObjetosSonambulo(child_turnr_son, mapa);
				}
			}

			
			// Generar hijo actTURN_L JUGADOR
			nodeN3 child_turnl = current_node; 
			nodos_generados++;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			child_turnl.coste+=costeActualN3(actTURN_L, current_node.st.jugador,mapa,child_turnl.st);
			child_turnl.heuristica=distanciaChebychev(child_turnl.st.sonambulo, final); 
			if (explored.find(child_turnl.st)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push(child_turnl);
				actualizarObjetosJugador(child_turnl, mapa);
			}
			// Generar hijo actTURN_R JUGADOR
			nodeN3 child_turnr = current_node;
			nodos_generados++;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			child_turnr.coste+=costeActualN3(actTURN_R, current_node.st.jugador,mapa,child_turnr.st);
			child_turnr.heuristica=distanciaChebychev(child_turnr.st.sonambulo, final);
			
			if (explored.find(child_turnr.st)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push(child_turnr);
				actualizarObjetosJugador(child_turnr, mapa);
			}

			
		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.top();
			while(!frontier.empty() and explored.find(current_node.st)!=explored.end()){
				frontier.pop();
				if(!frontier.empty()){
					current_node=frontier.top();
					actualizarObjetosSonambulo(current_node, mapa);	
					actualizarObjetosJugador(current_node, mapa);
				}
			}
		}

		if (current_node.st.sonambulo.c==final.c && current_node.st.sonambulo.f==final.f){
			actualizarObjetosSonambulo(current_node, mapa);	
			actualizarObjetosJugador(current_node, mapa);
			SolutionFound = true;
		}
	}
	
	if (nodos_generados>=800000){
		cout<<"he creado demasiados nodos"<<endl;
		nodos_generados=0;
		return nivel4(inicio, final, plan, mapa);
	}
	cout<<"he creado: "<<nodos_generados<<endl;
	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;
}

/** funcion para comprobar si el sonambulo esta en el campo de vision del jugador */
bool veoSonambulo(const ubicacion &jugador, const ubicacion &sonambulo){
		
	bool encontrado=false;
	
	switch(jugador.brujula){
		case norte:
			if (abs(jugador.c-sonambulo.c)<=1 && (jugador.f-sonambulo.f)==1 )
				encontrado=true;
			else if (abs(jugador.c-sonambulo.c)<=2 && (jugador.f-sonambulo.f)==2 )
				encontrado=true;
			else if (abs(jugador.c-sonambulo.c)<=3 && (jugador.f-sonambulo.f)==3 )
				encontrado=true;
			break;
		case sur:
			if (abs(jugador.c-sonambulo.c)<=1 && (sonambulo.f-jugador.f)==1 )
				encontrado=true;
			else if (abs(jugador.c-sonambulo.c)<=2 && (sonambulo.f-jugador.f)==2 )
				encontrado=true;
			else if (abs(jugador.c-sonambulo.c)<=3 && (sonambulo.f-jugador.f)==3 )
				encontrado=true;
			break;
		case este:
			if (abs(jugador.f-sonambulo.f)<=1 && (sonambulo.c-jugador.c)==1 )
				encontrado=true;
			else if (abs(jugador.f-sonambulo.f)<=2 && (sonambulo.c-jugador.c)==2 )
				encontrado=true;
			else if (abs(jugador.f-sonambulo.f)<=3 && (sonambulo.c-jugador.c)==3 )
				encontrado=true;
			break;	
		case oeste:
			if (abs(jugador.f-sonambulo.f)<=1 && (jugador.c-sonambulo.c)==1 )
				encontrado=true;
			else if (abs(jugador.f-sonambulo.f)<=2 && (jugador.c-sonambulo.c)==2 )
				encontrado=true;
			else if (abs(jugador.f-sonambulo.f)<=3 && (jugador.c-sonambulo.c)==3 )
				encontrado=true;
			break;
	}

	return encontrado;
}
/** funcion para comprobar si el sonambulo esta en el campo de vision del jugador */
bool veoSonambuloN4(const ubicacion &jugador, const ubicacion &sonambulo){
		
	bool encontrado=false;
	
	switch(jugador.brujula){
		case norte:
			if (abs(jugador.c-sonambulo.c)<=1 && (jugador.f-sonambulo.f)==1 )
				encontrado=true;
			else if (abs(jugador.c-sonambulo.c)<=2 && (jugador.f-sonambulo.f)==2 )
				encontrado=true;
			//else if (abs(jugador.c-sonambulo.c)<=3 && (jugador.f-sonambulo.f)==3 )
			//	encontrado=true;
			break;
		case sur:
			if (abs(jugador.c-sonambulo.c)<=1 && (sonambulo.f-jugador.f)==1 )
				encontrado=true;
			else if (abs(jugador.c-sonambulo.c)<=2 && (sonambulo.f-jugador.f)==2 )
				encontrado=true;
			//else if (abs(jugador.c-sonambulo.c)<=3 && (sonambulo.f-jugador.f)==3 )
			//	encontrado=true;
			break;
		case este:
			if (abs(jugador.f-sonambulo.f)<=1 && (sonambulo.c-jugador.c)==1 )
				encontrado=true;
			else if (abs(jugador.f-sonambulo.f)<=2 && (sonambulo.c-jugador.c)==2 )
				encontrado=true;
			//else if (abs(jugador.f-sonambulo.f)<=3 && (sonambulo.c-jugador.c)==3 )
			//	encontrado=true;
			break;	
		case oeste:
			if (abs(jugador.f-sonambulo.f)<=1 && (jugador.c-sonambulo.c)==1 )
				encontrado=true;
			else if (abs(jugador.f-sonambulo.f)<=2 && (jugador.c-sonambulo.c)==2 )
				encontrado=true;
			//else if (abs(jugador.f-sonambulo.f)<=3 && (jugador.c-sonambulo.c)==3 )
			//	encontrado=true;
			break;
	}

	return encontrado;
}




/** funcion para calcular la distancia Manhattane entre dos casillas. Nos sirve como heuristica para el nivel 2*/
int distanciaManhattan(const ubicacion &inicio, const ubicacion &final){
	int distancia=abs(inicio.f-final.f)+abs(inicio.c-final.c);
	if (distancia<=0)
		distancia=0;
	return distancia;
}

/** funcion para calcular la distancia Chebychev entre dos casillas. Nos sirve como heuristica para el nivel 3*/
int distanciaChebychev(const ubicacion &inicio, const ubicacion &final){
	int distancia= max(abs(inicio.f- final.f), abs(inicio.c - final.c ));
	if (distancia<=0)
		distancia=0;
	return distancia;
}

/** funcion para calcular el coste uniforme*/
int costeActualN3(Action accion, const ubicacion &inicio, const vector<vector<unsigned char>> &mapa, const stateN3 &estado){
	int coste=0;
	
	if (accion==actIDLE)
		coste=0; 
	
	//ACTFORWARD JUGADOR
	if (accion==actFORWARD){
		if (mapa[inicio.f][inicio.c]=='A' && !estado.bikini)
			coste=100;
		else if(mapa[inicio.f][inicio.c]=='A' && estado.bikini)
			coste=10;
		else if(mapa[inicio.f][inicio.c]=='B' && !estado.zapatillas)
			coste=50;
		else if(mapa[inicio.f][inicio.c]=='B' && estado.zapatillas)
			coste=15;
		else if(mapa[inicio.f][inicio.c]=='T')
			coste=2; 
		else  //resto de casillas
			coste=1;
	}

	//ACTFORWARD SONMABULO
	if (accion==actSON_FORWARD){
		if (mapa[inicio.f][inicio.c]=='A' && !estado.bikini_son)
			coste=100;
		else if(mapa[inicio.f][inicio.c]=='A' && estado.bikini_son)
			coste=10;
		else if(mapa[inicio.f][inicio.c]=='B' && !estado.zapatillas_son)
			coste=50;
		else if(mapa[inicio.f][inicio.c]=='B' && estado.zapatillas_son)
			coste=15;
		else if(mapa[inicio.f][inicio.c]=='T')
			coste=2; 
		else  //resto de casillas
			coste=1;
	}

	//ACTTURNL/R JUGADOR
	if (accion==actTURN_L or accion==actTURN_R){
		if (mapa[inicio.f][inicio.c]=='A' && !estado.bikini)
			coste=25;
		else if(mapa[inicio.f][inicio.c]=='A' && estado.bikini)
			coste=5;
		else if(mapa[inicio.f][inicio.c]=='B' && !estado.zapatillas)
			coste=5;
		else if(mapa[inicio.f][inicio.c]=='B' && estado.zapatillas)
			coste=1;
		else if(mapa[inicio.f][inicio.c]=='T')
			coste=2;
		else //resto de casillas
			coste=1;
	}

	//actSON_TURN_SR/L SONAMBULO
	if (accion==actSON_TURN_SL or accion==actSON_TURN_SR){
		if (mapa[inicio.f][inicio.c]=='A' && !estado.bikini_son)
			coste=7;
		else if(mapa[inicio.f][inicio.c]=='A' && estado.bikini_son)
			coste=2;
		else if(mapa[inicio.f][inicio.c]=='B' && !estado.zapatillas_son)
			coste=3;
		else if(mapa[inicio.f][inicio.c]=='B' && estado.zapatillas_son)
			coste=1;
		else //resto de casillas
			coste=1;
	}

	return coste;
}

/** funcion para calcular el coste uniforme*/
int costeActualN2(Action accion, const ubicacion &inicio, const vector<vector<unsigned char>> &mapa, const stateN2 &estado){
	int coste=0;
	
	if (accion==actIDLE)
		coste=0; 
	
	//ACTFORWARD
	if (accion==actFORWARD){
		if (mapa[inicio.f][inicio.c]=='A' && !estado.bikini)
			coste=100;
		else if(mapa[inicio.f][inicio.c]=='A' && estado.bikini)
			coste=10;
		else if(mapa[inicio.f][inicio.c]=='B' && !estado.zapatillas)
			coste=50;
		else if(mapa[inicio.f][inicio.c]=='B' && estado.zapatillas)
			coste=15;
		else if(mapa[inicio.f][inicio.c]=='T')
			coste=2; 
		else  //resto de casillas
			coste=1;
	}

	//ACTTURNL/R JUGADOR
	if (accion==actTURN_L or accion==actTURN_R){
		if (mapa[inicio.f][inicio.c]=='A' && !estado.bikini)
			coste=25;
		else if(mapa[inicio.f][inicio.c]=='A' && estado.bikini)
			coste=5;
		else if(mapa[inicio.f][inicio.c]=='B' && !estado.zapatillas)
			coste=5;
		else if(mapa[inicio.f][inicio.c]=='B' && estado.zapatillas)
			coste=1;
		else if(mapa[inicio.f][inicio.c]=='T')
			coste=2;
		else //resto de casillas
			coste=1;
	}

	return coste;
}

/** funcion para actualizar el bikini y las zapas*/
void actualizarObjetos(nodeN2 &current_node, const vector<vector<unsigned char>> &mapa){

	if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='D' and current_node.st.bikini==false)
		current_node.st.zapatillas=true;
	else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='K' and current_node.st.zapatillas==false)
		current_node.st.bikini=true;
	else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='D' and current_node.st.bikini==true){
		current_node.st.zapatillas=true;
		current_node.st.bikini=false;
	}else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='K' and current_node.st.zapatillas==true){
		current_node.st.bikini=true;
		current_node.st.zapatillas=false;
	}

}

/** funcion para actualizar el bikini y las zapas*/
void actualizarObjetosJugador(nodeN3 &current_node, const vector<vector<unsigned char>> &mapa ){

	if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='D' and current_node.st.bikini==false)
		current_node.st.zapatillas=true;
	else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='K' and current_node.st.zapatillas==false)
		current_node.st.bikini=true;
	else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='D' and current_node.st.bikini==true){
		current_node.st.zapatillas=true;
		current_node.st.bikini=false;
	}else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c]=='K' and current_node.st.zapatillas==true){
		current_node.st.bikini=true;
		current_node.st.zapatillas=false;
	}

}

/** funcion para actualizar el bikini y las zapas*/
void actualizarObjetosSonambulo(nodeN3 &current_node, const vector<vector<unsigned char>> &mapa ){

	if (mapa[current_node.st.sonambulo.f][current_node.st.sonambulo.c]=='D' and current_node.st.bikini_son==false)
		current_node.st.zapatillas_son=true;
	else if (mapa[current_node.st.sonambulo.f][current_node.st.sonambulo.c]=='K' and current_node.st.zapatillas_son==false)
		current_node.st.bikini_son=true;
	else if (mapa[current_node.st.sonambulo.f][current_node.st.sonambulo.c]=='D' and current_node.st.bikini_son==true){
		current_node.st.zapatillas_son=true;
		current_node.st.bikini_son=false;
	}else if (mapa[current_node.st.sonambulo.f][current_node.st.sonambulo.c]=='K' and current_node.st.zapatillas_son==true){
		current_node.st.bikini_son=true;
		current_node.st.zapatillas_son=false;
	}

}


/** funcion para el nivel 4 */
bool costeUniforme(const stateN3 &inicio, const ubicacion &final, list<Action> & plan, const vector<vector<unsigned char>> &mapa){
	nodeN3 objetivo;
	objetivo.st.jugador=final;
	nodeN3 current_node;
	current_node.st = inicio;
	current_node.coste=0;	//inicializo el coste
	//current_node.heuristica=0;
	actualizarObjetosJugador(current_node, mapa);//por si empieza en bikini/zapas
	priority_queue<nodeN3, vector<nodeN3>, greater<nodeN3>> frontier;
	set<stateN3> explored;
	bool SolutionFound = (current_node.st.jugador.f == final.f and current_node.st.jugador.c == final.c);
	frontier.push(current_node);//meto en abiertos el nodo inicial (no miro coste pq es el unico que hay)

	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop();	//saca el de menor coste
		explored.insert(current_node.st);//lo meto en cerrados y ahora tengo que expandir:

		if (!SolutionFound)
		{
			// Generar hijo actFORWARD
			nodeN3 child_forward = current_node; 

			actualizarObjetosJugador(child_forward, mapa);
			child_forward.st = applyN4(actFORWARD, current_node.st, mapa);
			child_forward.coste+=costeActualN2(actFORWARD, current_node.st.jugador,mapa,child_forward.st);
			if (explored.find(child_forward.st)==explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				actualizarObjetosJugador(child_forward, mapa);
				frontier.push(child_forward);
			}

			// Generar hijo actTURN_R
			nodeN3 child_turnr = current_node; 
			actualizarObjetosJugador(child_turnr, mapa);
			child_turnr.st = applyN4(actTURN_R, current_node.st, mapa);
			child_turnr.coste+=costeActualN2(actTURN_R, current_node.st.jugador,mapa,child_turnr.st);
			

			if (explored.find(child_turnr.st)==explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				actualizarObjetosJugador(child_turnr, mapa);
				frontier.push(child_turnr);
			}

			// Generar hijo actTURN_L
			nodeN3 child_turnl = current_node;  
			actualizarObjetosJugador(child_turnl, mapa);
			child_turnl.st = applyN4(actTURN_L, current_node.st, mapa);
			child_turnl.coste+=costeActualN2(actTURN_L, current_node.st.jugador,mapa,child_turnl.st);

			if (explored.find(child_turnl.st)==explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				actualizarObjetosJugador(child_turnl, mapa);
				frontier.push(child_turnl);
			}	

		}

		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.top();
			while(!frontier.empty() and explored.find(current_node.st)!=explored.end()){
				frontier.pop();
				if(!frontier.empty()){
					current_node=frontier.top();
					actualizarObjetosJugador(current_node, mapa);
				}
			}
		}

		if (current_node.st.jugador.c==final.c && current_node.st.jugador.f==final.f){
			actualizarObjetosJugador(current_node, mapa);	
			SolutionFound = true;
		}
	}

	if(SolutionFound){  
		plan = current_node.secuencia;
	}

	return SolutionFound;

}


void PonerTerrenoEnMatriz (const stateN3 &st, vector<vector<unsigned char>>&matriz, Sensores sensores){
    //Actualiza el mapa con la casilla donde se encuentre el agente
    //matriz[st.jugador.c][st.jugador.f] = sensores.terreno[0];
    if ((st.jugador.brujula==norte) or (st.jugador.brujula==sur) or (st.jugador.brujula==este) or (st.jugador.brujula==oeste))
      if (st.jugador.brujula==sur){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f+1][st.jugador.c+1] = sensores.terreno[1];
        matriz[st.jugador.f+1][st.jugador.c] = sensores.terreno[2];
        matriz[st.jugador.f+1][st.jugador.c-1] = sensores.terreno[3];

        matriz[st.jugador.f+2][st.jugador.c+2] = sensores.terreno[4];
        matriz[st.jugador.f+2][st.jugador.c+1] = sensores.terreno[5];
        matriz[st.jugador.f+2][st.jugador.c] = sensores.terreno[6];
        matriz[st.jugador.f+2][st.jugador.c-1] = sensores.terreno[7];
        matriz[st.jugador.f+2][st.jugador.c-2] = sensores.terreno[8];
		
        matriz[st.jugador.f+3][st.jugador.c+3] = sensores.terreno[9];
        matriz[st.jugador.f+3][st.jugador.c+2] = sensores.terreno[10];
        matriz[st.jugador.f+3][st.jugador.c+1] = sensores.terreno[11];
        matriz[st.jugador.f+3][st.jugador.c] = sensores.terreno[12];
        matriz[st.jugador.f+3][st.jugador.c-1] = sensores.terreno[13];
        matriz[st.jugador.f+3][st.jugador.c-2] = sensores.terreno[14];
        matriz[st.jugador.f+3][st.jugador.c-3] = sensores.terreno[15];  
      }

      if (st.jugador.brujula==norte){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f-1][st.jugador.c-1] = sensores.terreno[1];
        matriz[st.jugador.f-1][st.jugador.c] = sensores.terreno[2];
        matriz[st.jugador.f-1][st.jugador.c+1] = sensores.terreno[3];

        matriz[st.jugador.f-2][st.jugador.c-2] = sensores.terreno[4];
        matriz[st.jugador.f-2][st.jugador.c-1] = sensores.terreno[5];
        matriz[st.jugador.f-2][st.jugador.c] = sensores.terreno[6];
        matriz[st.jugador.f-2][st.jugador.c+1] = sensores.terreno[7];
        matriz[st.jugador.f-2][st.jugador.c+2] = sensores.terreno[8];

        matriz[st.jugador.f-3][st.jugador.c-3] = sensores.terreno[9];
        matriz[st.jugador.f-3][st.jugador.c-2] = sensores.terreno[10];
        matriz[st.jugador.f-3][st.jugador.c-1] = sensores.terreno[11];
        matriz[st.jugador.f-3][st.jugador.c] = sensores.terreno[12];
        matriz[st.jugador.f-3][st.jugador.c+1] = sensores.terreno[13];
        matriz[st.jugador.f-3][st.jugador.c+2] = sensores.terreno[14];
        matriz[st.jugador.f-3][st.jugador.c+3] = sensores.terreno[15];
      }

      if (st.jugador.brujula==este){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f-1][st.jugador.c+1] = sensores.terreno[1];
        matriz[st.jugador.f][st.jugador.c+1] = sensores.terreno[2];
        matriz[st.jugador.f+1][st.jugador.c+1] = sensores.terreno[3];

        matriz[st.jugador.f-2][st.jugador.c+2] = sensores.terreno[4];
        matriz[st.jugador.f-1][st.jugador.c+2] = sensores.terreno[5];
        matriz[st.jugador.f][st.jugador.c+2] = sensores.terreno[6];
        matriz[st.jugador.f+1][st.jugador.c+2] = sensores.terreno[7];
        matriz[st.jugador.f+2][st.jugador.c+2] = sensores.terreno[8];

        matriz[st.jugador.f-3][st.jugador.c+3] = sensores.terreno[9];
        matriz[st.jugador.f-2][st.jugador.c+3] = sensores.terreno[10];
        matriz[st.jugador.f-1][st.jugador.c+3] = sensores.terreno[11];
        matriz[st.jugador.f][st.jugador.c+3] = sensores.terreno[12];
        matriz[st.jugador.f+1][st.jugador.c+3] = sensores.terreno[13];
        matriz[st.jugador.f+2][st.jugador.c+3] = sensores.terreno[14];
        matriz[st.jugador.f+3][st.jugador.c+3] = sensores.terreno[15];
      }

      if (st.jugador.brujula==oeste){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f+1][st.jugador.c-1] = sensores.terreno[1];
        matriz[st.jugador.f][st.jugador.c-1] = sensores.terreno[2];
        matriz[st.jugador.f-1][st.jugador.c-1] = sensores.terreno[3];

        matriz[st.jugador.f+2][st.jugador.c-2] = sensores.terreno[4];
        matriz[st.jugador.f+1][st.jugador.c-2] = sensores.terreno[5];
        matriz[st.jugador.f][st.jugador.c-2] = sensores.terreno[6];
        matriz[st.jugador.f-1][st.jugador.c-2] = sensores.terreno[7];
        matriz[st.jugador.f-2][st.jugador.c-2] = sensores.terreno[8];

        matriz[st.jugador.f+3][st.jugador.c-3] = sensores.terreno[9];
        matriz[st.jugador.f+2][st.jugador.c-3] = sensores.terreno[10];
        matriz[st.jugador.f+1][st.jugador.c-3] = sensores.terreno[11];
        matriz[st.jugador.f][st.jugador.c-3] = sensores.terreno[12];
        matriz[st.jugador.f-1][st.jugador.c-3] = sensores.terreno[13];
        matriz[st.jugador.f-2][st.jugador.c-3] = sensores.terreno[14];
        matriz[st.jugador.f-3][st.jugador.c-3] = sensores.terreno[15];
      }

      if (st.jugador.brujula==noreste){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f-1][st.jugador.c] = sensores.terreno[1];
        matriz[st.jugador.f-1][st.jugador.c+1] = sensores.terreno[2];
        matriz[st.jugador.f][st.jugador.c+1] = sensores.terreno[3];

        matriz[st.jugador.f-2][st.jugador.c] = sensores.terreno[4];
        matriz[st.jugador.f-2][st.jugador.c+1] = sensores.terreno[5];
        matriz[st.jugador.f-2][st.jugador.c+2] = sensores.terreno[6];
        matriz[st.jugador.f-1][st.jugador.c+2] = sensores.terreno[7];
        matriz[st.jugador.f][st.jugador.c+2] = sensores.terreno[8];

        matriz[st.jugador.f-3][st.jugador.c] = sensores.terreno[9];
        matriz[st.jugador.f-3][st.jugador.c+1] = sensores.terreno[10];
        matriz[st.jugador.f-3][st.jugador.c+2] = sensores.terreno[11];
        matriz[st.jugador.f-3][st.jugador.c+3] = sensores.terreno[12];
        matriz[st.jugador.f-2][st.jugador.c+3] = sensores.terreno[13];
        matriz[st.jugador.f-1][st.jugador.c+3] = sensores.terreno[14];
        matriz[st.jugador.f][st.jugador.c+3] = sensores.terreno[15];
      }

      if (st.jugador.brujula==noroeste){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f][st.jugador.c-1] = sensores.terreno[1];
        matriz[st.jugador.f-1][st.jugador.c-1] = sensores.terreno[2];
        matriz[st.jugador.f-1][st.jugador.c] = sensores.terreno[3];

        matriz[st.jugador.f][st.jugador.c-2] = sensores.terreno[4];
        matriz[st.jugador.f-1][st.jugador.c-2] = sensores.terreno[5];
        matriz[st.jugador.f-2][st.jugador.c-2] = sensores.terreno[6];
        matriz[st.jugador.f-2][st.jugador.c-1] = sensores.terreno[7];
        matriz[st.jugador.f-2][st.jugador.c] = sensores.terreno[8];

        matriz[st.jugador.f][st.jugador.c-3] = sensores.terreno[9];
        matriz[st.jugador.f-1][st.jugador.c-3] = sensores.terreno[10];
        matriz[st.jugador.f-2][st.jugador.c-3] = sensores.terreno[11];
        matriz[st.jugador.f-3][st.jugador.c-3] = sensores.terreno[12];
        matriz[st.jugador.f-3][st.jugador.c-2] = sensores.terreno[13];
        matriz[st.jugador.f-3][st.jugador.c-1] = sensores.terreno[14];
        matriz[st.jugador.f-3][st.jugador.c] = sensores.terreno[15];
      }

      if (st.jugador.brujula==suroeste){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f+1][st.jugador.c] = sensores.terreno[1];
        matriz[st.jugador.f+1][st.jugador.c-1] = sensores.terreno[2];
        matriz[st.jugador.f][st.jugador.c-1] = sensores.terreno[3];

        matriz[st.jugador.f+2][st.jugador.c] = sensores.terreno[4];
        matriz[st.jugador.f+2][st.jugador.c-1] = sensores.terreno[5];
        matriz[st.jugador.f+2][st.jugador.c-2] = sensores.terreno[6];
        matriz[st.jugador.f+1][st.jugador.c-2] = sensores.terreno[7];
        matriz[st.jugador.f][st.jugador.c-2] = sensores.terreno[8];

        matriz[st.jugador.f+3][st.jugador.c] = sensores.terreno[9];
        matriz[st.jugador.f+3][st.jugador.c-1] = sensores.terreno[10];
        matriz[st.jugador.f+3][st.jugador.c-2] = sensores.terreno[11];
        matriz[st.jugador.f+3][st.jugador.c-3] = sensores.terreno[12];
        matriz[st.jugador.f+2][st.jugador.c-3] = sensores.terreno[13];
        matriz[st.jugador.f+1][st.jugador.c-3] = sensores.terreno[14];
        matriz[st.jugador.f][st.jugador.c-3] = sensores.terreno[15];
      }

      if (st.jugador.brujula==sureste){
        matriz[st.jugador.f][st.jugador.c] = sensores.terreno[0];

        matriz[st.jugador.f][st.jugador.c+1] = sensores.terreno[1];
        matriz[st.jugador.f+1][st.jugador.c+1] = sensores.terreno[2];
        matriz[st.jugador.f+1][st.jugador.c] = sensores.terreno[3];

        matriz[st.jugador.f][st.jugador.c+2] = sensores.terreno[4];
        matriz[st.jugador.f+1][st.jugador.c+2] = sensores.terreno[5];
        matriz[st.jugador.f+2][st.jugador.c+2] = sensores.terreno[6];
        matriz[st.jugador.f+2][st.jugador.c+1] = sensores.terreno[7];
        matriz[st.jugador.f+2][st.jugador.c] = sensores.terreno[8];

        matriz[st.jugador.f][st.jugador.c+3] = sensores.terreno[9];
        matriz[st.jugador.f+1][st.jugador.c+3] = sensores.terreno[10];
        matriz[st.jugador.f+2][st.jugador.c+3] = sensores.terreno[11];
        matriz[st.jugador.f+3][st.jugador.c+3] = sensores.terreno[12];
        matriz[st.jugador.f+3][st.jugador.c+2] = sensores.terreno[13];
        matriz[st.jugador.f+3][st.jugador.c+1] = sensores.terreno[14];
        matriz[st.jugador.f+3][st.jugador.c] = sensores.terreno[15];
      }
    
  }

// Función que busca una letra  en una matriz de unsigned char y devuelve la casilla donde se encuentra
ubicacion buscarEnMatriz (vector<vector<unsigned char>>&matriz, unsigned char letra, const ubicacion &jugador){
	int maxsize = matriz.size();

	//almaceno toda las recargas en un vector y voy a la que mas cerca esté
	vector<ubicacion> recargas;

	// Recorremos cada casilla del vector de vectores
	for (int i = 0; i < maxsize; i++) {
		for (int j = 0; j < maxsize; j++) {
			if (matriz[i][j] == letra) {
				ubicacion recarga;
				recarga.f=i;
				recarga.c=j;
				//recarga.brujula=0;//puede ser cualquiera
				recargas.push_back(recarga);
				//return make_pair(i, j);
			}
		}
	}
	int distancia=1000;
	ubicacion recarga_mas_cercana;
	if (recargas.size()>0){
		recarga_mas_cercana=recargas[0];
		//comprobamos con la distancia manhhatan cual esta mas cerca del jugador
		for (int i = 0; i < recargas.size(); i++) {
			int nueva_distancia=distanciaManhattan(recargas[i],jugador);
			if (nueva_distancia<distancia){
				distancia=distanciaManhattan(recargas[i],jugador);
				recarga_mas_cercana=recargas[i];
			}
		}
	}
	//else recarga_mas_cercana=jugador;
	if (recarga_mas_cercana.c>0 and recarga_mas_cercana.f>0){
		return recarga_mas_cercana;// Si no se encontró la letra
	}else{
		recarga_mas_cercana.f=-1;
		recarga_mas_cercana.c=-1;
		return recarga_mas_cercana;
	}
    
}




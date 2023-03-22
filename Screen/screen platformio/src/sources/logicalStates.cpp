#include "LCDWIKI_KBV.h"
#include "logicalStates.hpp"

int menuActual = 1;
int menuDevuelto = 0;
boolean campo = true;
int spawn = 1;
int plan = 1;
constexpr int numeroPlanes = 5;
boolean lidar = true;

namespace logicalStates {
    int getMenuActual() {return menuActual;}
    int getMenuDevuelto() {return menuDevuelto;}
    boolean getCampo() {return campo;}
    int getSpawn() {return spawn;}
    int getPlan() {return plan;}
    boolean getLidar() {return lidar;}
    int setMenuActual(int m) {menuActual = m;}

    int setMenuDevuelto(int m) {
        menuDevuelto = m;
        return m;
    }

    void setCampo(boolean b) {campo = b;}

    //Al conmutar el campo
    void setSpawnB2() {
        if(spawn != 10)
          spawn++;
        else {
          if(campo)
            spawn = 1;
          else
            spawn = 2;
        }
    }

    //Al conmutar el spawn
    void setSpawnB3() {
        if(spawn < 9)
          spawn += 2;
        else {
          if(campo)
            spawn = 1;
          else
            spawn = 2;
        }
    }

    void setPlan() {
        if(plan < numeroPlanes)
          plan++;
        else
          plan = 1;
    }

    void setLidar(boolean b) {lidar = b;}
}

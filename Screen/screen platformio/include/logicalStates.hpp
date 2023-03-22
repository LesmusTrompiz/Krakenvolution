#ifndef LOGICALSTATES_HPP
#define LOGICALSTATES_HPP

namespace logicalStates {
    int getMenuActual();
    int getMenuDevuelto();
    boolean getCampo();
    int getSpawn();
    int getPlan();
    boolean getLidar();
    int setMenuActual(int m);
    int setMenuDevuelto(int m);
    void setCampo(boolean b);
    //Al conmutar el campo
    void setSpawnB2();
    //Al conmutar el spawn
    void setSpawnB3();
    void setPlan();
    void setLidar(boolean b);
}

#endif

#pragma once

#include "motor_driver.hpp"
#include "encoder_driver.hpp"

struct Pose
{
    float x;
    float y;
    float alfa;

    Pose() : x{0}, y{0}, alfa{0} {};
    Pose(float _x, float _y, float _alfa) : x{_x}, y{_y}, alfa{_alfa} {};
};


struct Consigna
{
    float v;        // Velocidad lineal
    float w;        // Velocidad angular
    Consigna() : v{0}, w{0} {};
    Consigna(float _v, float _w) : v{_v}, w{_w}{};
};

struct VRuedas
{
    float r;        
    float l;        
    VRuedas() : r{0}, l{0} {};
    VRuedas(float _r, float _l) : r{_r}, l{_l}{};
};



class PoseController{

    public:

        PoseController(
            MotorDriver     &rmotor_,
            MotorDriver     &lmotor_,
            EncoderDriver   &rencoder_,
            EncoderDriver   &lencoder_,
            uint16_t         L_,
            uint16_t         R_);
        // Funciones interfaz
        void giro(int degrees);
        void recta(int mm);

        // Funciones internas:
        void ley_de_control();
        void ley_de_control(const int vd, const int wd);
        void consigna_to_velocidad();
        void update_motor_speed();
        void cuentas_to_odom();
        void reset_counts();
        void reset_controller();
        bool check_stop();
        bool in_goal();


    private:
        MotorDriver     *rmotor;
        MotorDriver     *lmotor;
        EncoderDriver   *rencoder;
        EncoderDriver   *lencoder;
        uint16_t         L;                                     // longitud_eje
        uint16_t         R;                                     // radio_rueda
        Pose             robot_pose;
        Pose             ref_pose;
        Consigna         cons;
        VRuedas          v_ruedas;
};

#include <eurouart.hpp>

/* Tracción */
extern motion_controller controlador_tactico;

/* Servos */
extern RobotServo servo_disparador;

uahruart::parser::Protocol protocol;

void setup_serial() {
    protocol.on_write([](const char* msg) {
        Serial.println(msg);
    });

    // Register methods
    protocol.register_method("traction", "turn", [](int32_t arg) {
        controlador_tactico.ref_ang = static_cast<float>(arg);
        controlador_tactico.prev_move_calculus(0);
    });

    protocol.register_method("traction", "advance", [](int32_t arg) {
        controlador_tactico.ref_distancia = static_cast<float>(arg);
        controlador_tactico.prev_move_calculus(1);
    });

    protocol.register_method("admin", "reset", [](int32_t arg) {
        rstc_start_software_reset(RSTC);
    });

    protocol.register_method("actuadores", "servo_disparador", [](int32_t arg) {
        servo_disparador.set_angle(arg);
    });

    on_finished([]() {
        uahruart::primitives::Bool test = true;
        protocol.send(test);
    });
}

void serialEvent()
{
    static string input_string;
    while(Serial.available()>0)
    {
        input_string = string(Serial.readStringUntil('\n').c_str());

        if(input_string != "")
        {
            protocol.receive(input_string);
            return;
        }
  }
}


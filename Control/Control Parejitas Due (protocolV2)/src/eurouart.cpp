#include <eurouart.hpp>

extern uahruart::parser::Protocol protocol;

void serialEvent()
{
    while(Serial.available()>0)
    {
        auto input_string = string(Serial.readStringUntil('\n').c_str());

        if(input_string != "")
        {
            protocol.receive(input_string);
            return;
        }
  }
}

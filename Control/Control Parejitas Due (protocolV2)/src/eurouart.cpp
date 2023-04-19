#include <eurouart.hpp>

extern uahruart::parser::Protocol protocol;

void serialEvent()
{
    static string input_string;
    while(Serial.available()>0)
    {
        input_string = string(Serial.readStringUntil('\0').c_str());

        if(input_string != "")
        {
            protocol.receive(input_string);
            return;
        }
  }
}
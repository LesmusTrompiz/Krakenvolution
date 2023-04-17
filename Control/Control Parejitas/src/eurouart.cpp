#include <eurouart.hpp>

extern uahruart::parser::Protocol protocol;

void serialEvent()
{
    static string input_string;
    while(SerialUSB.available()>0)
    {
        input_string = string(SerialUSB.readStringUntil('\n').c_str());

        if(input_string != "")
        {
            protocol.receive(input_string);
            return;
        }
  }
}


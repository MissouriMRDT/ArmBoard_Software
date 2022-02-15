#include "RovesODrive.h"

RovesODrive Wrist(&Serial5);
RovesODrive Elbow(&Serial3);
String command;

void setup()
{
    Serial.begin(115200);
    Wrist.begin();
    Elbow.begin();
    delay(1000);
}

void loop()
{
    if(Serial5.available())
    {
        Serial.print("--->");
        while(Serial5.available())
        {
            Serial.write(Serial5.read());
        }
    }

    if(Serial3.available())
    {
        Serial.print("--->");
        while(Serial3.available())
        {
            Serial.write(Serial3.read());
        }
    }
    
    
    while (Serial.available()) 
    {
        char c = Serial.read();  
        command += c; 
        delay(2);  
    }

    if (command.length() >0) 
    {
        Serial.println(command);

        if (command == "reboot")
        {
            Serial.println("Rebooting");
            Wrist.left.reboot();
            Elbow.left.reboot();
            Serial.println("Done");
        }
        else if(command == "closed")
        {
            Wrist.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            Wrist.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            Elbow.left.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            Elbow.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);

        }
        else if(command == "v")
        {
            Serial.println("Enter target Velocity for  motor");
            command = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                command += c; 
                delay(2);  
            }
            int vel = command.toInt();

            Serial.println("Setting Velocity Set Point...");

            Wrist.left.writeVelocitySetpoint(vel, 0);
            Wrist.right.writeVelocitySetpoint(vel, 0);
            Elbow.left.writeVelocitySetpoint(vel, 0);
            Elbow.right.writeVelocitySetpoint(vel, 0);


            Serial.println("Done");
        }
        command = "";
    }
}
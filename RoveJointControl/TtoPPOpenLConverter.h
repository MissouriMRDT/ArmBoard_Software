/*
 * TorquePowerPercentConverter.h
 *
 *  Created on: Oct 6, 2017
 *      Author: drue
 */

#ifndef ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_
#define ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_

#include "AbstractFramework.h"

//The various types of motors that this
//class supports
typedef enum TorqueConverterMotorTypes
{
  TorqueConvert_BrushedDC
} TorqueConverterMotorTypes;

//Class for converting torque into power percent.
//Open loop; it relies on mathematical equations to do its conversion
//see the readme.md for more info
class TtoPPOpenLConverter: public IOConverter
{
  private:

    //the type of motor to do the torque conversion for
    TorqueConverterMotorTypes motorType;

    //motor's torque constant, if applicable. Newton-meter per Amp.
    const float KT;

    //Voltage sensor for converting voltage to PWM, if used.
    FeedbackDevice* const VoltSensor;

    //Whether a voltage sensor is used (IE don't assume a static power voltage)
    //or not (IE assume the motor has a static power voltage)
    const bool voltConverterUsed;

    //The constant voltage used to power the motor (if used), in millivolts.
    const unsigned int staticMilliVolts;

    //The internal resistance of the motor, in milliohms
    const int motorR_mOhm;

    //Overview: Converts a given amount of millinewton-meters into a power percent value, for
    //          a brushed DC motor. Feeding this value to the motor will allow it to output up to roughly the given torque.
    //
    //Input:    The amount of torque to convert in milliNewton-meters
    //
    //returns:  A power percentage rating to give to the motor, in order to allow it to output up to roughly the given torque.
    long runAlgorithmBrushedDC(const long torque_milliNewtonsMeters);

    //Overview: Runs this class's algorithm, which is to convert the input in milli-newton-meters into a power
    //          percent rating. Feeding this value to the motor will allow it to output up to roughly the given torque.
    //
    //Inputs:   input: the amount of milli-newton-meters to convert
    //          ret_OutputFinished: A pass-by-pointer return value signifying if the output has finished or if
    //                              the class needs to be called again. Since this is an open loop algorithm,
    //                              it's always set to true.
    //
    //returns:  A power percentage rating to give to the motor, in order to allow it to output up to roughly the given torque.
    //
    //notes:    Overrides DrivingAlgorithm's runAlgorithm virtual function.
    long runAlgorithm(const long input, bool * ret_OutputFinished);

    //internal version of runAlgorithm that's designed to service both runAlgorithm(long, bool*) and addToOutput
    long runAlgorithm(const long input, const long oldOutput, bool * ret_OutputFinished);

    //function to be called when class is acting as a support algorithm to another IOConverter.
    long addToOutput(const long inputValue, const long calculatedOutput);

  public:

    //overview: Constructor. This version will tell the converter to use a voltage sensor for its calculations, rather
    //          than simply assuming what the motor power line's voltage is.
    //
    //Inputs:   motor_type: What type of motor is attached to this joint, so the class knows which equation to use for the conversion.
    //          Kt: The motor's torque constant, in Newton-meters
    //          motResistance_milliOhms: The static resistance found at the motor's terminals, in milliohms.
    //          voltSensor: A voltage sensing Feedback Device class, set up to sense the voltage of the motor power line.
    //
    //Warning:  The constructor will hang the program in an infinite fault loop for debugging purposes if it detects that
    //          the passed feedback device doesn't return voltage values.
    TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, FeedbackDevice *voltSensor);

    //overview: Constructor. This version will tell the converter to assume a static amount of voltage for the motor's
    //          power line.
    //
    //Inputs:   motor_type: What type of motor is attached to this joint, so the class knows which equation to use for the conversion.
    //          Kt: The motor's torque constant, in Newton-meters
    //          motResistance_milliOhms: The static resistance found at the motor's terminals, in milliohms.
    //          staticMillivolts: The static amount of voltage on the motor's power line.
    //
    //Warning:  The constructor will hang the program in an infinite fault loop for debugging purposes if it detects that
    //          the passed value for static millivolts is greater than VOLT_MAX or less than VOLT_MIN
    TtoPPOpenLConverter(TorqueConverterMotorTypes motor_type, float Kt, int motResistance_milliOhms, int staticMillivolts);

};




#endif /* ROVEJOINTCONTROL_TORQUEPOWERPERCENTCONVERTER_H_ */

#include "Energia.h" 
/*  
 *   
 *   This is the library for the controllers (either basic H Bridge or full fledged controller) which will be used for the 2017 Mars Rover Design Team
 *   It is subject to change. Does not include support for IK yet.    
 *   
 *   The interface class is friend classes to both the controllerType class and algorithm class. The interface is called to initialize the motor instance.
 *   To create a motor instance you must pass the interface the expected input, controller type, and the pins for that controller.
 *   Once that is done you can call the move command from the interface(since it is a friend of the controller) without worry (hopefully)
 *   
*/  

  //All the types of input that can be expected and selected
  //only spd is implemented for now as it covers Open loop
  enum input{spd, pos};
  
  //types schemes that will move the arm. 
  //only no Feedback works for now
  enum scheme{noFeed, Feed, IK, haptic};
  
  //used later for maping in move commands. constants so they can be easily changed if something changes on input or expected output.
  const int PWM_MIN = 0, PWM_MAX = 255;
  const int SPEED_MIN = -1000, SPEED_MAX = 1000;
  
  //Place holder for actual position values
  const int POS_MIN = 0, POS_MAX = 10000;
 
  //controller class for when something is used(custom H bridge or H Bridge IC)
  class controllerType{
    friend class interface;
    protected:
      //all the controllers have different ways of movement controlled by algorithms. The algorithm will spit out the speed or position that is actually usable by the controller (pwm or whatever) which will always be an int.
      //The pins are taken care of directly by the controller instance. The polarity by the sign of the integer for the move command. cant do much about it.
      virtual void moveMotor(const int movement);
    public:
      //the types of controllers which are possible add or remove as needed
      enum contTypeID {directDiscreteHBridge};  
      contTypeID ID;
  };

  //custom H Bridge which has only two inputs to control forward and backwards.
  class directDiscreteHBridge : public controllerType{
    private:
      //same as for drv8829 FPWM is the forward PWM and RPWM is the reverse PWM
      int FPWM_PIN, RPWM_PIN;
    public:
      //constructor, user must give pin assignments.
      directDiscreteHBridge(const int FPWM, const int RPWM);  
      void moveMotor(const int movement);
  };

  class inputAlgorithm{
    friend class interface;
    protected:
      virtual int modify(const int in);
  };

  class spdToSpdNoFeedAlgorithm : public inputAlgorithm{
    protected:
      int modify(const int in);
  };
  
  //What is seen by the outside program. It contains a pointer to a controller(which is passed to it on creation with the pins needed for the controller) and the type of input that will be sent(speed or position).
  //Selects the proper algorithm from these two points of information.
  class interface{
    protected:
      scheme method;
      input I;
      controllerType* controller;
      inputAlgorithm* manip;
    public:
      interface(input in, scheme meth, controllerType * cont)
      {
        I = in;
        method = meth;
        controller = cont;
      
        if((controller->ID == controllerType::directDiscreteHBridge) && in == spd && method == noFeed)
        {
          manip = new spdToSpdNoFeedAlgorithm();
        }
      }
      
      ~interface();       
      void mov(const int movement); 
  };
 
  
/* Not implemented Yet
  class spdToSpdFeedAlgorithm : public inputAlgorithm{
    protected:
      int modify(const int in);
  };

  class spdToPosAlgorithm : public inputAlgorithm{
    protected: 
      int modify(const int in);
  };

  class posToPosAlgorithm : public inputAlgorithm{
    protected:
      int modify(const int in);
  };
 */

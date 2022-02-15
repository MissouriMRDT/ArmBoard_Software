/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveBoardTivaC.h"

////////////////////////////////////////
RoveRingBuff < int, 10 > RingBuff;

/////////////////////
void setup() 
{
  Serial.begin(9600);
}

//////////////////////////////////////////////////////////////////////////////
void loop() 
{
  Serial.println("RoveRingBuff sweep");

  for(int i=1; i <= 40; i++ )
  {
    RingBuff.pushToBack( i );
    Serial.println();  
    Serial.println("////////////////////////////////////");
    Serial.print("Push Back: ");  Serial.println( i                        );
    Serial.print("Back:      ");  Serial.println( RingBuff.peekFromBack()  );
    Serial.print("Front:     ");  Serial.println( RingBuff.peekFromFront() );
    Serial.print("Head:      ");  Serial.println( RingBuff.head            );
    Serial.print("Tail:      ");  Serial.println( RingBuff.tail            );
    Serial.print("Size:      ");  Serial.println( RingBuff.size()          );
    Serial.print("Count:     ");  Serial.println( RingBuff.cnt             );
    Serial.print("Empty:     ");  Serial.println( RingBuff.isEmpty()       );
    Serial.print("Full:      ");  Serial.println( RingBuff.isFull()        );
    Serial.print("Available: ");  Serial.println( RingBuff.isAvailable()   );
    Serial.print("Sum:       ");  Serial.println( RingBuff.sum()           );
    Serial.print("Average:   ");  Serial.println( RingBuff.average()       );
    delay(250);
  }

  for(int i=1; i > -40; i-- )
  {
    RingBuff.pushToBack( i );
    Serial.println();  
    Serial.println("////////////////////////////////////");
    Serial.print("Push Back: ");  Serial.println( i                        );
    Serial.print("Back:      ");  Serial.println( RingBuff.peekFromBack()  );
    Serial.print("Front:     ");  Serial.println( RingBuff.peekFromFront() );
    Serial.print("Head:      ");  Serial.println( RingBuff.head            );
    Serial.print("Tail:      ");  Serial.println( RingBuff.tail            );
    Serial.print("Size:      ");  Serial.println( RingBuff.size()          );
    Serial.print("Count:     ");  Serial.println( RingBuff.cnt             );
    Serial.print("Empty:     ");  Serial.println( RingBuff.isEmpty()       );
    Serial.print("Full:      ");  Serial.println( RingBuff.isFull()        );
    Serial.print("Available: ");  Serial.println( RingBuff.isAvailable()   );
    Serial.print("Sum:       ");  Serial.println( RingBuff.sum()           );
    Serial.print("Average:   ");  Serial.println( RingBuff.average()       );
    delay(250);
  }

  delay(500);

  ///////////////////////////////////////////////////////////////////////////
  for(int i=1; i <= 40; i++ )
  {
    Serial.println();  
    Serial.println("////////////////////////////////////");
    Serial.print("Pop Front: ");  Serial.println( RingBuff.popFromFront()  );
    Serial.print("Back:      ");  Serial.println( RingBuff.peekFromBack()  );
    Serial.print("Front:     ");  Serial.println( RingBuff.peekFromFront() );
    Serial.print("Head:      ");  Serial.println( RingBuff.head            );
    Serial.print("Tail:      ");  Serial.println( RingBuff.tail            );
    Serial.print("Size:      ");  Serial.println( RingBuff.size()          );
    Serial.print("Count:     ");  Serial.println( RingBuff.cnt             );
    Serial.print("Empty:     ");  Serial.println( RingBuff.isEmpty()       );
    Serial.print("Full:      ");  Serial.println( RingBuff.isFull()        );
    Serial.print("Available: ");  Serial.println( RingBuff.isAvailable()   );
    Serial.print("Sum:       ");  Serial.println( RingBuff.sum()           );
    Serial.print("Average:   ");  Serial.println( RingBuff.average()       );
    delay(500);
  }

  delay(500);
}
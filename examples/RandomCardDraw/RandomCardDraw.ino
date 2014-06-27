//    FILE: RandomCardDraw.ino
//  AUTHOR: Rob Tillaart and Walter Anderson
// VERSION: 1.0.0
// PURPOSE: generate random sequence (optimized)
//    DATE: April 24, 2014
//     URL: 

// The Entropy library provides true random numbers and can be obtained from:
// http://code.google.com/p/avr-hardware-random-number-generation/wiki/WikiAVRentropy
#include <Entropy.h>

const byte MAX_VALUE=10;
int RandomDraw[MAX_VALUE+1];
byte tmp;

// The following function swaps the contents of an array at the locations a & b
void swap(int a, int b)
{
  int t = RandomDraw[a];
  RandomDraw[a] = RandomDraw[b];
  RandomDraw[b] = t;
}

void setup() {
  
  Serial.begin(9600);
  
  // Initialize array with the sequence of values we will randomly draw from
  for (int i=0; i<=MAX_VALUE; i++) {
    RandomDraw[i] = i;
  }
  
  // Initialize the Entropy library to ensure that the draws are truely random.
  Entropy.initialize();
  
  Serial.print("Starting to draw random sequences from 1 to ");
  Serial.println(MAX_VALUE);
}

void loop()
{
  // fetch MAX_VALUE random numbers from the sequence 1 .. MAX_VALUE with no repeats and no missed values
  for (int i=1; i<=MAX_VALUE; ++i)
  {
    tmp = Entropy.random(1, (MAX_VALUE+1)-i);    // select from a decreasing set
    Serial.print(RandomDraw[tmp]);
    if (i < MAX_VALUE)
      Serial.print(",");
    swap(tmp, (MAX_VALUE+1)-i);             // switch the chosen one with the last of the selection set.
  }
  Serial.println(" ");
}


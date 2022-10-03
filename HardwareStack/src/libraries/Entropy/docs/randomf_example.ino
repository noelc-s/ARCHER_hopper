#include <Entropy.h>

void setup()
{
  float random_num;

  Entropy.Initialize();

  // Obtain a random floating [0,1)
  random_num = Entropy.randomf(); // return a 0 or a 1

  // Obtain a random float [0,10)
  random_num = Entropy.randomf(10);

  // Obtain a random float [101. 200)
  random_num = Entropy.randomf(101,200);
}

void loop()
{
}

#include "ArxTypeTraits.h"

#if 1 // Check with standard libray or just ArxTypeTraits
#if __AVR__
#include <ArduinoSTL.h>
#endif
#include <limits>
#endif

// Check this doesn't break
using namespace arx;

std::function<void()> foo = []() {};

// When using ArduinoSTL, this is *not* constexpr
int y = std::numeric_limits<int>::max();
// But this is :-D
constexpr int z = arx::stdx::numeric_limits<int>::max();

// Check that void_t is available and not ambiguous
using x = std::void_t<int>;

// Check that initializer_list works
size_t listlen(std::initializer_list<int> s) { return s.size(); }
const size_t len = listlen({1, 2, 3});

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  if (random(2)) {
    foo = []() {
      Serial.println("A");
    };
  } else {
    new (&foo) std::function<void()>([]() {
      Serial.println("B");
    });
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  foo();
}

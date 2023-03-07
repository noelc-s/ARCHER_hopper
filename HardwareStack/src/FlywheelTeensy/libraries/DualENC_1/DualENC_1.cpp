#include <DualENC_1.h>

DualENC_1::DualENC_1(int s1,int s2){
  _SS1 = s1;
  _SS2 = s2;
}

void DualENC_1::initEncoders(){
  // Set slave selects as outputs
  pinMode(_SS1, OUTPUT);
  pinMode(_SS2, OUTPUT);
  
  // Raise select pins, Communication begins when you drop the individual select signal
  digitalWrite(_SS1,HIGH);
  digitalWrite(_SS2,HIGH);
  // Initialize encoder 1
  digitalWrite(_SS1,LOW);         // Begin SPI conversation
  SPI1.transfer(0x88);            // Write to MDR0
  SPI1.transfer(0x03);            // Configure to 4 byte mode
  digitalWrite(_SS1,HIGH);        // Terminate SPI conversation 
  delay(10);
  // Initialize encoder 2
  digitalWrite(_SS2,LOW);         // Begin SPI conversation
  SPI1.transfer(0x88);            // Write to MDR0
  SPI1.transfer(0x03);            // Configure to 4 byte mode
  digitalWrite(_SS2,HIGH);        // Terminate SPI conversation 
}

long DualENC_1::readEncoder(int encID){
  // Initialize temporary variables for SPI read
  unsigned int count_1 = 0, count_2 = 0, count_3 = 0, count_4 = 0;
  long count_value;  
  
  // Read encoder 1
  if(encID == 1){
    digitalWrite(_SS1,LOW);                  // Begin SPI conversation
    SPI1.transfer(0x60);                     // Request count
    count_1 = SPI1.transfer(0x00);           // Read highest order byte
    count_2 = SPI1.transfer(0x00);           // Read next Byte
    count_3 = SPI1.transfer(0x00);           // Read next Byte
    count_4 = SPI1.transfer(0x00);           // Read lowest order byte
    digitalWrite(_SS1,HIGH);                 // Terminate SPI conversation 
    // Calculate encoder count
    count_value = (count_1 << 8) + count_2;
    count_value = (count_value << 8) + count_3;
    count_value = (count_value << 8) + count_4;
    
    return count_value;
  }
  
  // Read encoder 2
  else if(encID == 2){
    digitalWrite(_SS2,LOW);                  // Begin SPI conversation
    SPI1.transfer(0x60);                     // Request count
    count_1 = SPI1.transfer(0x00);           // Read highest order byte
    count_2 = SPI1.transfer(0x00);           // Read next Byte
    count_3 = SPI1.transfer(0x00);           // Read next Byte
    count_4 = SPI1.transfer(0x00);           // Read lowest order byte
    digitalWrite(_SS2,HIGH);                 // Terminate SPI conversation 
    // Calculate encoder count
    count_value = (count_1 << 8) + count_2;
    count_value = (count_value << 8) + count_3;
    count_value = (count_value << 8) + count_4;
    
    return count_value;
  }
}

void DualENC_1::clearEncoderCount(int encID){
  if((encID==0)||(encID==1)){
    // Set encoder1's data register to 0
    digitalWrite(_SS1,LOW);      // Begin SPI conversation
    // Write to DTR
    SPI1.transfer(0x98);
    // Load data
    SPI1.transfer(0x00);          // Highest order byte
    SPI1.transfer(0x00);          
    SPI1.transfer(0x00);          
    SPI1.transfer(0x00);          // lowest order byte
    digitalWrite(_SS1,HIGH);     // Terminate SPI conversation
    
    delayMicroseconds(100);  // provides some breathing room between SPI conversations
    
    // Set encoder1's current data register to center
    digitalWrite(_SS1,LOW);      // Begin SPI conversation
    SPI1.transfer(0xE0);    
    digitalWrite(_SS1,HIGH);     // Terminate SPI conversation
  }
  if((encID==0)||(encID==2)){
    // Set encoder2's data register to 0
    digitalWrite(_SS2,LOW);      // Begin SPI conversation
    // Write to DTR
    SPI1.transfer(0x98);    
    // Load data
    SPI1.transfer(0x00);          // Highest order byte
    SPI1.transfer(0x00);          
    SPI1.transfer(0x00);          
    SPI1.transfer(0x00);          // lowest order byte
    digitalWrite(_SS2,HIGH);     // Terminate SPI conversation
    
    delayMicroseconds(100);  // provides some breathing room between SPI conversations
    
    // Set encoder2's current data register to center
    digitalWrite(_SS2,LOW);      // Begin SPI conversation
    SPI1.transfer(0xE0);    
    digitalWrite(_SS2,HIGH);     // Terminate SPI conversation
  }
}
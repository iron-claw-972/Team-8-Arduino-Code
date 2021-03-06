// Robust Rotary encoder reading
//
// Copyright John Main - best-microcontroller-projects.com
//
#define CLK 12
#define DATA 8

#define CLK1 7
#define DATA1 6

void setup() {
  pinMode(CLK, INPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT);
  pinMode(DATA, INPUT_PULLUP);

  pinMode(CLK1, INPUT);
  pinMode(CLK1, INPUT_PULLUP);
  pinMode(DATA1, INPUT);
  pinMode(DATA1, INPUT_PULLUP);
  Serial.begin (115200);
  Serial.println("KY-040 Start:");
}

static uint8_t prevNextCode = 0;
static uint16_t store=0;

static uint8_t prevNextCode1 = 0;
static uint16_t store1=0;

void loop() {
static int8_t c,val;
static int8_t c1,val1;

   if( val=read_rotary() ) {
      c +=val;
      Serial.println("Left:");
      Serial.println(c);
   }

   if( val1=read_rotary1() ) {
      c1 +=val1;
      Serial.println("Right:");
      Serial.println(c1);
   }
}

// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(DATA)) prevNextCode |= 0x02;
  if (digitalRead(CLK)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}

int8_t read_rotary1() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode1 <<= 2;
  if (digitalRead(DATA1)) prevNextCode1 |= 0x02;
  if (digitalRead(CLK1)) prevNextCode1 |= 0x01;
  prevNextCode1 &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode1] ) {
      store1 <<= 4;
      store1 |= prevNextCode1;
      //if (store1==0xd42b) return 1;
      //if (store1==0xe817) return -1;
      if ((store1&0xff)==0x2b) return -1;
      if ((store1&0xff)==0x17) return 1;
   }
   return 0;
}

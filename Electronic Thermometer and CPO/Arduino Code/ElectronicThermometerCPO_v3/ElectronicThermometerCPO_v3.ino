#define dot 18
#define off 17
#define sep 16
#define delay_digit 450
#define delay_off   50

const int mode = 9;
const int buzzer = 8;
const int button = 10;
const int temp = 0;
const int ssd[7] = {1,2,3,4,5,6,7}; //a,b,c,d,e,f,g 
bool checklong = true;
uint8_t count = 0;
float temperature = 0;
int temperature_int = 0;
const char digit[19] = {0x7e,0x0c,0xb6,0x9e,0xcc,0xda,0xfa,0x0e,0xfe,0xce,0xee,0xf8,0x72,0xbc,0xf2,0xe2,0x04,0x00,0xb8};    //0,1,2,3,4,5,6,7,8,9,a,b,c,d,e,f,dash,off,small o
volatile bool is_complete = false;
volatile uint8_t temp_now[10] = {0,off,0,off,dot,off,0,off,sep,off};
volatile uint8_t index = 0;
volatile bool l_delay = false;

void TIM16_WriteOCR1A( unsigned int i ){
  unsigned char sreg;
  /* Save global interrupt flag */
  sreg = SREG;
  /* Disable interrupts */
  cli();
  /* Set OCR1A to i */
  OCR1A = i;
  /* Restore global interrupt flag */ 
  SREG = sreg;
}

void setup_tim1int(void){
  sei();
  TCCR1B = (1 << CS11) | (1 << WGM12);
  TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect){
  PORTA = digit[temp_now[index]];
  index++;
  //index %= 10;
  if(index > 9){
    index = 0;
    is_complete = false;
  }
  if(!l_delay){
    TIM16_WriteOCR1A(65000);
    l_delay = !l_delay;
  }
  else if(l_delay){
    TIM16_WriteOCR1A(30000);
    l_delay = !l_delay;
  }
  //is_complete = false;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(mode,INPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(button,INPUT);
  pinMode(temp,INPUT);
  TCCR0A = (1 << COM0A0) | (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
  OCR0A = 255 - 0;
  DDRA |= (1 << DDA7) | (1 << DDA6) | (1 << DDA5) | (1 << DDA4) | (1 << DDA3) | (1 << DDA2) | (1 << DDA1);
  PORTA = digit[off];
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(mode)){
    sei(); 
    if(!is_complete){
      temperature = 0;
      for(int i = 0; i<64; i++){
        temperature += analogRead(temp);
      }
      temperature /= 64;
      temperature = (temperature*500)/1023;
      temperature_int = (temperature * 10.0) + 0.5; 
      temp_now[0] = ((temperature_int / 100) % 10);
      temp_now[2] = ((temperature_int / 10) % 10);
      temp_now[6] = (temperature_int % 10);
      is_complete = true;
      setup_tim1int();
      TIM16_WriteOCR1A(65000);
    }
    if(!digitalRead(button)){
      TCCR0A = (1 << COM0A0) | (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
      TCCR0B = (1 << CS00);
      OCR0A = 255 - 0;
      delay(10);
      while(!digitalRead(button)){
        OCR0A = 255 - 127;
      }
      OCR0A = 255 - 0;
      delay(10);
    }  
  }
  else if(!digitalRead(mode)){
    cli();
    if(!digitalRead(button)){
      TCCR0A = (1 << WGM01) | (1 << WGM00);
      OCR0A = 0xFF;
      TCCR0B = (1 << CS02) | (1 << CS00);
      TCNT0 = 0;
      delay(10);
      while(!digitalRead(button)){
        if(TCNT0 > 250){
          TCCR0B = 0x00;
          count++;
          count %= 16;
          PORTA = digit[count];
          delay(15);
        }
      }
      delay(10);
      count++;
      count %= 16;
      //PORTA = digit[count];
    }
    PORTA = digit[count];
  }
}

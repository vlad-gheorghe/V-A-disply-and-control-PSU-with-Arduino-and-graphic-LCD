#include <EEPROM.h>
//#include <GyverOLED.h>
#include <GyverINA.h>
#include "GyverEncoder.h"

//#include <Fonts/FreeSansBold9pt7b.h> // Font rotunjit, modern și compact
#include <U8g2lib.h>
#include <SPI.h>

#include <SPI.h>

//INA226 ina(0.0188f, 5.0f, 0x40); //(*0.02 R, Imax 5A, ADR)  
INA226 ina(0.011f, 10.0f, 0x40);



//U8G2_ST7565_EA_DOGM128_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 5, /* dc=*/ 7, /* reset=*/ 6);
U8G2_ST7565_NHD_C12864_F_4W_SW_SPI u8g2(U8G2_R0, /*scl/sck=*/ 13, /*si/mosi=*/ 11, /*cs=*/ 5, /*rs/dc=*/ A2, /*reset=*/ 6);
//EEPROM ADRES

#define E_ST  0  //byte
#define E_C_PWM   1 //int 
#define E_V_PWM   3 //int
int int_val = 0;
byte byte_val = 0;


#define MaxAdcCNT 8

int DIV_PWM_V = 136.5;  
int DIV_PWM_C = 511;  

int PWM_V = 0; 
int PWM_C = 0; 

int PWM_C_MIN = 0;  //0 A
int PWM_C_MAX = 4094; // Max 4 A
int PWM_V_MAX = 4094; // Max 31 V

float OutV, OutC, OutP, ADC_IN, SetI, SetU = 0;
byte Mode,X_1_10, OutST, AdcCicles = 0; 
byte POWER_SW_CNT, ENC_SW_CNT, MODE_SW_CNT = 0;


#define ENC_A 2
#define ENC_B 3
#define ENC_SW 12

Encoder enc1(ENC_A, ENC_B, ENC_SW);  

#define MODE_SW A0
#define OUT_SW 4

#define MODE_LED 8
#define OUT_LED 7

#define SET_U  1
#define SET_I  2

float vechi_OutV = -1.0;
float vechi_OutC = -1.0;
float vechi_OutP = -1.0;

//******************************************************************

void PrinVal()
{  
  // Calculele pentru valorile tale
  SetU = PWM_V;
  SetU = SetU / DIV_PWM_V;

  SetI = PWM_C - PWM_C_MIN;
  SetI = SetI / DIV_PWM_C;

  // Începem bucla de desenare obligatorie pentru U8g2
  u8g2.clearBuffer(); // Șterge bufferul intern din RAM
  
  // ==========================================
  // ZONA SETĂRI (Text mic, stânga)
  // ==========================================
  u8g2.setFont(u8g2_font_5x7_tr); // Un font foarte mic, extrem de compact (5x7 pixeli)
  
  u8g2.drawStr(0, 42, "U:");
  u8g2.setCursor(12, 42); u8g2.print(SetU, 1); u8g2.print("V");

  u8g2.drawStr(0, 56, "I:");
  u8g2.setCursor(12, 56); u8g2.print(SetI, 2); u8g2.print("A");

  if(Mode == SET_U)
  {
    u8g2.drawStr(0, 12, "SET U");  
    u8g2.setCursor(0, 24); u8g2.print(PWM_V); 
  } 
  else if(Mode == SET_I)
  {
    u8g2.drawStr(0, 12, "SET I");  
    u8g2.setCursor(0, 24); u8g2.print(PWM_C);      
  }     
  
  // ==========================================
  // ZONA VALORI REALE (Text mare, dreapta)
  // ==========================================
  // Folosim un font rotunjit și elegant predefinit în U8g2 (mărime potrivită pentru 128x64)
  u8g2.setFont(u8g2_font_9x15B_tf); // Cifre îngroșate de ~15 pixeli înălțime

  // --- TENSIUNE REALĂ (OutV) ---
  u8g2.setCursor(55, 15); // Aliniat la dreapta
  u8g2.print(OutV, 2);
  u8g2.print("V");

  // --- CURENT REAL (OutC) ---
  u8g2.setCursor(55, 38);
  u8g2.print(OutC, 2);
  u8g2.print("A");
  
  // --- PUTERE REALĂ (OutP) ---
  u8g2.setCursor(55, 61);
  u8g2.print(OutP, 1);
  u8g2.print("W");  

  u8g2.sendBuffer(); // Trimite totul deodată pe ecran
}


//*******************************************************************

void setup() 
{



  TCCR1A = 0;  //Curățarea regiștri
  TCCR1B = 0;  //Curățarea regiștri
  
  TCCR1A |= (1 << WGM11);                  //Setarea modului Fast PWM (Modul 14)
  TCCR1B |= (1 << WGM12) | (1 << WGM13); 
  TCCR1B |= (1 << CS10);                  //Setarea frecvenței (Prescaler 1)
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);  //Activarea ieșirilor PWM pe pini
  
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  ICR1 = 0xFFF; // Max CNT   ICR1 = 0xFFF (4095 în zecimal): Setează valoarea maximă până la care numără timerul. Aceasta oferă o rezoluție foarte fină a PWM-ului, de 12 biți 
  OCR1A = 0; //PWM_C;//0;  // SET_I // 9 pin PWM
  OCR1B = 0; //PWM_V; //0;  // SET_U // 10 pin PWM 

  delay(500);  

  pinMode(OUT_SW, INPUT_PULLUP);
  pinMode(MODE_SW, INPUT_PULLUP);  
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(OUT_LED, OUTPUT);  
  pinMode(MODE_LED, OUTPUT); 
  
  digitalWrite(OUT_LED,LOW);
  digitalWrite(MODE_LED,LOW);

  Mode = 0;
  X_1_10 = 0;

  enc1.setType(TYPE2);
  
  attachInterrupt(0, isrCLK, CHANGE);    
  attachInterrupt(1, isrDT, CHANGE);    
    
  ina.begin();  
  
  ina.setSampleTime(INA226_VBUS, INA226_CONV_2116US);   
  ina.setSampleTime(INA226_VSHUNT, INA226_CONV_8244US); 
  ina.setAveraging(INA226_AVG_X4);   
  
 
  u8g2.begin();
u8g2.setContrast(155); // GMG12864 are nevoie de ajustare de contrast pentru a fi lizibil!

  PrinVal();
  
  EEPROM.get(E_ST, byte_val);
  if(byte_val != 5)
  {
    EEPROM.put(E_ST, 5);    
    EEPROM.put(E_C_PWM, PWM_C);
    EEPROM.put(E_V_PWM, PWM_V);    
    }
  else
  {
    EEPROM.get(E_C_PWM, PWM_C);
    EEPROM.get(E_V_PWM, PWM_V);
    }  

}

//--------------------------------------------------------------------

void isrCLK() 
{
  enc1.tick();  
 }

void isrDT() 
{
  enc1.tick(); 
 }

//--------------------------------------------------------------------

void loop() 
{
  enc1.tick();
      
  //--------------------------------------------------------------------

  if (digitalRead(ENC_SW) == 0)
  {
    if(ENC_SW_CNT < 200)ENC_SW_CNT++;
    
    if(ENC_SW_CNT == 5)
    {
      Mode++;
      if(Mode > 2) Mode = 0;
     } 
  }
  else ENC_SW_CNT = 0;   

  //--------------------------------------------------------------------

  if (digitalRead(MODE_SW) == 0)
  {
    if(MODE_SW_CNT < 200)MODE_SW_CNT++;
    
    if(MODE_SW_CNT == 2)
    {
      X_1_10++;
      if (X_1_10>1)X_1_10 = 0;
      
      if(X_1_10 ==0) digitalWrite(MODE_LED,LOW);
      else digitalWrite(MODE_LED,HIGH);
     } 
  }
  else MODE_SW_CNT = 0;   
  
  //--------------------------------------------------------------------
  
  if (enc1.isRight())
  {
    if(Mode == SET_U)
    {
      if(X_1_10 == 0)
      {
        if(PWM_V < PWM_V_MAX - 1) PWM_V = PWM_V + 1;
        }
        else 
        {
          if(PWM_V < PWM_V_MAX - 50) PWM_V = PWM_V + 50;
          }
        OCR1B = PWM_V; // SET_U 
      }

    if(Mode == SET_I)
    {
      if(X_1_10 == 0)
      {
        if(PWM_C < PWM_C_MAX - 5 )PWM_C = PWM_C + 5;
       }
       else
       {
        if(PWM_C < PWM_C_MAX - 50)PWM_C = PWM_C + 50;
        }
       OCR1A = PWM_C; // SET_I 
      }        
   }      
  //--------------------------------------------------------------------

  if (enc1.isLeft())
  {
    if(Mode == SET_U)
    {
      if(X_1_10 == 0)
      {
        if(PWM_V > 1)PWM_V = PWM_V - 1;
        }
        else 
        {
          if(PWM_V > 50) PWM_V = PWM_V - 50;
          }
        OCR1B = PWM_V; // SET_U 
      }

    if(Mode == SET_I)
    {
      if(X_1_10 == 0)
      {
        if(PWM_C > 5)PWM_C = PWM_C - 5;
       }
       else
       {
        if(PWM_C > PWM_C_MIN + 50)PWM_C = PWM_C - 50;
        }
       OCR1A = PWM_C; // SET_I 
      }        
   }
   
  //--------------------------------------------------------------------

  if (digitalRead(OUT_SW) == 0)
  {
    if(POWER_SW_CNT < 200)POWER_SW_CNT++;    
    if(POWER_SW_CNT == 2)
    {
      if(OutST == 0)
      {
        OutST = 1;
        digitalWrite(OUT_LED,HIGH); 
        OCR1A = PWM_C; // SET_I  
        OCR1B = PWM_V; // SET_U  
        
        EEPROM.get(E_C_PWM, int_val);
        if(int_val != PWM_C) EEPROM.put(E_C_PWM, PWM_C);

        EEPROM.get(E_V_PWM, int_val);
        if(int_val != PWM_V) EEPROM.put(E_V_PWM, PWM_V);
                
       }
      else
      {
        OutST = 0;
        digitalWrite(OUT_LED,LOW);
        OCR1A = 0;  // SET_I  
        OCR1B = 0;  // SET_U 
       }
     } 
  }
  else POWER_SW_CNT = 0;     


  

  //---------------------------------------------------------------------
  
    OutV = ina.getVoltage();
    OutC = ina.getCurrent();  
    OutV = OutV *1.009;  // * V cal
    OutC =OutC*1.063;
    if(OutC < 0) OutC = OutC * -1;
    OutP = OutV * OutC;
    delay(150);
    PrinVal();   

  //delay(10);  
//---------------------------------------------------------------------- 
}


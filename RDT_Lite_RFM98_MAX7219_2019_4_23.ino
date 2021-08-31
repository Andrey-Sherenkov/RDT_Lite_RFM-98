/* 2018-11-14
 *  2019-4-23 нарушенна совместимость радиопротокола!
 *  2019-4-30 нарушенна совместимость радиопротокола!
 *  2021 edit to git terminal
*/
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "MAX7219.h"

#define enableradio
#define comandamount 4
#define debug
#define decodemessage
#define activezumm
#ifdef activezumm
  #define zummstart digitalWrite(zummPin,1); 
  #define zummstop  digitalWrite(zummPin,0);
#else
  #define zummstart analogWrite(zummPin,127); 
  #define zummstop  analogWrite(zummPin,0);
#endif
//детермик в медленном режиме!!!

#define uartspeed 9600
#ifdef enableradio
  #include <SPI.h>
  #include <RH_RF95.h>
  // Singleton instance of the radio driver
  RH_RF95 radio;
  float freq=860.0;
  //M=маяк:type;uid,num  ;rssi;bat;time  ;fix ;alt     lat     ;lon    ;hdop   ;end=0;
  //[0]   ;[1] ;[2-3-4-5];[6] ;[7];[8-11];[12];[13-14] ;[15-17];[18-20];[21-22];[23] ;
#endif
#define btnlewel 0
#define WDT1SREAL 1046
#define WDT8SREAL 8388
#define FASTTXTIME 10
#define stepUconst 0.0051
#define modemax comandamount+2

const unsigned int  c_btn_time_down_one =100, c_btn_time_down =500;
/* c_btn_time_down = время удержания кнопки, c_btn_time_dbl = минимальная пауза перед вторым нажатием, c_btn_time_dbl_reset = максимальная пауза перед вторым нажатием, 
c_btn_time_start = время долгого нажатия для запуска; время измеряется в циклах, продолжительность цикла от 15 до 20 мс */
const int BUFLEN = 255, Umax=830, ULedOn=710, UOff=580;
//A3=715=3,68V;740=3,8V;792=4,08V;960=5,04V;min=532(отсечка)~2,7V;680~3,5V;max=5,37V  //680 при отработке глючит сенсорную кнопку
//700 =10%bat;790 =100%
//ULedOn - минимальный заряд батарейки для полёта; 710~3,66V
const byte buttonPin[]={3,A0,A1,A2,A3}, ledPin=6, wifiEnPin=7, zummPin=9, pelengPin=A6,  batPin=A7;
const  String  S_TimerDataLite = "TL ";
const  String  S_TimerRDT = "CRDT ";
#ifdef debug
const  String  S_TimerDataSimple = "TS ";
#endif
char instr[BUFLEN];
uint8_t strlenh,state=0;// состояния таймера: 0- первый запуск. 1- ожидание. 2- програмирование. 3- села батарейка. 4- работа. 5- писк
int iterat = 1,rssi;

unsigned long zummstarttime=0,timestop=0;
uint16_t data[comandamount];              //2^16= 65536 шагов= 1310 сек= 21 мин;шаг=1/50сек;
uint32_t nummodel;
volatile uint8_t strvec=0;

byte dataIn = 4;
byte load = 5;
byte clock = 6;
MAX7219 ledscr(load, dataIn, clock);


byte mode=0, wifistate=1, findmode=0;
unsigned long buttonPress[5];
byte btn_fast[2];//={0,0};
   


void setup(){
  wdt_disable();
  
  Serial.begin(uartspeed);
  #ifdef debug
    Serial.println(F("start"));
  #endif
  for(uint8_t n=1;n<=8;n++){
    ledscr.maxAll(n,1); 
  }
  EEPROMtoTimerData();
  #ifdef enableradio
    if (!radio.init()){
      #ifdef debug
        Serial.println(F("InitRadioFailed"));
      #endif
      ;
    }
    if (!radio.setFrequency(freq)){
      #ifdef debug
        Serial.println(F("SetFrequencyFailed"));
      #endif
      ;
    }
    #ifdef debug
    else{
      Serial.print(F("SetFrequency "));Serial.println(freq);
    }
    #endif
    #ifdef slowradio
      radio.setModemConfig(RH_RF95::Bw7_8Cr48Sf128);
      #ifdef debug
        Serial.println(F("\tRadio:Bw7_8Cr48Sf128"));
      #endif
    #else
      radio.setModemConfig(RH_RF95::Bw500Cr45Sf128);
      #ifdef debug
        Serial.println(F("\tRadio:Bw500Cr45Sf128"));
      #endif
    #endif
    radio.setTxPower(17);  //if > 20 =20
    if(!radio.sleep()){
      #ifdef debug
        Serial.println(F("\tRadioNoSleep"));
      #endif
      ;
    }
  #endif
  pinMode(ledPin, OUTPUT);
  pinMode(zummPin, OUTPUT);
  pinMode(wifiEnPin, OUTPUT);
  digitalWrite(wifiEnPin, 1);//включить WiFi
  #if (btnlewel == 0)
    for(byte c=0;c<5;c++){                                       // назначение входом и подключение встроенного резистора к плюсу
      pinMode(buttonPin[c], INPUT_PULLUP); 
    }
    #ifdef debug
      Serial.println(F("ActiveLevelLOW"));
    #endif
  #else
    for(byte c=0;c<5;c++){
      pinMode(buttonPin[c], INPUT); 
    }
    #ifdef debug
      Serial.println(F("ActiveLevelHIGH"));
    #endif
  #endif
  //analogReference(INTERNAL);
  #ifdef debug
    Serial.print(F("Vbat="));
    Serial.println(analogRead(batPin));
  #endif
  delay(100);//время на выравнивание напряжения на делителе
  #ifdef debug
    Serial.print(F("A6="));
    Serial.println(analogRead(A6));
    Serial.print(F("A7="));
    //for(byte m=0;m<10;m++){
    //  delay(100);
      Serial.println(analogRead(A7));
    //}
  #endif
}

void loop(){
   #ifdef enableradio
        if (radio.available()){
          uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);
          if (radio.recv(buf, &len)) {
            //сначало ответ потом остальное!!!
            if(digitalRead(buttonPin[1]) == btnlewel || ('M' == buf[0] && byte(nummodel>>24) == buf[2] && byte(nummodel>>16) == buf[3] && byte(nummodel>>8) == buf[4] && byte(nummodel|7) == (buf[5]|7))){ 
              //отвечаем только своему таймеру
              uint8_t datat[] = "D    OK";        //надо uint8_t, char работает с варнингом
              datat[1]=byte(nummodel>>24);
              datat[2]=byte(nummodel>>16);
              datat[3]=byte(nummodel>>8);
              datat[4]=byte(nummodel);
              //Send a reply
              radio.send(datat, sizeof(datat));
              #ifdef debug
                Serial.println("\nSent a reply");
              #endif
              radio.waitPacketSent();
            }
            #ifdef debug
            else{
              Serial.println("\n\tnonreqvest");
              Serial.print(buf[0],HEX);Serial.print("_");
              Serial.print(buf[1],HEX);Serial.print("_");
              Serial.print(buf[2],HEX);Serial.print("_");
              Serial.print(buf[3],HEX);Serial.print("_");
              Serial.print(buf[4],HEX);Serial.print("_");
              Serial.println(buf[5],HEX);
            }
            #endif
            int rssi = radio.lastRssi();
            #ifdef debug
              Serial.print(F("got request: "));
              Serial.println((char*)buf);
              for(byte v=0;v<len;v++){
                Serial.print(buf[v],HEX);Serial.print(" ");
              }
              Serial.print(F("\tRSSI: "));
              Serial.println(rssi, DEC);
              //Serial.print("\t");
            #endif
            #ifdef debug
              #ifdef decodemessage
                //M=маяк:type;uid,num  ;rssi;bat;time  ;fix ;alt     lat     ;lon    ;hdop   ;end=0;
                //[0]   ;[1] ;[2-3-4-5];[6] ;[7];[8-11];[12];[13-14] ;[15-17];[18-20];[21-22];[23] ;
                if(buf[0]=='M'){
                  Serial.print("\nМаяк \tnum=");
                }else{
                  Serial.print("Сигнал \tnum=");
                }
                Serial.print(((uint32_t(buf[2])<<24) |(uint32_t(buf[3])<<16) |(uint16_t(buf[4])<<8) | buf[5]), DEC);
                Serial.print("\trssi=");
                Serial.print(buf[6], DEC);
                Serial.print("\tbat=");
                Serial.print((float(buf[7])*0.1),1);
                uint32_t txtime=uint32_t(buf[8])<<24 | uint32_t(buf[9])<<16 | uint32_t(buf[10])<<8 | uint32_t(buf[11]);
                Serial.print("\ttime=");
                Serial.print(txtime);
              #endif
            #endif
            Serial.println("");
            uint32_t mn=((uint32_t(buf[2])<<24) |(uint32_t(buf[3])<<16) |(uint16_t(buf[4])<<8) | buf[5]);
            int mb=int(buf[7]);
            uint32_t mt=uint32_t(buf[8])<<24 | uint32_t(buf[9])<<16 | uint32_t(buf[10])<<8 | uint32_t(buf[11]);
            //"MD ",num        ,rssi ,bat  ,time             ,fix-alt  ,lat        ,lon        ,hdop
            //0-1-2,3-4-5-6-7-8,9-10 ,11-12,13-14-15-16-17-18,19-20-21 ,22-23-24-25,26-27-28-29,30-31-32-33
            //             preamb,      num                                                                   ,   rssi                                                  ,     bat                                         ,      time                                                                                                                                                   ,  fix alt   [9];[10-11]                                                                                                ,  lat                                                                                                                                                    ,  lon
            char uartbuf[28];
            uartbuf[0]='M';
            uartbuf[1]='D';
            uartbuf[2]=' ';
            //num
            uartbuf[3]=char(((mn>>30)&0x3F)|0x40);
            uartbuf[4]=char(((mn>>24)&0x3F)|0x40);
            uartbuf[5]=char(((mn>>18)&0x3F)|0x40);
            uartbuf[6]=char(((mn>>12)&0x3F)|0x40);
            uartbuf[7]=char(((mn>>6)&0x3F)|0x40);
            uartbuf[8]=char(((mn)&0x3F)|0x40);
            //rssi
            uartbuf[9]=char(((buf[3]>>6)&0x3F)|0x40);
            uartbuf[10]=char(((buf[3])&0x3F)|0x40);
            //bat//char(((mb>>6)&0x3F)|0x40)+char(((mb)&0x3F)|0x40)
            uartbuf[11]=char(((mb>>6)&0x3F)|0x40);
            uartbuf[12]=char(((mb)&0x3F)|0x40);
            //time//char(((mt>>30)&0x3F)|0x40)+char(((mt>>24)&0x3F)|0x40)+char(((mt>>18)&0x3F)|0x40)+char(((mt>>12)&0x3F)|0x40)+char(((mt>>6)&0x3F)|0x40)+char(((mt)&0x3F)|0x40)
            uartbuf[13]=char(((mt>>30)&0x3F)|0x40);
            uartbuf[14]=char(((mt>>24)&0x3F)|0x40);
            uartbuf[15]=char(((mt>>18)&0x3F)|0x40);
            uartbuf[16]=char(((mt>>12)&0x3F)|0x40);
            uartbuf[17]=char(((mt>>6)&0x3F)|0x40);
            uartbuf[18]=char(((mt)&0x3F)|0x40);
            //fix-alt
            uartbuf[19]=char((((buf[9]>>2))&0x3F)|0x40);
            uartbuf[20]=char((((buf[9]<<4)|(buf[10]>>4))&0x3F)|0x40);
            uartbuf[21]=char((((buf[11]>>6)|(buf[10]<<2))&0x3F)|0x40);
            uartbuf[22]=char(((buf[11])&0x3F)|0x40);
            //lat//char((((buf[12]>>2))&0x3F)|0x40)+char((((buf[13]>>4)|(buf[12]<<4))&0x3F)|0x40)+char((((buf[14]>>6)|(buf[13]<<2))&0x3F)|0x40)+char(((buf[14])&0x3F)|0x40)
            uartbuf[23]=char((((buf[12]>>2))&0x3F)|0x40);
            uartbuf[24]=char((((buf[13]>>4)|(buf[12]<<4))&0x3F)|0x40);
            uartbuf[25]=char((((buf[14]>>6)|(buf[13]<<2))&0x3F)|0x40);
            uartbuf[26]=char(((buf[14])&0x3F)|0x40);
            //lon//char((((buf[15]>>2))&0x3F)|0x40)+char((((buf[16]>>4)|(buf[15]<<4))&0x3F)|0x40)+char((((buf[17]>>6)|(buf[16]<<2))&0x3F)|0x40)+char(((buf[17])&0x3F)|0x40) +'0');
            uartbuf[27]=char((((buf[15]>>2))&0x3F)|0x40);
            uartbuf[28]=char((((buf[16]>>4)|(buf[15]<<4))&0x3F)|0x40);
            uartbuf[29]=char((((buf[17]>>6)|(buf[16]<<2))&0x3F)|0x40);
            uartbuf[30]=char(((buf[17])&0x3F)|0x40);
            //end
            uartbuf[31]=0;
            Serial.println(uartbuf);
            //0  |3 |6|8|10   |16 | 19|23 |27
            //|  |  | | |     |   |   |   |

            bool peleng=0;
            if (findmode !=3) {
              if(buf[2]==byte(nummodel>>24) && buf[3]==byte(nummodel>>16) &&  buf[4]==byte(nummodel>>8) && (buf[5]|7)==byte(nummodel|7)){
                //пеленгатор с номером
                peleng=1;
              }
            }else{
              //пеленг без номера
              peleng=1;
            }
            if(peleng){
              #ifdef debug
                Serial.print("\nA6=");
                Serial.print((analogRead(pelengPin)>>2));
                Serial.print(" \t-rssi=");
                Serial.print((-rssi));
                Serial.print("\tlen=");
                Serial.println(((analogRead(pelengPin)>>2)+rssi)*4);
              #endif
              if((analogRead(pelengPin)>>2)+rssi>0){
                zummstart;
                zummstarttime=millis();
                timestop=zummstarttime+(((analogRead(pelengPin)>>2)+rssi)*4);
              }
            }
            if(mode==0){
              switch(findmode){
                case 3:
                  //find all
                case 0:   //rssi
                  ledscr.print(rssi,0,4);
                  if(buf[6]>200){
                    ledscr.print(256-buf[6],4,4);
                  }else{
                    ledscr.print(-buf[6],4,4);
                  }
                break;
                case 1:   //UID
                  ledscr.maxSingle(8, 62);  //2-3-4-5
                  ledscr.print(((uint32_t(buf[2])<<21) |(uint32_t(buf[3])<<13) |(uint16_t(buf[4])<<5) | (buf[5]>>3)),1,4);
                  ledscr.maxSingle(2, 21);
                  ledscr.print((buf[5]&7),7,1);
                break;
                case 2:   //bat
                  ledscr.maxSingle(8, 31);
                  ledscr.print((float(buf[7])*0.1),1,3);
                break;
              }
            }
          }
          else
          {
            #ifdef debug
              Serial.println(F("recv failed"));
            #endif
          }
        }
  #endif
  
  if (digitalRead(buttonPin[0]) == btnlewel) {                        // выполнять если нажата кнопка
    zummstop;
    //rdt
    #ifdef enableradio
      uint8_t datat[] = "CRDT 0020";//надо uint8_t, char работает с варнингом//"CRDT 20"
      datat[5] = 0;
      datat[6] = 0;
      datat[7] = 50;
      datat[8] = 0;
      //Send
      radio.sleep();
      delay(10);
      //radio.setModemConfig(RH_RF95::Bw7_8Cr48Sf128);
      radio.setModemConfig(RH_RF95::Bw31_25Cr48Sf512); 
      #ifdef debug
        //Serial.println(F("\tRadio:Bw7_8Cr48Sf128"));
        Serial.println(F("\tRadio::Bw31_25Cr48Sf512"));
      #endif
      delay(10);
      radio.setModeIdle();
      while(digitalRead(buttonPin[0]) == btnlewel){
        radio.send(datat, sizeof(datat));
        radio.waitPacketSent();
        delay(200);
        #ifdef debug
          Serial.println(F("packet send"));
        #endif
      }
      radio.sleep();
      radio.setModemConfig(RH_RF95::Bw500Cr45Sf128);
      #ifdef debug
        Serial.println(F("\tRadio::Bw500Cr45Sf128"));
      #endif
      radio.setModeIdle();
    #endif
    #ifdef debug
        Serial.println(F("BTN1"));
    #endif
  }
  if(millis()>timestop){
    zummstop;
  }
  while(Serial.available() > 0) {
      if(addchar(Serial.read())){
        parcestr();
      }
  }
  for(byte c=1;c<5;c++){
    if (digitalRead(buttonPin[c]) == btnlewel) {
      if(buttonPress[c]==0){buttonPress[c]=millis()+c_btn_time_down_one;}
      if(buttonPress[c]<millis()){
        buttonPress[c]=millis()+c_btn_time_down;
        switch(c){
          case 1: 
            if(mode<modemax){mode++;}else{mode=0;} 
          break;
          case 2: 
            if(mode>0){mode--;}else{mode=modemax;}
            break;
          case 3:
          if(mode>0 && mode<= comandamount){
            if(btn_fast[0]<50){
              if(data[mode-1]<65535){
                data[mode-1]++;
              }
              btn_fast[0]++;
            }else{
              if(btn_fast[0]<100){
                if(data[mode-1]<65485){
                  data[mode-1]+=50;
                }else{
                  data[mode-1]=65535;
                }
                btn_fast[0]++;
              }else{
                if(btn_fast[0]<150){
                  if(data[mode-1]<64535){
                    data[mode-1]+=1000;
                  }else{
                    data[mode-1]=65535;
                  }
                  btn_fast[0]++;
                }else{
                  if(btn_fast[0]<200){
                    if(data[mode-1]<55535){
                      data[mode-1]+=10000;
                      btn_fast[0]++;
                    }else{
                      data[mode-1]=65535;
                    }
                  }
                }
              }
            }
          }else{
            if(mode==0){
              if(findmode<3){
                findmode++;
              }
            }
            if(mode==comandamount+1){
              //tx data
              state=testdata();
              if(state==1){
                #ifdef debug
                    Serial.println(F("\tSend "));
                #endif
                uint8_t datap[] = "TL 123456789012";
                //nummodel
                datap[S_TimerDataLite.length()]=byte(nummodel>>24);
                datap[S_TimerDataLite.length()+1]=byte(nummodel>>16);
                datap[S_TimerDataLite.length()+2]=byte(nummodel>>8);
                datap[S_TimerDataLite.length()+3]=byte(nummodel);
                for (u8 c = 0 ;c <comandamount;c++){
                   datap[c*2+S_TimerDataLite.length()+4]=byte(data[c]>>8);
                   datap[c*2+S_TimerDataLite.length()+5]=byte(data[c]);
                }
                
                radio.send(datap, sizeof(datap));
                radio.waitPacketSent();
              }
            }else{
              if(mode==modemax){
                wifistate=1;
                digitalWrite(wifiEnPin,1);
              }
            }
          }
          break;
          case 4:
          if(mode>0 && mode<= comandamount){
            if(btn_fast[1]<50){
              if(data[mode-1]>=1){
                data[mode-1]--;
              }
              btn_fast[1]++;
            }else{
              if(btn_fast[1]<100){
                if(data[mode-1]>=50){
                  data[mode-1]-=50;
                }else{
                  data[mode-1]=0;
                }
                btn_fast[1]++;
              }else{
                if(btn_fast[1]<150){
                  if(data[mode-1]>=1000){
                    data[mode-1]-=1000;
                  }else{
                    data[mode-1]=0;
                  }
                  btn_fast[1]++;
                }else{
                  if(btn_fast[1]<200){
                    if(data[mode-1]>=10000){
                      data[mode-1]-=10000;
                      btn_fast[1]++;
                    }else{
                      data[mode-1]=0;
                    }
                  }
                }
              }
            }
          }else{
            if(mode==0){
              if(findmode>0){
                findmode--;
              }
            }
            if(mode==comandamount+1){
              //tx data?
            }else{
              if(mode==modemax){
                wifistate=0;
                digitalWrite(wifiEnPin,0);
              }
            }
          }
          break;
        }
        ledscr.clear();
        if(mode == 0){        //Find
          ledscr.maxSingle(8, 71);
          ledscr.maxSingle(7, 16);
          ledscr.maxSingle(6, 21);
          ledscr.maxSingle(5, 61);
        }
        if(mode>0 && mode<= comandamount){
          ledscr.print(mode,0,1);
          //ledscr.print(float(data[mode-1])/50,2,6);
          ledscr.print(long(data[mode-1])*20,1);
        }
        if(mode == comandamount+1){        //Set
          ledscr.maxSingle(8, 91);
          ledscr.maxSingle(7, 79);
          ledscr.maxSingle(6, 15);
        }
        if(mode == modemax){        //WiFi
          ledscr.maxSingle(8, 190);//62
          ledscr.maxSingle(7, 22);
          ledscr.maxSingle(6, 71);
          ledscr.maxSingle(5, 16);
          ledscr.print(wifistate,7,1);
        }
      }
    }else{
      buttonPress[c]=0;
      if(c==3)btn_fast[0]=0;
      if(c==4)btn_fast[1]=0;
    }
  }
} 


/*WDT BYTE variables for setting timer value
WDTO_15MS   0,016384  WDTO_30MS 0,032768  WDTO_60MS  0,065536   WDTO_120MS  0,131072  WDTO_250MS  0,262144
WDTO_500MS  0,524288  WDTO_1S   1,048576  WDTO_2S    2,097152   WDTO_4S     4,194304  WDTO_8S     8,388608   
+/- 1%, в 1.6.4 для работы 4 и 8 надо править ./hardware/tools/avr/avr/include/avr/wdt.h*/
void delayWDT(byte timer) {
  sleep_enable(); //enable the sleep capability
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set the type of sleep mode. Default is Idle
  ADCSRA &= ~(1<<ADEN); //Turn off ADC before going to sleep (set ADEN bit to 0)
  WDTCSR |= 0b00011000;    //Set the WDE bit and then clear it when set the prescaler, WDCE bit must be set if changing WDE bit   
  WDTCSR =  0b01000000 | timer; //Or timer prescaler byte value with interrupt selectrion bit set
  wdt_reset(); //Reset the WDT 
  sleep_cpu(); //enter sleep mode. Next code that will be executed is the ISR when interrupt wakes Arduino from sleep
  sleep_disable(); //disable sleep mode
  ADCSRA |= (1<<ADEN); //Turn the ADC back on
}

void delaypowerSaveWDT(byte timer) {
  sleep_enable(); //enable the sleep capability
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); //set the type of sleep mode. Default is Idle
  ADCSRA &= ~(1<<ADEN); //Turn off ADC before going to sleep (set ADEN bit to 0)
  WDTCSR |= 0b00011000;    //Set the WDE bit and then clear it when set the prescaler, WDCE bit must be set if changing WDE bit   
  WDTCSR =  0b01000000 | timer; //Or timer prescaler byte value with interrupt selectrion bit set
  wdt_reset(); //Reset the WDT 
  sleep_cpu(); //enter sleep mode. Next code that will be executed is the ISR when interrupt wakes Arduino from sleep
  sleep_disable(); //disable sleep mode
  ADCSRA |= (1<<ADEN); //Turn the ADC back on
}
//This is the interrupt service routine for the WDT. It is called when the WDT times out. 
//This ISR must be in your Arduino sketch or else the WDT will not work correctly
ISR (WDT_vect){
  wdt_disable();
   MCUSR = 0; //Clear WDT flag since it is disabled, this is optional
}  // end of WDT_vect



void parcestr(){
    if(instr[0]=='U' && instr[1]=='I' && instr[2]=='D' && instr[3]==' ' ){
      int val=-256;
      String instring=instr;
      byte pos1 = instring.indexOf(" ");
      byte pos2 = instring.indexOf(" ",pos1+1);
      String number = instring.substring(pos1,pos2);
      val = number.toInt();
      if((val>0) && (val<32765)){
        Serial.print(F("UID "));
        Serial.println(val);
        nummodel=val;
        EEPROM.write(509, byte(val>>8)); delay(10);
        EEPROM.write(510, byte(val)); delay(10);
      }
    }

    if(instr[0]=='F' && instr[1]=='R' && instr[2]=='E' && instr[3]=='Q' && instr[4]==' '){
      float val=0;
      String instring=instr;
      byte pos1 = instring.indexOf(" ");
      byte pos2 = instring.indexOf(" ",pos1+1);
      String number = instring.substring(pos1,pos2);
      val = atof(number.c_str());
      if((val>410) && (val<960) ){
        Serial.print(F("len "));
        Serial.print(val);
        freq = val;
        long val2 = long(freq*1000);
        EEPROM.write(506, byte(val2)); delay(10);
        EEPROM.write(505, byte(val2>>8)); delay(10);
        EEPROM.write(504, byte(val2>>16)); delay(10);
      }
    }
    if(instr[0]=='h' && instr[1]=='e' && instr[2]=='l' && instr[3]=='p' && instr[4]=='!'){
      #ifdef debug
      Serial.print(F("\tUID "));
      Serial.print(nummodel);
      Serial.print(F("\tFREQ "));
      Serial.print(freq);
      #endif
    }

    uint8_t CRC =0, CRC1=0, c ;
  bool inen=1;

  for(byte i=0;i<S_TimerDataLite.length();i++){
    if(instr[i] != S_TimerDataLite.charAt(i) ){
      inen=0;
      //Serial.print("if_non "+String(instr[i])+" " +String(S_TimerDataLite.charAt(i))+" \t");
    }
  }
  if(inen){
    #ifdef debug
        Serial.println("input byte");
        for(byte i=0;i<254;i++){
          Serial.print((instr[i]&0x3F),HEX);Serial.print(" ");
        }
        Serial.println("");
    #endif

    for (c = S_TimerDataLite.length() ;c <comandamount*3+S_TimerDataLite.length();c++){ 
      CRC = CRC^instr[c];
    }
    CRC1 = (instr[comandamount*3+S_TimerDataLite.length()]&0x3F) | (int(instr[comandamount*3+1+S_TimerDataLite.length()]&0x3F)<<6);
    #ifdef debug
      Serial.print("\nCRC ");Serial.print(CRC,HEX);Serial.print(" ");Serial.println(CRC1,HEX);
    #endif
    if(CRC==CRC1){
      #ifdef debug
        Serial.println("CRC_OK");
      #endif
      for(byte v=0;v<comandamount;v++){
        data[v]=(instr[v*3+S_TimerDataLite.length()]&0x3F) | (int(instr[v*3+1+S_TimerDataLite.length()]&0x3F)<<6)| (int(instr[v*3+2+S_TimerDataLite.length()]&0x3F)<<12);
      }
      state=testdata();
      if(state==1){
        #ifdef debug
          Serial.println(F("\tSend "));
        #endif
      

      
      //const  String  S_TimerDataLite = "TL ";
      //for (u8 c = 0 ;c <comandamount;c++){
      //   data[c]=uint16_t(buf[c*2+S_TimerDataLite.length()])<<8|(buf[c*2+S_TimerDataLite.length()+1]);
      //}
      
      uint8_t datap[] = "TL 123456789012   ";
      //nummodel
      datap[S_TimerDataLite.length()]=byte(nummodel>>24);
      datap[S_TimerDataLite.length()+1]=byte(nummodel>>16);
      datap[S_TimerDataLite.length()+2]=byte(nummodel>>8);
      datap[S_TimerDataLite.length()+3]=byte(nummodel);
      for (u8 c = 0 ;c <comandamount;c++){
        datap[c*2+S_TimerDataLite.length()+4]=byte(data[c]>>8);
        datap[c*2+S_TimerDataLite.length()+5]=byte(data[c]);
      }
      
      radio.send(datap, sizeof(datap));
      radio.waitPacketSent();

        #ifdef debug
          for (u8 c = 0 ;c <comandamount;c++){
            Serial.print(data[c]);Serial.print(" ");
          }
          Serial.print(F("\tOk\ntxbuf:\n"));
          //send  54 4C 20 32 0 0 32 0 FA 2 EE 23 28 31 32 0
          //rx    54 4C 20 32 0 0 32 0 FA 2 EE 23 28 31 32 0
          for (u8 c = 0 ;c <sizeof(datap);c++){
            Serial.print(datap[c],HEX);Serial.print(" ");
          }
          Serial.println("");
        #endif
      }
    }
  }
}

bool addchar(char input){   //NL&CR   10 13
  if(input==13){  //Carriage return
    ;
  }else{
    if(input==10){  //Line feed   linux
      instr[strvec]=0;
      strlenh=strvec;
      strvec=0;
      return 1;
    }else{
      if(input=='$'){  //начало строки GPS
        strvec=0; 
      }
      //inputstr+=input;  ошибки со строкой длинее 128;string inputstr
      /*instr[strvec++]=input; убивает оптимизатор!!! */
      instr[strvec]=input;
      if(strvec<BUFLEN-1){
        strvec++;
      }else{
        strvec=0;    // защита от выхода за пределы буфера
      }
    }
  }
  return 0;
}

byte testdata(){
  byte out=1;
  for(byte i=0;i<comandamount-1;i++){
    for(byte k=i+1;k<comandamount;k++){
      if(data[i] > data[k]){
        out=2;
        #ifdef debug
          Serial.println(F("mintime > max time, error"));
          Serial.flush() ;// дождаться окончания передачи
        #endif
      }
    }
  }
  if(data[comandamount-1] ==0){
    out=2;
    #ifdef debug
      Serial.println(F("time=0, error"));
      Serial.flush() ;// дождаться окончания передачи
    #endif
  }
  return out;
}

void EEPROMtoTimerData(){                                                              // чтение настроек из флеш памяти
  #ifdef debug
    Serial.println(F("EEPROMtoTimerData"));
  #endif

  nummodel= (uint16_t(EEPROM.read(509))<<8) | EEPROM.read(510);
  if(nummodel==0 || nummodel > 32765){
    nummodel=(uint16_t(50)<<8);
    EEPROM.write(510, 0); delay(10);
    EEPROM.write(509, 50); delay(10);
  }

  #ifdef enableradio
    freq=(long(EEPROM.read(504))<<16 | long(EEPROM.read(505))<<8 | EEPROM.read(506))/1000;
    if(freq<410 || freq>950){
      freq=860.0f;
      EEPROM.write(506, byte(860000)); delay(10);
      EEPROM.write(505, byte(860000>>8)); delay(10);
      EEPROM.write(504, byte(860000>>16)); delay(10);
    }
  #endif
}


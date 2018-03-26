#include <stdint.h>
#include<stdio.h>
#include <stdbool.h>
#include<ctype.h>
#include<math.h>
#include <string.h>
#include <strings.h>
#include "tm4c123gh6pm.h"

//Leds Defined
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  //PF3
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))   //PF2
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  //PF1

//5 Gpio Pins Defined
#define MEAS_C (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))  //PD6
#define LOWERSIDE_R (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))   // PA4
#define INTEGRATE (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))  //PF4
#define MEAS_LR (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))   //PA5
#define HIGHSIDE_R (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))  //PE4

//Globally Declared Varibles
int count=0;
int MAX_CHAR=80;
char str[81];
char field_offset[20];
char field_type[10];
int field_count=0;
char alphabet[]={65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122};
int numeric[]={48,49,50,51,52,53,54,55,56,57};
char delimiter[]={32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,58,59,60,61,62,63,91,92,93,94,95,96,123,124,125,126};
int i,s;

bool timeMode = false;
uint32_t time = 0;
bool timeUpdate = false;
extern void ResetISR(void); //ResetISR

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
       {
           __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
           __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
           __asm("             CBZ  R1, WMS_DONE1");   // 8
           __asm("             NOP");                  // 5
           __asm("             NOP");                  // 5
           __asm("             B    WMS_LOOP1");       // 10
           __asm("WMS_DONE1:   SUB  R0, #1");          // 1
           __asm("             CBZ  R0, WMS_DONE0");   // 1
           __asm("             NOP");                  // 1
           __asm("             B    WMS_LOOP0");       // 2
           __asm("WMS_DONE0:");                        // ---
                                                       // 40 clocks/us + error
       }

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;
    // Enable GPIO port peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOF;

    // Configure LED and INTEGRATE
    GPIO_PORTF_DIR_R = 0x1E;  // bits 1,2,3 are LED, 4 is Integrate
    GPIO_PORTF_DR2R_R = 0x1E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable 1,2,3,4

    //Configure MEAS_LR and LOWERSIDE_R
    GPIO_PORTA_DIR_R = 0x30;  // bits 4 and 5
    GPIO_PORTA_DR2R_R = 0x30; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0x30;  // enable 4 and 5

    //Configure HIGHSIDE_R and AN0
    GPIO_PORTE_DIR_R = 0x18;  // bits 4 and 3
    GPIO_PORTE_DR2R_R = 0x18; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = 0x18;  // enable 4 and 3

    //Configure MEAS_C
    GPIO_PORTD_DIR_R = 0x40;  // bit 6
    GPIO_PORTD_DR2R_R = 0x40; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R = 0x40;  // enable 6

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
        UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


   // Configure AN0 as an analog input
           SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
           GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
           GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
           GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3
           ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
           ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
           ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
           ADC0_SSMUX3_R = 0;                               // set first sample to AN0
           ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
           ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


    // Configure analog comparator
           SYSCTL_RCGCACMP_R=SYSCTL_RCGCACMP_R0; //Enable the analog comparator clock
           GPIO_PORTC_DEN_R &= ~0XC0; //Enable the clock to the appropriate GPIO modules
           GPIO_PORTC_AMSEL_R|=0xC0; //Enable the GPIO port/pin associated with the input signals

    //Unlock PF0
           GPIO_PORTF_LOCK_R=GPIO_LOCK_KEY;
           GPIO_PORTF_CR_R=GPIO_LOCK_LOCKED;
           GPIO_PORTF_AFSEL_R=0x01;

    // PORTF
           GPIO_PORTF_PCTL_R &= ~GPIO_PCTL_PF0_M;
           GPIO_PORTF_DEN_R|=0X01;
           GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF0_C0O;
           GPIO_PORTF_DIR_R|=0X01;
           GPIO_PORTF_DEN_R|=0X01;
           GPIO_PORTF_AFSEL_R|=0x01;
           COMP_ACREFCTL_R=0x0000020F; //specifies whether the resistor ladder is powered on as well as the range and tap
           COMP_ACCTL0_R= 0x00000402; //The output of the comparator is inverted prior to being processed by hardware
           waitMicrosecond(10); //delay

           // Configure FREQ_IN for frequency counter
              GPIO_PORTC_AFSEL_R |= 0x10;                      // select alternative functions for FREQ_IN pin
              GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to FREQ_IN
              GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
              GPIO_PORTC_DEN_R |= 0x10;                        // enable bit 6 for digital input

}

void setTimerMode()
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;     // turn-on timer
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER0_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER0_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    NVIC_EN2_R |= 1 << (INT_WTIMER0A-16-64);         // turn-on interrupt 120 (WTIMER0A)
}

int readAdc0Ss3()
  {
      ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
      while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
      return ADC0_SSFIFO3_R;                           // get single result from the FIFO
  }

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}


void inputString()
        {
            int count=0;
        while(1)    // Endless loop
                  {
                   char c = getcUart0();
                     if (c == 8)  //backspace
                     {
                         GREEN_LED=1; // Indication for backspace
                         waitMicrosecond(500);
                         GREEN_LED=0;

                     if(count>0)
                       {
                           count--;
                       }
                     }
                     else if(c==13)  //CR
                        {
                             str[count]=0;
                             GREEN_LED= 1; // Indication for CR
                             waitMicrosecond(500);
                             GREEN_LED=0;
                             break;
                        }

                     else if (c>=32) //Space
                      {
                         str[count++]=c;
                         if(count==MAX_CHAR)
                             {
                                 str[count]=0;
                                 GREEN_LED= 1; // Indication for space
                                 waitMicrosecond(500);
                                 GREEN_LED=0;
                                 break;
                             }
                      }
               }
        }

// Convert user entered string to lowercase (Case insensitive)
  void convert_to_lowercase()
    {
         int k;
                     for(k = 0; k<strlen(str); k++)
                         {
                            str[k] = tolower (str[k]);
                         }
                     putsUart0(str);
     }


// Converting delimiters to null characters
   void Conv_Delimeters_to_Null()
    {
       int count,r;
       count=strlen(str);
       for(i = 0; str[i] != '\0'; i++)
                        {
                           if ((str[i]>=32 && str[i] <= 47) || (str[i] >= 58 && str[i] <= 64)|| (str[i] >= 91 && str[i] <= 96)||(str[i] >= 123 && str[i] <= 126)) //Check for delimiters
                            {
                               str[i] = 0; // make it null
                            }
                        }
       for(r=0; r<count; r++)
              {
                  if((str[r]=='\0')    &&    (str[r+1]>=65 && str[r+1]<=122))  //Compare if alphabet is there after null
                  {
                      field_offset[field_count]=(r+1);
                      field_type[field_count]='A';   //Alphabet
                      field_count++;
                  }
                  if((str[r]=='\0')    &&    (str[r+1]>=48 && str[r+1]<=57))  //Compare if number is there after null
                  {
                      field_offset[field_count]=(r+1);
                      field_type[field_count]='N';    //Number
                      field_count++;
                  }
             }
     }


   //Compare R,L,C strings and turn on RGB leds
   void iscommand()
   {
   if(strcmp((str+field_offset[0]),"inductance")==0) //Compare string for L and turn on blue led
    {
       putsUart0("inductance");
       BLUE_LED=1;
       waitMicrosecond(100000);
       BLUE_LED=0;
     }

   if(strcmp((str+field_offset[1]),"resistance")==0)  //Compare string for R and turn on green led
   {
        putsUart0("resistance");
        GREEN_LED=1;
        waitMicrosecond(100000);
        GREEN_LED=0;
    }

   if(strcmp((str+field_offset[1]),"capacitance")==0)  //Compare string for C and turn on red led
   {
        putsUart0("capacitance");
        RED_LED=1;
        waitMicrosecond(100000);
        RED_LED=0;
   }
   }

   // Compare strings and turn on/off five GPIOs
   void stringComp_gpio_Onoff()
           {
       for(s=0;s<field_count;s++)
                 {
                 if(strcmp((str+field_offset[s]),"io")==0)  //compare string on offset0
                         {
                             if(strcmp((str+field_offset[s+1]),"measlr")==0) //compare string on offset1
                                {
                                     if(strcmp((str+field_offset[s+2]),"on")==0) //compare string on offset2
                                           {
                                              putsUart0("MEAS_LR ON");
                                              MEAS_LR=1;    //turn on
                                              BLUE_LED=1;
                                              waitMicrosecond(500); //delay
                                              BLUE_LED=0;
                                           }

                                     else if(strcmp((str+field_offset[s+2]),"off")==0)  //compare string on offset2
                                              {
                                              MEAS_LR=0;    //turn off
                                              RED_LED=1;
                                              waitMicrosecond(500); //delay
                                              RED_LED=0;
                                              }
                               }

                else if(strcmp((str+field_offset[s+1]),"measc")==0)
                        {
                           if(strcmp((str+field_offset[s+2]),"on")==0)
                               {
                                  putsUart0("MEAS_C ON");
                                  MEAS_C=1;    //turn on
                                  BLUE_LED=1;
                                  waitMicrosecond(500);
                                  BLUE_LED=0;
                              }

                           else if(strcmp((str+field_offset[s+2]),"off")==0)
                              {
                                   MEAS_C=0;     //turn off
                                   RED_LED=1;
                                   waitMicrosecond(500);
                                   RED_LED=0;
                              }
                       }

                else if(strcmp((str+field_offset[s+1]),"highsider")==0)
                       {
                          if(strcmp((str+field_offset[s+2]),"on")==0)
                              {
                                  putsUart0("HIGHSIDE_R ON");
                                  HIGHSIDE_R=1;  //turn on
                                  BLUE_LED=1;
                                  waitMicrosecond(500);
                                  BLUE_LED=0;
                              }
                          else if(strcmp((str+field_offset[s+2]),"off")==0)
                              {
                                  HIGHSIDE_R=0;   //turn off
                                  RED_LED=1;
                                  waitMicrosecond(500);
                                  RED_LED=0;
                              }
                      }

               else if(strcmp((str+field_offset[s+1]),"lowersider")==0)
                      {
                         if(strcmp((str+field_offset[s+2]),"on")==0)
                            {
                                 putsUart0("LOWERSIDE_R ON");
                                 LOWERSIDE_R=1;   //turn on
                                 BLUE_LED=1;
                                 waitMicrosecond(500);
                                 BLUE_LED=0;
                            }

                        else if(strcmp((str+field_offset[s+2]),"off")==0)
                           {
                                LOWERSIDE_R=0;    //turn off
                                putsUart0("LOWERSIDE_R OFF");
                                RED_LED=1;
                                waitMicrosecond(500);
                                RED_LED=0;
                           }
                     }

               else if(strcmp((str+field_offset[s+1]),"integrate")==0)
                       {
                         if(strcmp((str+field_offset[s+2]),"on")==0)
                           {
                               putsUart0("INTEGRATE ON");
                               INTEGRATE=1;   //turn on
                               BLUE_LED=1;
                               waitMicrosecond(500);
                               BLUE_LED=0;

                           }

              else if(strcmp((str+field_offset[s+2]),"off")==0)
                          {
                              INTEGRATE=0;    //turn off
                              putsUart0("INTEGRATE OFF");
                              RED_LED=1;
                              waitMicrosecond(500);
                              RED_LED=0;
                           }
                     }
                }
          }
     }

   //Calculate instant voltage and display measured voltage
   void instVoltage()
   {
       char str1[20];
       char str2[20];
       int raw;
       float instantVoltage;

   // Read sensor (Turn on measc,integrate and turn off measlr)
       if(strcmp((str+ field_offset[0]),"io")==0)   //compare string on offset0
               {
                if(strcmp((str+ field_offset[1]),"meas")==0)   //compare string on offset1
                 {
                  if(strcmp((str+ field_offset[2]),"voltage")==0)   //compare string on offset2
                  {
                      BLUE_LED=1;
                      waitMicrosecond(100000);
                      BLUE_LED=0;
                      MEAS_LR=0;
                      MEAS_C=1;
                      INTEGRATE=1;

             while(1)
              {
                 raw = readAdc0Ss3(); //read raw value
                 instantVoltage = (raw / 4096.0 * 3.3)-0.00727;  //calculate voltage and offset value subtracted

            // display raw ADC value and voltage
                 putsUart0("\r\n Raw:");
                 sprintf(str1, "%u", raw);
                 putsUart0(str1);

                 putsUart0("\r\n Instant Voltage:");
                 sprintf(str2, "%f", instantVoltage);
                 putsUart0(str2);
                 waitMicrosecond(500000);
               }
           }
       }
   }
}

   //Calculate IIR voltage
   void iirVoltage()
   {
       char str1[20];
       char str2[20];
       char str4[20];
       int raw;
       float instantVoltage, iirVoltage;
       float alpha = 0.99;
       int firstUpdate = true;

       if(strcmp((str+ field_offset[0]),"io")==0)  //compare string on offset0
                  {
                   if(strcmp((str+ field_offset[1]),"iir")==0)   //compare string on offset1
                    {
                     if(strcmp((str+ field_offset[2]),"voltage")==0)    //compare string on offset2
                     {
                                 BLUE_LED=1;
                                 waitMicrosecond(100000);
                                 BLUE_LED=0;
                                 MEAS_LR=0;
                                 waitMicrosecond(500);
                                 MEAS_C=1;
                                 INTEGRATE=1;

                        while(1)
                            {
                                raw = readAdc0Ss3();
                                instantVoltage = (raw / 4096.0 * 3.3)-0.00727;  //calculate voltage
                                    if (firstUpdate)
                                        {
                                            iirVoltage = instantVoltage;
                                            firstUpdate = false;
                                        }
                                    else
                                            iirVoltage = iirVoltage * alpha + instantVoltage * (1-alpha); //calculate iir voltage

                      // Display raw ADC value and voltages
                                    putsUart0("\r\n Raw:");
                                    sprintf(str1, "%u", raw);
                                    putsUart0(str1);

                                    putsUart0("\r\n Instant Voltage:");
                                    sprintf(str2, "%f", instantVoltage);
                                    putsUart0(str2);

                                    putsUart0("\r\n Iir Voltage:");
                                    sprintf(str4, "%f", iirVoltage);
                                    putsUart0(str4);
                                    waitMicrosecond(500000);
                         }
                   }
             }
        }
   }

   // Period timer service publishing latest time measurements every positive edge
      void WideTimer0Isr()
           {
                time = WTIMER0_TAV_R;                        // read counter input
                WTIMER0_TAV_R = 0;                           // zero counter for next edge
                time /= 40;                                  // scale to us units
                timeUpdate = true;                           // set update flag
                GREEN_LED ^= 1;                              // status
                WTIMER0_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
           }

   //Measuring time
      void testcmd()
           {
             if(strcmp((str+field_offset[0]),"test")==0)
                {
                  if(strcmp((str+field_offset[1]),"cmd")==0)
                     {
                          putsUart0("time");
                          LOWERSIDE_R=1;
                          INTEGRATE=1;
                          waitMicrosecond(500);
                          LOWERSIDE_R=0;
                          WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                          HIGHSIDE_R=1;

           while(1)
                    {
                        if (timeUpdate)
                             {
                                 timeUpdate = false;
                                 sprintf(str, "%7lu", time);
                                 putsUart0(str);
                                 ResetISR();
                                 break;
                              }
                   }
               }
          }
    }

      //Measuring resistance in manual mode
      void measure_resistance()
           {
               if(strcmp((str+field_offset[0]),"meas")==0)
                 {
                     if(strcmp((str+field_offset[1]),"res")==0)
                         {
                             putsUart0("RESISTANCE");
                             LOWERSIDE_R=1;   //turn on
                             INTEGRATE=1;     //turn on
                             waitMicrosecond(100000);
                             LOWERSIDE_R=0;   //turn off
                             WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                             MEAS_LR=1;    //turn on
               while(1)
                         {
                              if (timeUpdate)
                                 {
                                     float res1=time/1.45; //formula for R
                                     timeUpdate = false;
                                     sprintf(str, "%f", res1);
                                     putsUart0(str);
                                     ResetISR();
                                     break;
                                 }
                         }
                  }
           }
      }

      //Measuring capacitance in manual mode
      void measure_capacitance()
          {
              char cap[100];
              if(strcmp((str+field_offset[0]),"meas")==0)
                 {
                    if(strcmp((str+field_offset[1]),"cap")==0)
                       {
                           putsUart0("CAPACITANCE");
                           LOWERSIDE_R=1;   //turn on
                           MEAS_C=1;     //turn on
                           waitMicrosecond(100000);
                           LOWERSIDE_R=0;   //turn off
                           WTIMER0_TAV_R=0;   //Zero counter for first period
                           WTIMER0_CTL_R |= TIMER_CTL_TAEN;                   // turn-on counter
                           timeUpdate = false;
                           HIGHSIDE_R=1;  //turn on
                  while(1)
                          {
                              if (timeUpdate)
                              {
                                   time/=1.399;
                                   timeUpdate = false;
                                   sprintf(cap, "%f", time/100000.000); //c=time/100k
                                   putsUart0(cap);
                                   putsUart0("\n\r");
                                   ResetISR();
                                   break;
                              }
                        }
                  }
           }
     }

     //Measuring esr in mannual mode
     void measure_esr()
        {
               uint16_t raw1=0;
               float Voltage1, Voltage2, Voltage3,Voltage ;
               float esr=0;

               if(strcmp((str+field_offset[0]),"meas")==0)
                   {
                      if(strcmp((str+field_offset[1]),"esr")==0)
                         {
                           putsUart0("ESR");
                           raw1 = readAdc0Ss3();
                           Voltage1= raw1*3.3;    //v=raw*3.3/4096-offset
                           Voltage2 = Voltage1/4096;
                           Voltage3 = Voltage2 - .0072;
                           Voltage = Voltage3;

                           putsUart0("Voltage value is:");
                           putsUart0("\n\r");
                           sprintf(str, "%f", Voltage);
                           putsUart0("\n\r");
                           putsUart0(str);

                           putsUart0("\n\r");
                           esr=(101.68/Voltage)-33;   //Voltage=(R/R+esr)*(vdd-vce); R=33,vce=0.044
                           putsUart0("ESR value is:");
                           sprintf(str, "%f", esr);
                           putsUart0("\n\r");
                           putsUart0(str);

                           LOWERSIDE_R=0;
                           MEAS_LR=0;
                           waitMicrosecond(500000);
                           WTIMER0_TAV_R=0;        //Zero counter for first period
                           WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                           MEAS_LR=1;
                           ResetISR();

                      }
                }
          }

     //Measuring inductance in manual mode
          void measure_inductance()
              {
                   uint16_t raw1=0;
                   float Voltage1, Voltage2, Voltage3,Voltage ;
                   float esr=0;
                   float L;
                   if(strcmp((str+field_offset[0]),"meas")==0)
                       {
                           if(strcmp((str+field_offset[1]),"ind")==0)
                               {
                                      putsUart0("INDUCTANCE");
                                      LOWERSIDE_R=1;
                                      MEAS_LR=1;
                                      waitMicrosecond(200000);
                                      raw1 = readAdc0Ss3();
                                      Voltage1= raw1*3.3;
                                      Voltage2 = Voltage1/4096;
                                      Voltage3 = Voltage2 - .0072;
                                      Voltage = Voltage3;
                                      esr=(101.68/Voltage)-33;
                                      MEAS_LR=0;
                                      waitMicrosecond(500000);
                                      WTIMER0_TAV_R=0;           //Zero counter for first period
                                      WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                                      MEAS_LR=1;

                           while(1)
                               {
                                   if (timeUpdate)
                                       {
                                           putsUart0("\n\r");
                                           L= ((33+esr)*time)/1.9;  //L=(R+Resr)*time/(constant=1.299)
                                           sprintf(str, "%f", L);
                                           putsUart0(str);
                                           ResetISR();
                                           break;
                                       }
                              }
                     }
             }
     }

     //Calling function for measuring capacitance in auto mode
     void auto_cap()
        {
            char cap[100];
                            LOWERSIDE_R=0;
                            INTEGRATE=0;
                            MEAS_LR=0;
                            MEAS_C = 0;
                            HIGHSIDE_R = 0;
                            LOWERSIDE_R=1;
                            MEAS_C=1;
                            waitMicrosecond(200000);
                            LOWERSIDE_R=0;
                            WTIMER0_TAV_R=0;
                            WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                            HIGHSIDE_R=1;
                   while(1)
                       {
                           if (timeUpdate)
                               {
                                   time/=1.399;
                                   timeUpdate = false;
                                   sprintf(cap, "%f", time/100000.000);
                                   putsUart0(cap);
                                   putsUart0("\n\r");
                                   ResetISR();
                                   break;
                               }
                       }
        }

     void auto_res()
         {
             LOWERSIDE_R=0;
             INTEGRATE=0;
             MEAS_LR=0;
             MEAS_C = 0;
             HIGHSIDE_R = 0;
             LOWERSIDE_R=1;
             INTEGRATE=1;
             waitMicrosecond(100000);
             LOWERSIDE_R=0;
             WTIMER0_TAV_R=0;
             timeUpdate = false;
             WTIMER0_CTL_R |= TIMER_CTL_TAEN;   // turn-on counter
             MEAS_LR=1;

       while(1)
           {
               if (timeUpdate)
                   {
                       timeUpdate = false;
                       float res1=time/1.45;
                       sprintf(str, "%f", res1);
                       putsUart0(str);
                       WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
                       ResetISR();
                       break;
                   }
           }
        }

        void auto_ind()
            {
                   uint16_t raw1=0;
                   float Voltage1, Voltage2, Voltage3,Voltage ;
                   float esr=0;
                   float L;

                               LOWERSIDE_R=1;
                               MEAS_LR=1;
                               waitMicrosecond(200000);
                               raw1 = readAdc0Ss3();

                               Voltage1= raw1*3.3;
                               Voltage2 = Voltage1/4096;
                               Voltage3 = Voltage2 - .0072;
                               Voltage = Voltage3;

                               esr=(101.68/Voltage)-33;

                               MEAS_LR=0;
                               waitMicrosecond(500000);
                               WTIMER0_TAV_R=0;
                               WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                               MEAS_LR=1;

                     while(1)
                         {
                             if (timeUpdate)
                                 {
                                     putsUart0("\n\r");
                                     L= ((33+esr)*time)/1.9;
                                     sprintf(str, "%f", L);
                                     putsUart0(str);
                                     ResetISR();
                                     break;
                                 }
                         }
            }

     void auto_command()
         {
                int16_t raw2;
                uint16_t j=0;
                float Voltage3[10];
                float Voltage1=0.0;
                float Voltage2=0.0;
                float Voltage=0.0;

                if(strcmp((str+field_offset[0]),"auto")==0)
                    {
                        putsUart0("AUTO");
                        WTIMER0_CTL_R |= TIMER_CTL_TAEN;
                        WTIMER0_TAV_R = 0;
                        timeUpdate=false;
                        LOWERSIDE_R=0;
                        INTEGRATE=0;
                        MEAS_LR=0;
                        MEAS_C = 0;
                        HIGHSIDE_R = 0;
                        MEAS_C=1;
                        LOWERSIDE_R=1;
                        waitMicrosecond(1000);
                        LOWERSIDE_R=0;
                        HIGHSIDE_R=1;
                        MEAS_C=1;

                            for(j=0;j<5;j++)
                                {
                                    raw2 = readAdc0Ss3();
                                    Voltage1= raw2*3.3;
                                    Voltage2 = Voltage1/4096;
                                    Voltage3[j]=Voltage2;
                                    sprintf(str, "%f", Voltage3[j]);
                                    putsUart0(str);
                                    putsUart0("\n\r");
                                }

                            waitMicrosecond(15000000);
                            raw2 = readAdc0Ss3();
                            Voltage1= raw2*3.3;
                            Voltage2 = Voltage1/4096;
                            Voltage = Voltage2;
                            sprintf(str, "%f", Voltage);
                            putsUart0(str);
                            putsUart0("\n\r");

                                if (timeUpdate)
                                    {
                                        timeUpdate=false;
                                        if(Voltage>=3.0)
                                            {
                                                putsUart0("\n\r");
                                                putsUart0("This is a Capacitor");
                                                auto_cap();
                                                WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
                                            }
                                    }

                                else
                                    {
                                        WTIMER0_TAV_R = 0;
                                        timeUpdate=false;
                                        HIGHSIDE_R=0;
                                        MEAS_C=0;
                                        LOWERSIDE_R=0;
                                        INTEGRATE=0;
                                        MEAS_LR=0;
                                        MEAS_LR=1;
                                        LOWERSIDE_R=1;
                                        waitMicrosecond(60000);
                                        raw2 = readAdc0Ss3();
                                        Voltage1= raw2*3.3;
                                        Voltage2 = Voltage1/4096;
                                        Voltage=Voltage2;
                                        sprintf(str, "%f", Voltage);
                                        putsUart0(str);
                                        putsUart0("\n\r");
                                        WTIMER0_CTL_R |= TIMER_CTL_TAEN;
                                        timeUpdate=false;

                                        if(timeUpdate | ((Voltage>2.43) && (Voltage<3.0)))
                                            {
                                                timeUpdate=false;
                                                putsUart0("This is an inductor");
                                                LOWERSIDE_R=0;
                                                INTEGRATE=0;
                                                MEAS_LR=0;
                                                MEAS_C = 0;
                                                HIGHSIDE_R = 0;
                                                WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
                                                auto_ind();
                                            }

                                      else
                                          {
                                                putsUart0("\n\r");
                                                putsUart0("This is a resistor");
                                                timeUpdate=false;
                                                LOWERSIDE_R=0;
                                                INTEGRATE=0;
                                                MEAS_LR=0;
                                                MEAS_C = 0;
                                                HIGHSIDE_R = 0;
                                                auto_res();
                                                WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
                                          }
                             }
                  }
        }


     int main(void)
    {
        // Initialize hardware
        initHw();
        while(1)
        {
            setTimerMode();
            putsUart0("Enter the string\r\n");
            // Turn on green LED
            GREEN_LED = 1;
            // Wait for 500ms
            waitMicrosecond(500000);
            // Turn off green LED
            GREEN_LED = 0;
            // Wait for 500ms
            waitMicrosecond(500000);

            inputString();
            Conv_Delimeters_to_Null();
            convert_to_lowercase();
            iscommand();
            stringComp_gpio_Onoff();
            instVoltage();
            iirVoltage();
            testcmd();
            measure_resistance();
            measure_capacitance();
            measure_esr();
            measure_inductance();
            auto_command();
      }
  }



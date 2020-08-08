// ATtiny412

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 20000000UL/24


#define LED     PIN1_bp
#define IRLED   PIN3_bp
#define SENSPWR PIN2_bp
#define SENSIN  PIN6_bp
#define BUZZER  PIN7_bp


ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;
}

void sleep_2s() {
  cli();
  //while(RTC.PITSTATUS > 0) {}
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc | RTC_PITEN_bm;
  //while(RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
  sei();
  sleep_enable();
  SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
  SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
  sleep_cpu();
  sleep_disable();
}


void sleep_8s() {
  cli();
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;
  sei();
  sleep_enable();
  SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
  SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
  sleep_cpu();
  sleep_disable();
}

void sound_alarm(void) {
  for(unsigned char i=0; i<60; i++) {
    _delay_us(300);
    PORTA.OUTTGL = (1<<BUZZER);
  }
  PORTA.OUTCLR = (1<<BUZZER);
}

uint16_t adcVal;

void ADC0_init(void);
uint16_t ADC0_read(void);

void ADC0_init(void)
{
    /* Disable digital input buffer */
    PORTA.PIN6CTRL &= ~PORT_ISC_gm;
    PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    
    /* Disable pull-up resistor */
    PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;

    ADC0.CTRLC = ADC_PRESC_DIV2_gc      /* CLK_PER divided */
               | ADC_REFSEL_VDDREF_gc;  /* Internal reference */
    
    ADC0.CTRLA = ADC_ENABLE_bm          /* ADC Enable: enabled */
               | ADC_RESSEL_8BIT_gc;
    
    /* Select ADC channel */
    ADC0.MUXPOS  = ADC_MUXPOS_AIN6_gc;
}

uint16_t ADC0_read(void)
{
    /* Start ADC conversion */
    ADC0.COMMAND = ADC_STCONV_bm;
    
    /* Wait until ADC conversion done */
    while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) )
    {
        ;
    }
    
    /* Clear the interrupt flag by writing 1: */
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    
    return ADC0.RES;
}

int detect_obstacle(void) {
    ADC0.CTRLA |= ADC_ENABLE_bm;
    uint16_t sample, sample2;
    PORTA.OUTSET = (1<<SENSPWR)|(1<<IRLED);
    _delay_us(500);
    sample = ADC0_read();
    PORTA.OUTCLR = (1<<IRLED);
    _delay_us(500);
    sample2 = ADC0_read();
    PORTA.OUTCLR = (1<<SENSPWR);   
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
    if (sample>sample2+30) {
      return 1;
    }
    return 0;
}

int main(void)
{
  _PROTECTED_WRITE((CLKCTRL.MCLKCTRLB), CLKCTRL_PDIV_24X_gc | (1 << CLKCTRL_PEN_bp));
  _PROTECTED_WRITE((CLKCTRL.MCLKCTRLA), (0 << CLKCTRL_CLKOUT_bp) | CLKCTRL_CLKSEL_OSC20M_gc);

  // Set all pins to low power mode:
  for (uint8_t i = 0; i < 8; i++) {
     *((uint8_t *)&PORTA + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
  }

  PORTA.DIR |= (1<<BUZZER)|(1<<LED)|(1<<IRLED)|(1<<SENSPWR);
  PORTA.OUT = (0<<BUZZER)|(1<<IRLED)|(1<<SENSPWR);
  ADC0_init();
 
  while(RTC.STATUS > 0) {}
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // 32KHz divided by 32, i.e run at 1.024kHz
  while(RTC.PITSTATUS > 0) {}

  while (1) {
    int sensor;
check_sensor:
    sensor = detect_obstacle();
    if (sensor==1) goto first_detected_wait;
not_detected:
    sleep_8s();
    goto check_sensor;
first_detected_wait:
    sleep_2s();
detect_again:
    sensor = detect_obstacle();
    if (sensor) goto alarm;
    goto not_detected;
alarm:
    sound_alarm();
    sleep_2s();
    goto detect_again;
  }
}

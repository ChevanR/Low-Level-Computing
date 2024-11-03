#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Variabelen voor temp en pressure data
int TensTemp;
int OnesTemp;
int DecimalsTemp;
int hundredsPressure;
int tensPressure;
int onesPressure;

// Kalibratiewaardes
int16_t T1; // Kalibratie waarde Temp 1
int16_t T2; // Kalibratie waarde Temp 2
int16_t T3; // Kalibratie waarde Temp 3
int16_t P1; // Kalibratie waarde Pressure 1
int16_t P2; // Kalibratie waarde Pressure 2
int16_t P3; // Kalibratie waarde Pressure 3

// 7-segment
int dataArray[12] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01110011, // 'P' Pascal
    0b00111001, // 'C' Celsius
};

// Current state
volatile int currentState = 0;

void init()
{
  // I/O poorten
  DDRB = (1 << PB2) | (1 << PB3) | (1 << PB1) | (1 << PB4);
  // USI (Universal Serial Interface, communicatie met bmp280 and 7seg)
  USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK);
}

// Interrupt functie
void interrupt(int freq)
{
  cli();                                                          // Disable interrupts
  TCCR1 |= (1 << CS10) | (1 << CS11) | (1 << CS12) | (1 << CS13); // Prescaler 16384
  OCR1A = F_CPU / 16384 / freq - 1;                               // Calculate the value for the output compare register A in Hz
  TIMSK |= (1 << OCIE1A);                                         // Compare match interrupt
  sei();                                                          // Enable interrupts
}

// Data verzende via USI
uint8_t transfer(uint8_t data)
{
  USIDR = data;
  USISR |= (1 << USIOIF); // Clear overflow flag
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    while (!(USISR & (1 << USIOIF))) // Wait for overflow flag
    {                        
      USICR |= (1 << USITC); // Kloktoggle
    }
  }
  return USIDR;
}

// Segment displayen op 7-segment display
void showSegment(int i)
{
  PORTB &= ~(1 << PB3);
  transfer(i);
  PORTB |= (1 << PB3);
}

// BMP280 modus voor thermometer en barometer
void mode()
{
  PORTB &= ~(1 << PB4);
  transfer(0x74); // thermometer mode
  transfer(0x43); // barometer mode
  PORTB |= (1 << PB4);
}

// Function for temp and pressure sensor data bit readings
void readSensorData(uint8_t command, int16_t *calibrationValue1, int16_t *calibrationValue2)
{
  PORTB &= ~(1 << PB4);                          // Start sensor communication
  transfer(command);                             // Send command
  transfer(0);                                   // Dummy byte
  int8_t number1 = USIDR;                        // First sensor value
  transfer(0);                                   // Dummy byte
  int8_t number2 = USIDR;                        // Second sensor value
  *calibrationValue1 = (number2 << 8) + number1; // Combine two bytes to get the calibration value
  PORTB |= (1 << PB4);                           // End sensor communication
}

// Function to read temperature and pressure calibration values
void readCalibrationValues()
{
  readSensorData(0x88, &T1, &T2); // Kalibratiewaarden voor temp
  readSensorData(0x8A, &T3, &P1); // Kalibratiewaarden voor temp en druk
  readSensorData(0x8E, &P2, &P3); // kalibratiewaarden voor druk
}

// Sensor gegevens opvragen
void request()
{
  long adcT = 0;
  long adcP = 0;
  PORTB &= ~(1 << PB4);
  transfer(0xFA);

  uint32_t calculationOne = transfer(0);
  uint32_t calculationTwo = transfer(0);
  uint32_t calculationThree = transfer(0);

  PORTB |= (1 << PB4);
  adcT = (calculationOne << 12) | (calculationTwo << 4) | (calculationThree >> 4);

  long var1 = ((((adcT >> 3) - ((long)T1 << 1))) * ((long)T2)) >> 11;
  long var2 = (((((adcT >> 4) - ((long)T1)) * ((adcT >> 4) - ((long)T1))) >> 12) * ((long)T3)) >> 14;
  float temp = (var1 + var2) / 5120.0;

  // Variabelen voor tempgegevens
  TensTemp = (int)temp / 10;
  OnesTemp = (int)temp % 10;
  DecimalsTemp = (int)(temp * 10) % 10;

  // Drukgegevens opvragen
  PORTB &= ~(1 << PB4);
  transfer(0xFB);

  calculationOne = transfer(0);
  calculationTwo = transfer(0);
  calculationThree = transfer(0);

  PORTB |= (1 << PB4);
  adcP = (calculationOne << 12) | (calculationTwo << 4) | (calculationThree >> 4);

  // Pressure berekening
  long pressure = ((adcP >> 2) - 1638) * 50;

  // Variabelen voor drukgegevens
  hundredsPressure = (pressure / 100) % 10;
  tensPressure = (pressure / 10) % 10;
  onesPressure = pressure % 10;
}

int main()
{
  init();
  interrupt(1);
  readCalibrationValues();
  while (true)
  {
    mode();
    request();
  }
}

// Interrupt Service Routine for Timer 1 Compare Match A
ISR(TIMER1_COMPA_vect)
{
  switch (currentState)
  {
  case 0:
    showSegment(dataArray[TensTemp]);
    break;
  case 1:
    showSegment(dataArray[OnesTemp]);
    break;
  case 2:
    showSegment(0b10000000); // Decimal
    break;
  case 3:
    showSegment(dataArray[DecimalsTemp]);
    break;
  case 4:
    showSegment(dataArray[11]); // Display 'C'
    break;
  case 5:
    showSegment(dataArray[hundredsPressure]);
    break;
  case 6:
    showSegment(dataArray[tensPressure]);
    break;
  case 7:
    showSegment(dataArray[onesPressure]);
    break;
  case 8:
    showSegment(dataArray[10]); // Display 'P'
    break;
  default:
    break;
  }

  currentState++; // Next state

  if (currentState > 9)
  {
    currentState = 0; // Reset
  }
}

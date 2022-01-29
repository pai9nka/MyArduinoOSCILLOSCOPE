
/*
Простой осциллограф на Atmega168/328, I2C OLED экране и поворотном энкодере. 
Позволяет отображать уровень входного сигнала по времени от 0,9 мс. до 13,3 мс. 
Есть программный детектор частоты цифрового сигнала.
Используется графическая библиотека u8g2 (https://github.com/olikraus/u8g2).

Код скетча: https://drive.google.com/file/d/0B7Yh...
Скетч, использующий таймер для задания частоты опросов. 
Поддерживает длительность "окна" 1, 2, 5, 10, 25, 50, 100, 250, 500 мс. и 1 сек. 
https://drive.google.com/file/d/0B7Yh...
*/
#include <Wire.h>
#include <U8g2lib.h>

const uint8_t pinRotClk = 2;
const uint8_t pinRotDt = 3;
const uint8_t pinRotSw = 4;
const uint8_t pinPWM25 = 5;
const uint8_t pinPWM50 = 6;

const uint8_t bufferSize = 128; // Width of OLED screen

volatile uint8_t buffer[bufferSize];
volatile uint8_t bufferLen;

const uint8_t logicZero = 77; // 1.5V
const uint8_t logicOne = 153; // 3V

enum mode_t : uint8_t { MODE_1MS, MODE_2MS, MODE_5MS, MODE_10MS, MODE_25MS, MODE_50MS, MODE_100MS, MODE_250MS, MODE_500MS, MODE_1S };

const mode_t defMode = MODE_1MS;

volatile mode_t mode = defMode;
mode_t lastMode;

#define ROTATE U8G2_R0 // U8G2_R2 to flip screen

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(ROTATE, U8X8_PIN_NONE);

ISR(ADC_vect) {
  buffer[bufferLen++] = ADCH;
  if (bufferLen >= bufferSize) {
    TCCR1B &= ~(bit(CS12) | bit(CS11) | bit(CS10)); // Stop Timer1 (clear prescaler)
    ADCSRA &= ~(bit(ADATE) | bit(ADIE)); // Turn off ADC auto-triggering
  }
}

EMPTY_INTERRUPT(TIMER1_COMPB_vect);

uint32_t modeToUs() {
  if (lastMode == MODE_1MS)
    return 1000;
  else if (lastMode == MODE_2MS)
    return 2000;
  else if (lastMode == MODE_5MS)
    return 5000;
  else if (lastMode == MODE_10MS)
    return 10000;
  else if (lastMode == MODE_25MS)
    return 25000;
  else if (lastMode == MODE_50MS)
    return 50000U;
  else if (lastMode == MODE_100MS)
    return 100000UL;
  else if (lastMode == MODE_250MS)
    return 250000UL;
  else if (lastMode == MODE_500MS)
    return 500000UL;
  else if (lastMode == MODE_1S)
    return 1000000UL;
}

void getInfo(uint8_t &avg, uint8_t &minimum, uint8_t &maximum, uint16_t &freq) {
  uint8_t edge = 0, edgeCount = 0;
  uint16_t edgeSum = 0;
  uint16_t bufferSum = 0;

  minimum = 255;
  maximum = 0;

  for (uint8_t i = 0; i < bufferSize; ++i) {
    bufferSum += buffer[i];
    if (buffer[i] < minimum)
      minimum = buffer[i];
    if (buffer[i] > maximum)
      maximum = buffer[i];
//    if (i && (buffer[i] > logicOne) && (buffer[i - 1] < logicZero)) { // Rising edge detected
    if (i && (buffer[i] > logicOne) && (buffer[i - 1] < logicOne)) { // Rising edge detected
      if (edge) {
        edgeSum += (i - edge);
        ++edgeCount;
      }
      edge = i;
    }
  }

  avg = bufferSum / bufferSize;
  if (edgeCount) {
    if (lastMode < MODE_1S)
      freq = 1000000UL / (modeToUs() * (edgeSum / edgeCount) / bufferSize);
    else
      freq = 0;
  } else
    freq = 0;
}

void drawScreen(bool info) {
  uint8_t h_;
  uint8_t w_;
  u8g2.setDrawColor(1);
  
  h_ = u8g2.getDisplayHeight();
  w_ = u8g2.getDisplayWidth();
  
  for (int8_t v = 5; v >= 0; --v) {
    uint8_t tickY;
    
    if (v == 5)
      tickY = 0;
    else if (! v)
      tickY = h_ - 1;
    else
      tickY = h_ - h_ * v / 5;
    for (uint8_t x = 0; x < w_; x += 8)
      u8g2.drawPixel(x, tickY);
  }

  for (uint8_t x = 1; x < bufferSize; ++x) {
    u8g2.drawLine(x - 1, map(buffer[x - 1], 0, 255, h_ - 1, 0), x, map(buffer[x], 0, 255, h_ - 1, 0));
  }

  u8g2.setDrawColor(0);
  u8g2.setCursor(0, 0);
  u8g2.print(F("5V"));
  u8g2.setCursor(0, h_ - 8);
  if (info) {
    uint8_t avg, minimum, maximum;
    uint16_t freq;

    getInfo(avg, minimum, maximum, freq);
    u8g2.print(F("avg/min/max:"));
    u8g2.print(5.0 / 255 * avg, 1);
    u8g2.print('/');
    u8g2.print(5.0 / 255 * minimum, 1);
    u8g2.print('/');
    u8g2.print(5.0 / 255 * maximum, 1);
    u8g2.setCursor(0, h_ - 16);
    u8g2.print(F("freq:"));
    if (freq)
      u8g2.print(freq);
    else
      u8g2.print('-');
  } else {
    if (lastMode < MODE_1S) {
      u8g2.print(modeToUs() / 1000);
      u8g2.print(F(" ms"));
    } else {
      u8g2.print(modeToUs() / 1000000UL);
      u8g2.print(F(" s"));
    }
  }
}

void captureBuffer() {
  static uint8_t lastAdcClkDiv = 0;
  uint8_t timClkDiv, adcClkDiv;

  if (lastMode < MODE_1S)
    timClkDiv = 1; // DIV1
  else
    timClkDiv = 2; // DIV8
  if (lastMode == MODE_1MS)
    adcClkDiv = 3; // DIV8
  else if (lastMode == MODE_2MS)
    adcClkDiv = 4; // DIV16
  else if (lastMode == MODE_5MS)
    adcClkDiv = 5; // DIV32
  else if (lastMode == MODE_10MS)
    adcClkDiv = 6; // DIV64
  else
    adcClkDiv = 7; // DIV128

  noInterrupts();

  if (lastMode == MODE_1MS)
    OCR1A = F_CPU / (bufferSize * 1000UL) - 1;
  else if (lastMode == MODE_2MS)
    OCR1A = F_CPU / (bufferSize * 500UL) - 1;
  else if (lastMode == MODE_5MS)
    OCR1A = F_CPU / (bufferSize * 200) - 1;
  else if (lastMode == MODE_10MS)
    OCR1A = F_CPU / (bufferSize * 100) - 1;
  else if (lastMode == MODE_25MS)
    OCR1A = F_CPU / (bufferSize * 40) - 1;
  else if (lastMode == MODE_50MS)
    OCR1A = F_CPU / (bufferSize * 20) - 1;
  else if (lastMode == MODE_100MS)
    OCR1A = F_CPU / (bufferSize * 10) - 1;
  else if (lastMode == MODE_250MS)
    OCR1A = F_CPU / (bufferSize * 4) - 1;
  else if (lastMode == MODE_500MS)
    OCR1A = F_CPU / (bufferSize * 2) - 1;
  else if (lastMode == MODE_1S)
    OCR1A = (F_CPU / 8) / bufferSize - 1;
  OCR1B = OCR1A;
  TCNT1 = 0;
  TCCR1B |= timClkDiv; // Start timer with new prescaler

  if (adcClkDiv != lastAdcClkDiv) {
    ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2)); // Clear prescaler
    ADCSRA |= adcClkDiv; // Set prescaler

    lastAdcClkDiv = adcClkDiv;
  }

  bufferLen = 0;
  ADCSRA |= (bit(ADATE) | bit(ADIE) | bit(ADIF)); // Turn on automatic triggering, want interrupt on completion

  interrupts();

  while (bufferLen < bufferSize); // Wait for filling whole buffer
}

void setup() {

  pinMode(pinRotClk, INPUT_PULLUP); // INPUT
  pinMode(pinRotDt, INPUT_PULLUP); // INPUT
  pinMode(pinRotSw, INPUT_PULLUP);

  pinMode(pinPWM25, OUTPUT);
  pinMode(pinPWM50, OUTPUT);
  analogWrite(pinPWM25, 63); // 25% PWM = 63
  analogWrite(pinPWM50, 127); // 50% PWM

//  attachInterrupt(digitalPinToInterrupt(pinRotClk), rotISR, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(pinRotDt), rotISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000);

  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setFontDirection(0);
  u8g2.setFontMode(0);
  u8g2.setFontPosTop();
  u8g2.setFontRefHeightExtendedText();

  noInterrupts();

  // Setup Timer1
  TCCR1A = 0;
  TCCR1B = bit(WGM12); // CTC
  TIMSK1 = bit(OCIE1B); // Output Compare B Match Interrupt Enable

  // Setup ADC
  ADCSRA = bit(ADEN); // Turn ADC on
  ADMUX = bit(REFS0) | bit(ADLAR); // VCC as AREF, left align and channel 0 as input
  ADCSRB = bit(ADTS0) | bit(ADTS2); // Timer/Counter1 Compare Match B

  interrupts();
}

void loop() {
  
  static bool stopped = false;
  static uint8_t lastBtns = 0B00011100;
  uint8_t btns = PIND & 0B00011100; // D2, D3 and D4

  if (btns != lastBtns) {
    static const uint32_t debounceTime = 20;
    static uint32_t lastTime = 0;

    if (millis() - lastTime >= debounceTime) {
      if ((! (btns & 0B00000100)) && (lastBtns & 0B00000100)) { // D2 became low
        if (mode > MODE_1MS)
          --(*((uint8_t*)&mode));
      }
      if ((! (btns & 0B00001000)) && (lastBtns & 0B00001000)) { // D3 became low
        if (mode < MODE_1S)
          ++(*((uint8_t*)&mode));
      }
      if ((! (btns & 0B00010000)) && (lastBtns & 0B00010000)) { // D4 became low
        stopped = ! stopped;
      }
      lastBtns = btns;
    }
    lastTime = millis();
  }

  if (! stopped) {
    lastMode = mode;
    captureBuffer();
  }

  u8g2.firstPage();
  do {
    drawScreen(stopped);
  } while (u8g2.nextPage());
  
}

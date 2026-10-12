#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 pressure;

// ============================================================
//  ВЛИ часы: DS1302 + 2x74HC595 + динамическая индикация
//  Развёртка на Timer2, без delay в ISR
// ============================================================

// ---------------- ПИНЫ ----------------
#define RST_1302    4
#define SCLK_1302   2
#define DATA_1302   3

// HC595 №1 — аноды (пины 7,8,9)
#define DATA_HC595_1   9    // PB1
#define ENTER_HC595_1  8    // PB0
#define SCLK_HC595_1   7    // PD7

// HC595 №2 — сетки (пины 10,11,12)
#define DATA_HC595_2   10   // PB2
#define ENTER_HC595_2  11   // PB3
#define SCLK_HC595_2   12   // PB4

// ---------------- ДАННЫЕ ----------------
uint8_t number7SEG[] = {
  0x22, 0xEE, 0x0B, 0x4A, 0xC6,
  0x52, 0x12, 0xEA, 0x02, 0x42,
  0xFD, 0xDF, 0xFF, 0xC3
};

uint8_t gridData[16] = {
  0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F
};

// Буферы, которые читает ISR
volatile uint8_t anodeData[16];

// ---------------- БЫСТРАЯ ЗАПИСЬ В HC595 ----------------
// Аноды:  DATA=PB1, SCLK=PD7, LATCH=PB0
// Сетки:  DATA=PB2, SCLK=PB4, LATCH=PB3
inline void shiftBothFast(uint8_t anode, uint8_t grid) {
  for (uint8_t i = 0; i < 8; i++) {
    // --- аноды ---
    if (anode & (1 << i)) PORTB |=  (1 << PB1);
    else                  PORTB &= ~(1 << PB1);
    PORTD |=  (1 << PD7);
    PORTD &= ~(1 << PD7);

    // --- сетки ---
    if (grid & (1 << i))  PORTB |=  (1 << PB2);
    else                  PORTB &= ~(1 << PB2);
    PORTB |=  (1 << PB4);
    PORTB &= ~(1 << PB4);
  }
  // latch обоих
  PORTB |=  (1 << PB0) | (1 << PB3);
  PORTB &= ~((1 << PB0) | (1 << PB3));
}

// ---------------- ISR TIMER2 ----------------
ISR(TIMER2_COMPA_vect) {
  static uint8_t digit = 0;
  static uint8_t chaos = 0;

  // Сначала закрываем сетки, потом меняем аноды, потом открываем нужную
  shiftBothFast(anodeData[digit], gridData[digit]);

  digit++;
  if (digit >= 16)
  {
    chaos++;
    digit = 0;
  }

  if (chaos > 250)
  {
    for (int i = 0; i < 16;i++)
    {
      anodeData[0] = number7SEG[13];
      anodeData[1] = number7SEG[7];
      anodeData[2] = number7SEG[10];   // тире
      anodeData[3] = number7SEG[22];
      anodeData[4] = number7SEG[14];
      anodeData[5] = number7SEG[1];
      anodeData[6] = number7SEG[15];
      anodeData[7] = number7SEG[2];
      anodeData[8] = number7SEG[8];
      anodeData[9] = number7SEG[5];
      anodeData[10] = number7SEG[12];
      anodeData[11] = number7SEG[0x00];
      anodeData[12] = number7SEG[14];
      anodeData[13] = number7SEG[13];
      anodeData[14] = number7SEG[5];
      anodeData[15] = number7SEG[12];
      shiftBothFast(anodeData[i], gridData[i]);
      delay(30);
    }
    chaos = 0;
  }
}

// ---------------- НАСТРОЙКА TIMER2 ----------------
void setupTimer2() {
  noInterrupts();
  TCCR2A = (1 << WGM21);   // CTC
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);    // prescaler 64
  OCR2A  = 26;    
  TCNT2   = 0;
  TIMSK2 = (1 << OCIE2A);
  interrupts();
}

// ---------------- DS1302 ----------------
uint8_t readData(uint8_t regAddr) {
  uint8_t data = 0;

  pinMode(DATA_1302, OUTPUT);
  digitalWrite(SCLK_1302, LOW);
  digitalWrite(DATA_1302, LOW);
  digitalWrite(RST_1302, HIGH);

  // команда (LSB first)
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(DATA_1302, (regAddr >> i) & 0x01);
    digitalWrite(SCLK_1302, HIGH);
    delayMicroseconds(1);
    digitalWrite(SCLK_1302, LOW);
    delayMicroseconds(1);
  }

  // чтение (LSB first)
  pinMode(DATA_1302, INPUT);
  for (uint8_t i = 0; i < 8; i++) {
    data |= (digitalRead(DATA_1302) << i);
    digitalWrite(SCLK_1302, HIGH);
    delayMicroseconds(1);
    digitalWrite(SCLK_1302, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(RST_1302, LOW);
  pinMode(DATA_1302, OUTPUT);
  return data;
}

uint8_t translateBCD(uint8_t d) {
  return ((d & 0x70) >> 4) * 10 + (d & 0x0F);
}

// ---------------- SETUP ----------------
void setup() {
  // DS1302
  pinMode(RST_1302,  OUTPUT);
  pinMode(SCLK_1302, OUTPUT);
  pinMode(DATA_1302, OUTPUT);
  digitalWrite(RST_1302, LOW);

  // HC595 — все пины на выход
  DDRD |= (1 << PD7);                     // SCLK анодов
  DDRB |= (1 << PB0) | (1 << PB1) |       // ENTER, DATA анодов
          (1 << PB2) | (1 << PB3) |       // DATA, ENTER сеток
          (1 << PB4);                     // SCLK сеток

  // Начальные данные (всё погашено)
  for (uint8_t i = 0; i < 16; i++) {
    anodeData[i]   = 0x00;
  }

  Serial.begin(9600);
  setupTimer2();

  pressure.begin();

}

// ---------------- LOOP ----------------
void loop() {
  // Читаем время ОДИН раз за кадр (не в ISR!)
  uint8_t sec  = readData(0x81);
  uint8_t min  = readData(0x83);
  uint8_t hour = readData(0x85);
  uint8_t day = readData(0x87);
  uint8_t month = readData(0x89);
  uint8_t year = readData(0x8d);

  char status;
  double T;

  status = pressure.startTemperature();
  if (status!= 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
  }

  uint8_t s10 = translateBCD(sec)  / 10;
  uint8_t s1  = translateBCD(sec)  % 10;
  uint8_t m10 = translateBCD(min)  / 10;
  uint8_t m1  = translateBCD(min)  % 10;
  uint8_t h10 = translateBCD(hour) / 10;
  uint8_t h1  = translateBCD(hour) % 10;
  uint8_t d10 = translateBCD(day) / 10;
  uint8_t d1  = translateBCD(day) % 10;
  uint8_t mo10 = translateBCD(month) / 10;
  uint8_t mo1 = translateBCD(month) % 10;
  uint8_t T10 = int(T) / 10;
  uint8_t T1 = int(T) % 10;
  
  // Формат: HH-MM-SS, разряды 0..7
  //   0 1 : 2 3 : 4 5 : 6 7
  //   H H   -  M M   -  S S
  anodeData[0] = number7SEG[h10];
  anodeData[1] = number7SEG[h1];
    if (s1%2 == 0) anodeData[2] = number7SEG[11];   // тире
  else anodeData[2] = number7SEG[12];   // тире
  anodeData[3] = number7SEG[m10];
  anodeData[4] = number7SEG[m1];
  anodeData[5] = number7SEG[12];
  anodeData[6] = number7SEG[12];
  anodeData[7] = number7SEG[d10];
  anodeData[8] = number7SEG[d1]&number7SEG[10];
  anodeData[9] = number7SEG[mo10];
  anodeData[10] = number7SEG[mo1];
  anodeData[11] = number7SEG[12];
  anodeData[12] = number7SEG[12];
  anodeData[13] = number7SEG[T10];
  anodeData[14] = number7SEG[T1];
  anodeData[15] = number7SEG[13];
  
  delay(1);   // обновление данных 20 раз в секунду — с запасом
}

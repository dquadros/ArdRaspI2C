/*
 * Demonstração de comunicação entre Arduino e Raspberry Pi via I2C
 */

#include <Wire.h>

// Endereço do Arduino
#define I2C_ADDR 0x42

// Cor dos LEDs
// Cada LED corresponde a 3 bytes (G, R e B))
#define N_LEDS  7
static volatile uint8_t pixels [3*N_LEDS];

// Conexões
#define LED_PIN 13
#define RGB_PIN 3
#define POT_PIN A0

// Comandos recebidos pelo I2C
#define CMD_LEPOT 0
#define CMD_RGB   1
static volatile byte cmd;

void setup() {
  // LED da placa, para indicar que está vivo
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Iniciação do anel de LEDs RGB
  pinMode (RGB_PIN, OUTPUT);
  memset (pixels, 0, sizeof(pixels));
  refresh();

  // Iniciação do I2C em modo Slave
  Wire.begin(I2C_ADDR);
  Wire.onReceive(rxI2C);
  Wire.onRequest(txI2C);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(1900);
}

// Rotina chamada quando recebe algo pelo I2C
void rxI2C(int numBytes) {
  if (Wire.available() > 0) {
    cmd = Wire.read();
    if (cmd == CMD_RGB) {
      int pos = 0;
      while ((Wire.available() > 0) && (pos < sizeof(pixels))) {
        pixels[pos++] = Wire.read();
      }
      refresh();
    }
  }
}

// Rotina chamada quando tem uma solicitação de transmissão pelo I2C
void txI2C() {
  byte resp[2];
  int val;

  if (cmd == CMD_LEPOT)
  {
    // Le o potenciômetro e envia a leitura
    val = analogRead(POT_PIN);
  } else {
    val = 0;
  }
  resp[0] = val >> 8;
  resp[1] = val & 0xFF;
  Wire.write(resp, 2);
}

#define LED_PORT    PORTD
#define LED_BIT     _BV(PD3)

// Rotina para atualizar os LEDs
// 7 LEDs RGB, controlador WS2812B
// bit 0: 400nS high, 850nS low
// bit 1: 800nS high, 450nS low
// Vamos aproveitar as tolerâncias e trabalhar com 312,5/812,5
// A rotina abaixo é uma adaptação de código da biblioteca NeoPixel
// da Adafruit https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.cpp
// Garantir mínimo de 50uSeg entre chamadas
static void refresh()
{
    volatile uint16_t i = 3*N_LEDS; // Contador de bytes
    volatile uint8_t bit = 8;       // Contador de bits
    volatile uint8_t *ptr = pixels; // Ponteiro para os bytes
    volatile uint8_t  b = *ptr++;   // Byte atual
    volatile uint8_t  hi, lo;       // Programações do port
    volatile uint8_t next;          // Próxima transição
    volatile uint8_t *port = &LED_PORT;
    
    cli();      // Sem interrupções daqui para frente
    hi = *port | LED_BIT;
    lo = *port & ~LED_BIT;
    next = lo;

    // Assembly para contar os ciclos
    // A 16MHz, cada ciclo dura 62,5nS
    // Um bit (1,25uS) corresponde a 20 ciclos
    // HHHHHxxxxxxxxLLLLLLL
    // ^    ^       ^          mudanças em 0, 5 e 13 ciclos
    asm volatile(
         "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
          "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
          "sbrc %[byte],   7"        "\n\t" // 1-2  if(b & 128)
          "mov  %[next],   %[hi]"    "\n\t" // 0-1   next = hi    (T =  4)
          "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
          "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
          "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
          "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
          "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
          "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
          "nop"                      "\n\t" // 1    nop           (T = 13)
          "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
          "nop"                      "\n\t" // 1    nop           (T = 16)
          "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
          "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)

         "nextbyte20:"               "\n\t" //                    (T = 10)
          "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
          "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
          "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
          "nop"                      "\n\t" // 1    nop           (T = 16)
          "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
          "brne head20" "\n"                // 2    if(i != 0) -> (next byte)

          : [port]  "+e" (port),
            [byte]  "+r" (b),
            [bit]   "+r" (bit),
            [next]  "+r" (next),
            [count] "+w" (i)
          : [ptr]    "e" (ptr),
            [hi]     "r" (hi),
            [lo]     "r" (lo));    
    
    sei();     // Permite novamente interrupções
}



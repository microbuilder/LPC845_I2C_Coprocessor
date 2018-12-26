#include <Arduino.h>
#include <Wire.h>

#define LPC845_ADDRESS          (0x7E)

#define LPC845_OPCODE_WHOAMI    (0x01)
#define LPC845_OPCODE_VERSION   (0x02)
#define LPC845_OPCODE_ADC_HI    (0x11)
#define LPC845_OPCODE_ADC_LO    (0x12)
#define LPC845_OPCODE_ADC_STAT  (0x13)

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
void write8 (uint8_t opcode, uint32_t value)
{
  Wire.beginTransmission((byte)LPC845_ADDRESS);
  Wire.write(opcode);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t read8(uint8_t opcode, bool stop_cond = false)
{
  Wire.beginTransmission((byte)LPC845_ADDRESS);
  Wire.write(opcode);
  Wire.endTransmission(stop_cond);
  Wire.requestFrom((byte)LPC845_ADDRESS, (byte)1);
  return Wire.read();
}

/**************************************************************************/
/*!
    @brief  Configures the Arduino-compatible board for Serial output at
            115K and I2C master at the default rate (normally 100kHz).
            Also attempts to detect the presence of the LPC845 I2C
            coprocessor via a check to the WHOAMI register.
*/
/**************************************************************************/
void setup()
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }

  Serial.println("");
  Serial.println("---------------------------");
  Serial.println("LPC845 I2C Coprocessor Test");
  Serial.println("---------------------------");

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();

  /* Make sure we're actually connected */
  uint8_t x = read8(LPC845_OPCODE_WHOAMI);
  if (x != 0x45) {
    Serial.print("Unexpected response from WHOAMI: 0x");
    Serial.println(x, HEX);
    while(1) {
      /* Halt execution here. */
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
  }

  /* Check the firmware version */
  x = read8(LPC845_OPCODE_VERSION);
  Serial.print(F("Found LPC845 coprocessor (Firmware: 0x"));
  Serial.print(x, HEX);
  Serial.println(F(")"));

  /* TODO: Configure the coprocessor ADC channel. */
}

/**************************************************************************/
/*!
    @brief  This function will be called continously.
*/
/**************************************************************************/
void loop()
{
  uint8_t  adc_hi = 0;
  uint8_t  adc_lo = 0;
  uint16_t adc = 0;
  uint8_t  adc_stat = 0;

  /* Get an ADC reading from the coprocessor every second. */
  adc_hi = read8(LPC845_OPCODE_ADC_HI);
  adc_lo = read8(LPC845_OPCODE_ADC_LO);
  adc = (uint16_t)(adc_lo | (adc_hi << 8));
  adc_stat = read8(LPC845_OPCODE_ADC_STAT);

  /* Display the results */
  Serial.print("ADC: ");
  Serial.print(adc);
  Serial.print(" (CH: ");
  Serial.print((adc_stat >> 1) & 0xF);
  Serial.print(", Overrun: ");
  Serial.print(adc_stat & 0x1);
  Serial.print(")\n");

  /* Blinky, because ... blinky. */
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

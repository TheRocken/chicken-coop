#include <SPI.h>
#include <DHT.h>
#include <TSL2561.h>
#include <Adafruit_RGBLCDShield.h>
#include <Wire.h>
#include <RCSwitch.h>
//#include <Console.h>

#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>

// Listen to the default port 5555, the Yún webserver
// will forward there all the HTTP requests you send
YunServer server;

// Zustandsautomat
#define STATE_NIGHT       0
#define STATE_UP_START    1
#define STATE_UP          2
#define STATE_DAY         3
#define STATE_DOWN_START  4
#define STATE_DOWN        5

// Zustände Funkschalter
#define STATE_ON         0
#define STATE_OFF        1

#define STATE_OPEN       0
#define STATE_CLOSED     1

// Modus der Funkschalter und der Tür
#define MODE_MANUAL      0
#define MODE_AUTO        1

uint8_t stateDoor = STATE_DAY;
bool modeDoor  = MODE_AUTO;

bool stateFence = STATE_ON;
bool modeFence = MODE_AUTO;

bool stateHeater = STATE_OFF;
bool modeHeater = MODE_AUTO;
#define SWITCH_TEMP -5            // Temperatur ab der die Heizung angeschaltet wird

String lastOpening = "00:00";
String lastClosing = "00:00";

#define BLACK 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define PAGE_VIEW1 0
#define PAGE_VIEW2 1
#define PAGE_DOOR 2
#define PAGE_FENCE 3
#define PAGE_HEATER 4
uint8_t lcdPage = PAGE_VIEW1;

// Flash
const char char_flash = '\x01';
const byte char_flash_Mask[8] PROGMEM = {
  0b00010,
  0b00100,
  0b01000,
  0b11111,
  0b00010,
  0b10100,
  0b11000,
  0b11100
};

// Bulb
const char char_bulb = '\x02';
const byte char_bulb_Mask[8] PROGMEM = {
  0b01110,
  0b10001,
  0b10001,
  0b10001,
  0b01110,
  0b01110,
  0b01110,
  0b00100
};

// On left
const char char_on_left = '\x03';
const byte char_on_left_Mask[8] PROGMEM = {
  0b00000,
  0b00100,
  0b00010,
  0b00001,
  0b01110,
  0b00001,
  0b00010,
  0b00100
};

// on right
const char char_on_right = '\x04';
const byte char_on_right_Mask[8] PROGMEM = {
  0b00000,
  0b00100,
  0b01000,
  0b10000,
  0b01110,
  0b10000,
  0b01000,
  0b00100
};


// RCSwitch configuration
RCSwitch mySwitch = RCSwitch();
#define PIN_REMOTE 7

// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
TSL2561 tsl(TSL2561_ADDR_FLOAT);
uint32_t lux;                      // Aktueller Helligkeitswert
#define LUX_CHECK_TIME  10000      // Alle 10s eine Messung durchführen
unsigned long lux_last_time = 0;

unsigned long lux_limit_last_time = 0;
#define LUX_COUNT_MAX   90         // Wie oft LUX_CHECK_TIME muss die Helligkeit über-/unterschritten sein bis zum Öffnen/Schließen 90x10s = 0,25h
uint8_t lux_count;                 // Bisherige Anzahl der Über- Unterschreitungen

#define LUX_MIN  600                // Helligkeit kleiner als dieser Wert schließt die Tür
#define LUX_MAX 1000                // Helligkeit größer als dieser Wert öffnet die Tür

#define PIN_DS3234_SS 10

// Motorsteuerung
#define PIN_MOTOR_ENABLE 3
#define PIN_MOTOR_PHASE_A 5
#define PIN_MOTOR_PHASE_B 6

bool motor_reverse;

//Endlagenschalter
#define PIN_SWITCH_UP   8
#define PIN_SWITCH_DOWN 9

// Temperatursensoren
DHT dht_inside(2, DHT22);

#define WEATHER_CHECK_TIME  30000      // Alle 30s eine Messung durchfÃ¼hren
unsigned long weather_last_time = 0;

float temperature;
float humidity;

#define STATE_CHECK_TIME  100      // Alle 30s eine Messung durchfÃ¼hren
unsigned long state_last_time = 0;

void setup() {
  Bridge.begin();
 //Console.begin();
  
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);

  uint8_t character[8];
  // Add custom characters from the masks.
  memcpy_P(character, char_bulb_Mask, 8);
  lcd.createChar(char_bulb, character);
  memcpy_P(character, char_flash_Mask, 8);
  lcd.createChar(char_flash, character);
  memcpy_P(character, char_on_left_Mask, 8);
  lcd.createChar(char_on_left, character);
  memcpy_P(character, char_on_right_Mask, 8);
  lcd.createChar(char_on_right, character);


  //Endlagenschalter
  pinMode(PIN_SWITCH_UP, INPUT_PULLUP);
  pinMode(PIN_SWITCH_DOWN, INPUT_PULLUP);

  // Motor
  pinMode(PIN_MOTOR_PHASE_A, OUTPUT);
  pinMode(PIN_MOTOR_PHASE_B, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);

  RTC_init();
  //      DD.MM.YY hh:mm:ss
  //RTC_set(22, 11, 16, 11, 31, 00);

  if (tsl.begin())
  {
    // You can change the gain on the fly, to adapt to brighter/dimmer light situations
    //tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
    tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)

    // Changing the integration time gives you a longer time over which to sense light
    // longer timelines are slower, but are good in very low light situtations!
    //tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
    //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
    tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
    //uint32_t lum = tsl.getFullLuminosity();
    lux = tsl.getFullLuminosity(); //tsl.calculateLux(lum & 0xFFFF, lum >> 16);
  }


  dht_inside.begin();
  temperature = dht_inside.readTemperature();
  humidity = dht_inside.readHumidity();

  mySwitch.enableTransmit(PIN_REMOTE);
  //mySwitch.setPulseLength(24);
  mySwitch.switchOn(4, 1);                // Initial den Weidezaun aktivieren


  // Listen for incoming connection only from localhost
  // (no one from the external network could connect)
  server.listenOnLocalhost();
  server.begin();

  lcdPrint();
}
void loop() {

  // Get clients coming from server
  YunClient client = server.accept();

  // There is a new client?
  if (client) {
    // Process request
    process(client);

    // Close connection and free resources.
    client.stop();
  }

  if (millis() - state_last_time >= STATE_CHECK_TIME)
  {
    state_last_time = millis();
    StateMachine();
  }


  if (millis() - weather_last_time >= WEATHER_CHECK_TIME)
  {
    weather_last_time = millis();

    temperature = dht_inside.readTemperature();
    humidity = dht_inside.readHumidity();
    
    if (modeHeater == MODE_AUTO)
    {
      if (temperature < SWITCH_TEMP)
      {
        stateHeater = STATE_ON;
        mySwitch.switchOn(4, 2);         // Switch 1st socket from 1st group on
      }
      if (temperature > SWITCH_TEMP + 1)
      {
        stateHeater = STATE_OFF;
        mySwitch.switchOff(4, 2);        // Switch 1st socket from 1st group off
      }
    }
  }



  if (millis() - lux_last_time >= LUX_CHECK_TIME)
  {
    lux_last_time = millis();

    //uint32_t lum = tsl.getFullLuminosity();
    lux = tsl.getLuminosity(TSL2561_VISIBLE); // tsl.calculateLux(lum & 0xFFFF, lum >> 16);

    if (lcdPage == PAGE_VIEW2)
      lcdPrint();
  }


  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    if (buttons & BUTTON_UP)
    {
      switch (lcdPage)
      {
        case PAGE_VIEW1:
          lcdPage = PAGE_VIEW2;
          break;
        case PAGE_VIEW2:
          lcdPage = PAGE_VIEW1;
          break;

        case PAGE_DOOR:
          if (modeDoor == MODE_AUTO)
          {
            switch (stateDoor)
            {
              case STATE_UP_START:
              case STATE_UP:
              case STATE_DAY:
                modeDoor = MODE_MANUAL;
                stateDoor = STATE_UP_START;
                break;
              case STATE_NIGHT:
              case STATE_DOWN_START:
              case STATE_DOWN:
                stateDoor = STATE_UP_START;
                break;
            }

          }
          else
          {
            switch (stateDoor)
            {
              case STATE_UP_START:
              case STATE_UP:
              case STATE_DAY:
                stateDoor = STATE_DOWN_START;
                break;
              case STATE_NIGHT:
              case STATE_DOWN_START:
              case STATE_DOWN:
                modeDoor = MODE_AUTO;
                break;
            }
          }
          break;

        case PAGE_FENCE:
          if (modeFence == MODE_AUTO)
          {
            modeFence = MODE_MANUAL;
            stateFence = STATE_OFF;
            mySwitch.switchOff(4, 1);
          }
          else
          {
            if (stateFence == STATE_OFF)
            {
              stateFence = STATE_ON;
              mySwitch.switchOn(4, 1);
            }
            else
            {
              modeFence = MODE_AUTO;
              switch (stateDoor)
              {
                case STATE_UP_START:
                case STATE_UP:
                case STATE_DAY:
                case STATE_DOWN_START:
                case STATE_DOWN:
                  stateFence = STATE_ON;
                  mySwitch.switchOn(4, 1);
                case STATE_NIGHT:
                  stateFence == STATE_OFF;
                  mySwitch.switchOff(4, 1);
                  break;
              }

            }
          }
          break;

        case PAGE_HEATER:
          if (modeHeater == MODE_AUTO)
          {
            modeHeater = MODE_MANUAL;
            stateHeater = STATE_OFF;
            mySwitch.switchOff(4, 2);
          }
          else
          {
            if (stateHeater == STATE_OFF)
            {
              stateHeater = STATE_ON;
              mySwitch.switchOn(4, 2);
            }
            else
            {
              modeHeater = MODE_AUTO;
              if (temperature < SWITCH_TEMP)
              {
                stateHeater = STATE_ON;
                mySwitch.switchOn(4, 2);         // Switch 1st socket from 1st group on
              }
              if (temperature > SWITCH_TEMP + 1)
              {
                stateHeater = STATE_OFF;
                mySwitch.switchOff(4, 2);        // Switch 1st socket from 1st group off
              }
            }
          }
          break;
      }
      lcdPrint();
    }

    if (buttons & BUTTON_DOWN)
    {
      switch (lcdPage)
      {
        case PAGE_VIEW1:
        case PAGE_VIEW2:
          lcdPage = PAGE_DOOR;
          break;
        case PAGE_DOOR:
          lcdPage = PAGE_FENCE;
          break;
        case PAGE_FENCE:
          lcdPage = PAGE_HEATER;
          break;
        case PAGE_HEATER:
          lcdPage = PAGE_VIEW1;
          break;
      }
      lcdPrint();
    }

    if (buttons & BUTTON_LEFT)
    {
    }

    if (buttons & BUTTON_RIGHT)
    {
    }

    if (buttons & BUTTON_SELECT)
    {
    }
  }

}

void process(YunClient client) {
  // read the command
  String command = client.readStringUntil('/');

  // is "digital" command?
  if (command == F("digital")) {
    digitalCommand(client);
  }

  if (command == F("ta"))
  {
    modeDoor = MODE_MANUAL;
    stateDoor = STATE_UP_START;
  }

  if (command == F("tz"))
  {
    modeDoor = MODE_MANUAL;
    stateDoor = STATE_DOWN_START;
  }

  if (command == F("ts"))
  {
    modeDoor = MODE_AUTO;
  }

  // Weidezaun
  if (command == F("ze"))
  {
    modeFence = MODE_MANUAL;
    stateFence = STATE_ON;
    mySwitch.switchOn(4, 1);
  }

  if (command == F("za"))
  {
    modeFence = MODE_MANUAL;
    stateFence = STATE_OFF;
    mySwitch.switchOff(4, 1);
  }

  if (command == F("zs"))
  {
    modeFence = MODE_AUTO;
  }

  // Wärmelampe
  if (command == F("we"))
  {
    modeHeater = MODE_MANUAL;
    stateHeater = STATE_ON;
    mySwitch.switchOn(4, 2);
  }

  if (command == F("wa"))
  {
    modeHeater = MODE_MANUAL;
    stateHeater = STATE_OFF;
    mySwitch.switchOff(4, 2);
  }

  if (command == F("ws"))
  {
    modeHeater = MODE_AUTO;
  }

  if (command == F("page"))
  {
   //client.print(F("Welcome"));
    //WebServer(client);
  }
  WebServer(client);
}

void WebServer(YunClient client) {
  // Send feedback to client
  client.println("Status: 200");
  client.println("Content-type: text/html");
  client.println(); //mandatory blank line
  
  client.print(F("<html><head></head><body>"));

  client.print(F("<h1>H&uuml;hnerstall</h1>"));
  client.print(RTC_read());

  client.print(F("Uhr "));  client.print((int) temperature);  client.print(F("&deg;C ")); client.print((int) humidity);  client.print(F("% ")); client.print(lux); client.print(F(" Lux"));

   uint32_t lum = tsl.getFullLuminosity();
   client.print(tsl.calculateLux(lum & 0xFFFF, lum >> 16));

  client.print(F("<h3>T&uuml;r</h3>"));
  if (modeDoor == MODE_AUTO)
    client.print(F("Auto: "));
  else
    client.print(F("Manuell: "));

  switch (stateDoor)
  {
    case STATE_NIGHT:
      client.print(F("Zu"));
      break;
    case STATE_UP_START:
    case STATE_UP:
      client.print(F(">Auf"));
      break;
    case STATE_DAY:
      client.print(F("Auf"));
      break;
    case STATE_DOWN_START:
    case STATE_DOWN:
      client.print(F(">Zu"));
      break;
  }
  client.print(F("<br>"));
  client.print(F("<a href=\"./ta\">Auf</a> "));
  client.print(F("<a href=\"./tz\">Zu</a> "));
  client.print(F("<a href=\"./ts\">Auto</a><br>"));
  client.print(F("Gestern: ")); client.print(lastOpening); client.print(F("Uhr - "));  client.print(lastClosing); client.print(F("Uhr"));

  client.print(F("<h3>Zaun</h3>"));
  if (modeFence == MODE_AUTO)
    client.print(F("Auto:"));
  else
    client.print(F("Manuell:"));
  if (stateFence == STATE_ON)
    client.print(F("An"));
  else
    client.print(F("Aus"));
  client.print(F("<br>"));


  client.print(F("<a href=\"./ze\">An</a> "));
  client.print(F("<a href=\"./za\">Aus</a> "));
  client.print(F("<a href=\"./zs\">Auto</a>"));

  client.print(F("<h3>Heizung</h3>"));
  if (modeHeater == MODE_AUTO)
    client.print(F("Auto:"));
  else
    client.print(F("Manuell:"));

  if (stateHeater == STATE_ON)
    client.print(F("An"));
  else
    client.print(F("Aus"));
  client.print(F("<br>"));


  client.print(F("<a href=\"./we\">An</a> "));
  client.print(F("<a href=\"./wa\">Aus</a> "));
  client.print(F("<a href=\"./ws\">Auto</a>"));

  client.println(F("</body></html>"));

  //client.print(DELIMITER); // very important to end the communication !!!

}

void digitalCommand(YunClient client) {
  uint8_t pin, value;

  // Read pin number
  pin = client.parseInt();

  // If the next character is a '/' it means we have an URL
  // with a value like: "/digital/13/1"
  if (client.read() == '/') {
    value = client.parseInt();
    digitalWrite(pin, value);
  }

  // Send feedback to client
  client.println("Status: 200");
  client.println("Content-type: text/html");
  client.println(); //mandatory blank line
  
  client.print(F("Pin D"));
  client.print(pin);
  client.print(F(" set to "));
  client.print(value);
  //client.print(EOL);    //char terminator

}

void motorSet(boolean state, boolean reverse)
{
  if ( state)
    analogWrite(PIN_MOTOR_ENABLE, 255);
  else
    analogWrite(PIN_MOTOR_ENABLE, 0);
  digitalWrite(PIN_MOTOR_PHASE_A, !reverse);
  digitalWrite(PIN_MOTOR_PHASE_B, reverse);
}

/**
 * Gibt die im DS3234 eingestellte Zeit zurÃ¼ck
 * @return Datum und Uhrzeit im Format T.M.Y h:m:s
 */
int RTC_init() {
  pinMode(PIN_DS3234_SS, OUTPUT); // chip select
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1); // both mode 1 & 3 should work
  //set control register
  digitalWrite(PIN_DS3234_SS, LOW);
  SPI.transfer(0x8E);
  SPI.transfer(0x60); //60= disable Osciallator and Battery SQ wave @1hz, temp compensation, Alarms disabled
  digitalWrite(PIN_DS3234_SS, HIGH);
  delay(10);
}

/**
 * Stellt eine gegebene Uhrzeit und Datum im DS3234 ein
 *
 * @param d  Tag [1..31]
 * @param mo Monat [1..12]
 * @param y  Jahr [0..99]
 * @param h  Stunde [0..23]
 * @param mi Minute [0..59]
 * @param s  Sekunde [0..59]
 */
void RTC_set(int d, int mo, int y, int h, int mi, int s) {
  int TimeDate [7] = {s, mi, h, 0, d, mo, y};
  for (int i = 0; i <= 6; i++)
  {
    if (i == 3)
      i++;
    int b = TimeDate[i] / 10;
    int a = TimeDate[i] - b * 10;
    if (i == 2)
    {
      if (b == 2)
        b = B00000010;
      else if (b == 1)
        b = B00000001;
    }
    TimeDate[i] = a + (b << 4);

    digitalWrite(PIN_DS3234_SS, LOW);
    SPI.transfer(i + 0x80);
    SPI.transfer(TimeDate[i]);
    digitalWrite(PIN_DS3234_SS, HIGH);
  }
}


void lcdPrint(void)
{
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (lcdPage)
  {
    case PAGE_VIEW1:
      if (modeDoor == MODE_AUTO)
        lcd.print(F("Auto:"));
      else
        lcd.print(F("Manu:"));

      switch (stateDoor)
      {
        case STATE_NIGHT:
          lcd.print(F("Zu"));
          break;
        case STATE_UP_START:
        case STATE_UP:
          lcd.print(F(">Auf"));
          break;
        case STATE_DAY:
          lcd.print(F("Auf"));
          break;
        case STATE_DOWN_START:
        case STATE_DOWN:
          lcd.print(F(">Zu"));
          break;
      }

      if (LUX_COUNT_MAX - lux_count < 100)
      {
        lcd.print(F(" "));
        lcd.print((LUX_COUNT_MAX - lux_count)*LUX_CHECK_TIME/1000/60, DEC);
      }

      lcd.setCursor(12, 0);
      lcd.print((int) temperature); lcd.print(F("\xdf")); lcd.print(F("C"));

      lcd.setCursor(0, 1);

      if (stateFence == STATE_ON)
        lcd.print(F("\x03"));
      else
        lcd.print(F(" "));

      lcd.print(F("\x01"));

      if (stateFence == STATE_ON)
        lcd.print(F("\x04"));
      else
        lcd.print(F(" "));

      if (modeFence == MODE_AUTO)
        lcd.print(F("Auto"));
      else
        lcd.print(F("Manu"));

      lcd.setCursor(9, 1);

      if (stateHeater == STATE_ON)
        lcd.print(F("\x03"));
      else
        lcd.print(F(" "));

      lcd.print(F("\x02"));

      if (stateHeater == STATE_ON)
        lcd.print(F("\x04"));
      else
        lcd.print(F(" "));

      if (modeHeater == MODE_AUTO)
        lcd.print(F("Auto"));
      else
        lcd.print(F("Manu"));

      break;

    case PAGE_VIEW2:
      lcd.print((uint16_t)lux); lcd.print(F("Lux"));     // Helligkeitswert auf 16Bit begrenzen
      lcd.setCursor(8, 0);
      lcd.print((int) temperature); lcd.print(F("\xdf")); lcd.print(F("C"));
      lcd.setCursor(13, 0);
      lcd.print((int) humidity);  lcd.print(F("%"));
      lcd.setCursor(0, 1);
      lcd.print(F("Auf:")); lcd.print(lastOpening); lcd.print(F("-")); lcd.print(lastClosing);
      break;

    case PAGE_DOOR:
      lcd.print(F("T\xF5reinstellung"));
      lcd.setCursor(0, 1);

      if (modeDoor == MODE_AUTO)
        lcd.print(F("Auto"));
      else
        lcd.print(F("Manuell"));
      lcd.print(F(" / "));
      switch (stateDoor)
      {
        case STATE_NIGHT:
          lcd.print(F("Zu"));
          break;
        case STATE_UP_START:
        case STATE_UP:
          lcd.print(F(">Auf"));
          break;
        case STATE_DAY:
          lcd.print(F("Auf"));
          break;
        case STATE_DOWN_START:
        case STATE_DOWN:
          lcd.print(F(">Zu"));
          break;
      }
      break;

    case PAGE_FENCE:
      lcd.print(F("\x01Zauneinstellung"));
      lcd.setCursor(0, 1);

      if (modeFence == MODE_AUTO)
        lcd.print(F("Auto"));
      else
        lcd.print(F("Manuell"));
      lcd.print(F(" / "));
      if (stateFence == STATE_ON)
        lcd.print(F("An"));
      else
        lcd.print(F("Aus"));
      break;

    case PAGE_HEATER:
      lcd.print(F("\x02Heizeinstellung"));
      lcd.setCursor(0, 1);

      if (modeHeater == MODE_AUTO)
        lcd.print(F("Auto"));
      else
        lcd.print(F("Manuell"));
      lcd.print(F(" / "));
      if (stateHeater == STATE_ON)
        lcd.print(F("An"));
      else
        lcd.print(F("Aus"));
      break;
  }

}


/**
 * Gibt die im DS3234 eingestellte Zeit zurÃ¼ck
 * @return Datum und Uhrzeit im Format T.M.Y h:m:s
 */
String RTC_read() {
  String temp;
  int TimeDate [7];                           // [Sekunde, Minute, Stunde, NULL, Tag, monat, Jahr]
  for (int i = 0; i <= 6; i++)
  {
    if (i == 3)
      i++;
    digitalWrite(PIN_DS3234_SS, LOW);
    SPI.transfer(i + 0x00);
    unsigned int n = SPI.transfer(0x00);
    digitalWrite(PIN_DS3234_SS, HIGH);
    int a = n & B00001111;
    if (i == 2)
    {
      int b = (n & B00110000) >> 4; //24 hour mode
      if (b == B00000010)
        b = 20;
      else if (b == B00000001)
        b = 10;
      TimeDate[i] = a + b;
    }
    else if (i == 4)
    {
      int b = (n & B00110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 5)
    {
      int b = (n & B00010000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 6)
    {
      int b = (n & B11110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else
    {
      int b = (n & B01110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
  }
  //temp.concat(TimeDate[4]);
  //temp.concat(F("."));
  //temp.concat(TimeDate[5]);
  //temp.concat(F("."));
  //temp.concat(TimeDate[6]);
  //temp.concat(F(" "));
  if (TimeDate[2] < 10)
    temp.concat(F("0"));
  temp.concat(TimeDate[2]);
  temp.concat(F(":"));
  if (TimeDate[1] < 10)
    temp.concat(F("0"));
  temp.concat(TimeDate[1]);
  //temp.concat(F(":"));
  //temp.concat(TimeDate[0]);
  return (temp);
}

void StateMachine()
{
  switch (stateDoor)
  {
    case STATE_NIGHT:
      if (modeDoor == MODE_AUTO)
      {
        if (millis() - lux_limit_last_time >= LUX_CHECK_TIME)
        {
          lux_limit_last_time = millis();

          if (lux > LUX_MAX)
          {
            lux_count++;
          }
          else
          {
            lux_count = 0;
          }

          if (lux_count > LUX_COUNT_MAX)
          {
            lux_count = 0;
            stateDoor = STATE_UP_START;
            lcd.setBacklight(YELLOW);
            lcdPrint();
          }
        }
      }
      break;

    case STATE_UP_START:
      motorSet(true, false);
      stateDoor = STATE_UP;
      lastOpening = RTC_read();
      if (modeFence == MODE_AUTO)
      {
        stateFence = STATE_ON;
        mySwitch.switchOn(4, 1);
      }
      break;

    case STATE_UP:
      if (digitalRead(PIN_SWITCH_UP))
      {
        motorSet(false, false);
        stateDoor = STATE_DAY;
        lcd.setBacklight(GREEN);
        lcdPrint();
      }
      break;

    case STATE_DAY:
      if (modeDoor == MODE_AUTO)
      {
        if (millis() - lux_limit_last_time >= LUX_CHECK_TIME)
        {
          lux_limit_last_time = millis();

          if (lux < LUX_MIN)
          {
            lux_count++;
          }
          else
          {
            lux_count = 0;
          }

          if (lux_count > LUX_COUNT_MAX)
          {
            lux_count = 0;
            stateDoor = STATE_DOWN_START;
            lcd.setBacklight(YELLOW);
            lcdPrint();
          }
        }
      }
      break;

    case STATE_DOWN_START:
      motorSet(true, true);
      stateDoor = STATE_DOWN;
      lcd.setBacklight(YELLOW);
      lastClosing = RTC_read();
      if (modeFence == MODE_AUTO)
      {
        stateFence = STATE_OFF;
        mySwitch.switchOff(4, 1);
      }
      break;

    case STATE_DOWN:
      if (digitalRead(PIN_SWITCH_DOWN))
      {
        motorSet(false, false);
        stateDoor = STATE_NIGHT;
        lcd.setBacklight(GREEN);
        lcdPrint();
      }
      break;

  }
}

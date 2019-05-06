#include <SPI.h>
#include <Chrono.h>

//#define DEBUG
//#define DEBUG_TX

#ifdef DEBUG_BUTTONS
#define DEBUG
#endif

#ifdef DEBUG_TX
#define DEBUG
#endif

#ifdef DEBUG_MODE
#define DEBUG
#endif

// digital potentiometer
const int PIN_POT_14 = 0;
const int PIN_POT_23 = 10;
const int PIN_POT_67 = 22;

// leds
// gears
const int PIN_LED_R = 3;
const int PIN_LED_P = 4;
const int PIN_LED_D = 5;

// state
const int PIN_LED_GREEN = 20;
const int PIN_LED_RED = 21;

const int PWM_LED = 64;

// joystick
const int PIN_J1_X = 17;
const int PIN_J1_Y = 18;
const int PIN_J1_S = 19;
const int PIN_J2_X = 15;
const int PIN_J2_Y = 14;
const int PIN_J2_S = 16;

// modes
const int MODE_4CH = 0;
const int MODE_2CH_LIGHTS = 1;
const int MODE_2CH_REALISTIC_LIGHTS = 2;
const int MODE_MAX = MODE_2CH_REALISTIC_LIGHTS;
int mode = MODE_2CH_LIGHTS;// as an alternative: MODE_2CH_REALISTIC_LIGHTS;

// voltage check
const int PIN_9V_PROBE = 23;

// potentiometer stuff
const int POT_COMMAND_WRITE = 0b00010000;
const int POT_COMMAND_A = 0b00000001;
const int POT_COMMAND_B = 0b00000010;
const int POT_COMMAND_WRITE_A = POT_COMMAND_WRITE | POT_COMMAND_A;
const int POT_COMMAND_WRITE_B = POT_COMMAND_WRITE | POT_COMMAND_B;
const int POT_COMMAND_WRITE_BOTH = POT_COMMAND_WRITE_A | POT_COMMAND_WRITE_B;

const SPISettings SPIStandard = SPISettings(10000000, MSBFIRST, SPI_MODE0);

const int POT_POS_NULL = 0x80;

const int TX_CH1 = 0b00000001;
const int TX_CH2 = 0b00000010;
const int TX_CH3 = 0b00000100;
const int TX_CH4 = 0b00001000;
const int TX_CH6 = 0b00100000;
const int TX_CH7 = 0b01000000;
const int TX_CHALL = TX_CH1 | TX_CH2 | TX_CH3 | TX_CH4 | TX_CH6 | TX_CH7;

const int CH1 = 0;
const int CH2 = 1;
const int CH3 = 2;
const int CH4 = 3;
const int CH6 = 5;
const int CH7 = 6;

const int CH_J1_X = 0;
const int CH_J1_Y = 1;
const int CH_J2_X = 2;
const int CH_J2_Y = 3;

const int JOYSTICK_CH_PINS[4] = {PIN_J1_X, PIN_J1_Y, PIN_J2_X, PIN_J2_Y};

const unsigned int JOYSTICK_MIN = 0;
const unsigned int JOYSTICK_MAX = 1024;

byte channel[7] = {POT_POS_NULL, POT_POS_NULL, POT_POS_NULL, POT_POS_NULL, POT_POS_NULL, POT_POS_NULL, POT_POS_NULL};
int joystick_center[4] = {JOYSTICK_MAX / 2, JOYSTICK_MAX / 2, JOYSTICK_MAX / 2, JOYSTICK_MAX / 2};

#define DEADZONE_LOWER(x) (x & 0x0000FFFF)
#define DEADZONE_UPPER(x) ((x & 0xFFFF0000) >> 16)

unsigned int joystick_deadzone[4] = {(JOYSTICK_MAX / 2 << 16) + (JOYSTICK_MAX / 2), (JOYSTICK_MAX / 2 << 16) + (JOYSTICK_MAX / 2), (JOYSTICK_MAX / 2 << 16) + (JOYSTICK_MAX / 2), (JOYSTICK_MAX / 2 << 16) + (JOYSTICK_MAX / 2)};

#define RANGE_MAX(x) ((byte)((x & 0x00FF0000) >> 16))
#define RANGE_CENTER(x) ((byte)((x & 0x0000FF00) >> 8))
#define RANGE_MIN(x) ((byte)(x & 0x000000FF))
unsigned int channel_range[4] = { 0x00FF8000, 0x00FF8000, 0x00A4805C, 0x00FF8000}; // modify challen_range[3] according to your steering needs

int joystick_invert[4] = {0, JOYSTICK_MAX, JOYSTICK_MAX, JOYSTICK_MAX};
byte channel_trim[7] = {0, 0, 0, 0, 0, 0, 0};

// buttons
const int PIN_BTN_LIGHT = 8;
const int PIN_BTN_BLINK_LEFT = 9;
const int PIN_BTN_BLINK_RIGHT = 7;
const int PIN_BTN_HAZARD = 6;

const int BUTTON_J1_S = 0;
const int BUTTON_J2_S = 1;
const int BUTTON_BLINK_LEFT = 2;
const int BUTTON_HEADLIGHTS = 3;
const int BUTTON_HAZARD = 4;
const int BUTTON_BLINK_RIGHT = 5;

const int BUTTON_PINS[6] = {PIN_J1_S, PIN_J2_S, PIN_BTN_BLINK_LEFT, PIN_BTN_LIGHT, PIN_BTN_HAZARD, PIN_BTN_BLINK_RIGHT};

const byte BUTTON_NO_CHANGE = 0;
const byte BUTTON_PRESSED = 1;
const byte BUTTON_RELEASED = 2;

int button_state[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
int button_state_last[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
byte button_state_changed[6] = {BUTTON_NO_CHANGE, BUTTON_NO_CHANGE, BUTTON_NO_CHANGE, BUTTON_NO_CHANGE, BUTTON_NO_CHANGE, BUTTON_NO_CHANGE};

// Metros
Chrono  chronoCheckBattery9V = Chrono(Chrono::MILLIS);
Chrono  chronoResetCH1 = Chrono(Chrono::MILLIS);
Chrono  chronoResetCH4 = Chrono(Chrono::MILLIS);

bool txCH1 = false;
bool txCH4 = false;

byte LED_GREEN = 0;
byte LED_RED = 0;

void initPins()
{
  // analog in
  pinMode(PIN_J1_X, INPUT);
  pinMode(PIN_J1_Y, INPUT);
  pinMode(PIN_J2_X, INPUT);
  pinMode(PIN_J2_Y, INPUT);
  pinMode(PIN_9V_PROBE, INPUT);

  // analog out
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_P, OUTPUT);
  pinMode(PIN_LED_D, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  // digital in pullup
  pinMode(PIN_BTN_LIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_BLINK_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_BLINK_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_HAZARD, INPUT_PULLUP);
  pinMode(PIN_J1_S, INPUT_PULLUP);
  pinMode(PIN_J2_S, INPUT_PULLUP);

  // digital out
  pinMode(PIN_POT_14, OUTPUT);
  pinMode(PIN_POT_23, OUTPUT);
  pinMode(PIN_POT_67, OUTPUT);

  // chip deselect potentiometers
  digitalWrite(PIN_POT_14, HIGH);
  digitalWrite(PIN_POT_23, HIGH);
  digitalWrite(PIN_POT_67, HIGH);
}

void analogReadJoystickDeadzone(int ch)
{
  int lower = 0xFFFFFFFF, upper = 0;
  for(int i = 0; i < 50; ++i)
  {
    int read = analogReadJoystick(ch);
    if(read < lower) lower = read;
    if(read > upper) upper = read;
    delay(20);
  }
  joystick_deadzone[ch] = (upper << 8) + lower;
}

void analogReadJoystickDeadzones()
{
  unsigned int lower[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
  unsigned int upper[4] = {0, 0, 0, 0};

  delay(1000);
  #ifdef DEBUG_INPUT_PINS
  Serial.printf("ch |  val |  min |  max\n");
  #endif
  
  for(int i = 0; i < 4*50; ++i)
  {
    int ch = i % 4;
    int read = analogReadJoystick(ch);
    if(read < lower[ch]) lower[ch] = read;
    if(read > upper[ch]) upper[ch] = read;
    delay(20);

    #ifdef DEBUG_INPUT_PINS
    Serial.printf("%2d | %4d | %4d | %4d\n", ch, read, lower[ch], upper[ch]);
    #endif
  }

  for(int ch = 0; ch < 4; ++ch)
  {
    joystick_deadzone[ch] = (upper[ch] << 16) + lower[ch];
    #ifdef DEBUG_INPUT_PINS
    Serial.printf("%2d | ---- | %4d | %4d | %#010x | %4d | %4d \n", ch, lower[ch], upper[ch], joystick_deadzone[ch], DEADZONE_LOWER(joystick_deadzone[ch]),  DEADZONE_UPPER(joystick_deadzone[ch]));
    #endif
  }
}

void update_LEDs(byte d = 0, byte p = 0, byte r = 0)
{
  analogWrite(PIN_LED_GREEN, LED_GREEN);
  analogWrite(PIN_LED_RED, LED_RED);
  analogWrite(PIN_LED_D, d);
  analogWrite(PIN_LED_P, p);
  analogWrite(PIN_LED_R, r);
}

void center_joysticks()
{
  byte g = LED_GREEN;
  byte r = LED_RED;

  LED_GREEN = 255; LED_RED = 255; update_LEDs();

  analogReadJoystickDeadzones();

  LED_GREEN = g; LED_RED = r; update_LEDs();
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  
  // starting the engine
  analogWrite(PIN_LED_GREEN, 32);
  analogWrite(PIN_LED_RED, 64);
  // put your setup code here, to run once:
  SPI.begin();

  initPins();

  // center all potis
  tx(TX_CHALL);

  // learn joystick center
  center_joysticks();

  // done
  analogWrite(PIN_LED_GREEN, 48);
  analogWrite(PIN_LED_RED, 0);

  LED_RED = 0;
  LED_GREEN = 48;
}

int checkBattery9V(bool ignoreTimer = false)
{
  /*
     V+ --- 1M --- 470k --- 470k --- GND
                         |
                    PIN_9V_PROBE

     9.4V * 470k / (1M + 470k + 470k) = 2.28V
     7.2V * 470k / (1M + 470k + 470k) = 1.74V

     VRef = 3.3V
     2.28 / 3.3 * 1024 - 1: 706
     1.74 / 3.3 * 1024 - 1: 539
  */

  int probe = 0;
  if (ignoreTimer || chronoCheckBattery9V.hasPassed(5000, true))
  {
    probe = analogRead(PIN_9V_PROBE);
    if (probe < 500)
      analogWrite(PIN_LED_RED, 128);
  }
  return 13621 * (probe + 1) / 1024;
}

int analogReadJoystick(int ch)
{
  return abs(joystick_invert[ch] - analogRead(JOYSTICK_CH_PINS[ch]));
}

int analogToChannel(int ch, int range_max, int range_center, int range_min)
{
  int in = analogReadJoystick(ch);
  int center = joystick_center[ch];

  int trim = channel_trim[ch];

  int dz_lower = DEADZONE_LOWER(joystick_deadzone[ch]);
  int dz_upper = DEADZONE_UPPER(joystick_deadzone[ch]);

  int out = range_center;

  if(in > dz_upper)
    out = map(in, dz_upper, JOYSTICK_MAX, range_center, range_max);
  else if(in < dz_lower)
    out = map(in, JOYSTICK_MIN, dz_lower, range_min, range_center);
  else
    out = range_center;

  #ifdef DEBUG_INPUT_PINS
  Serial.printf("ch %2d in: %4d out: %3d dzl: %3d dzu: %3d rmin: %3d rcenter: %3d rmax: %3d\n", ch, in, out, dz_lower, dz_upper, range_min, range_center, range_max);
  #endif

  return out;
}

int analogToChannel(int ch)
{
  return analogToChannel(ch, RANGE_MAX(channel_range[ch]), RANGE_CENTER(channel_range[ch]), RANGE_MIN(channel_range[ch]));
}

void steering()
{
  channel[CH3] = analogToChannel(CH3);
}

void mode_2ch_lights()
{
  /*
     J1Y: motor     ==> CH2
     J2X: steering  ==> CH3
  */

  channel[CH2] = analogToChannel(CH2);
  steering();
  lights();

  bool d = channel[CH2] > RANGE_CENTER(channel_range[CH2]) + 2;
  bool r = channel[CH2] < RANGE_CENTER(channel_range[CH2]) - 2;

  LED_GREEN = 24;
  update_LEDs(d ? 64 : 0,
              !d && !r ? 64 : 0,
              r ? 64 : 0);

  tx((txCH1 ? TX_CH1 : 0) | TX_CH2 | TX_CH3 | (txCH4 ? TX_CH4 : 0));
  txCH1 = false;
  txCH4 = false;
}

void mode_4ch()
{
  /*
     J1Y: motor     ==> CH2
     J1X:           ==> CH1
     J2X: steering  ==> CH3
     J2Y:           ==> CH4
  */

  channel[CH1] = analogToChannel(CH1);
  channel[CH2] = analogToChannel(CH2);
  channel[CH3] = analogToChannel(CH3);
  channel[CH4] = analogToChannel(CH4);
 
  tx(TX_CH1 | TX_CH2 | TX_CH3 | TX_CH4);
}

unsigned int inertia = 0;
const float wheel_gear_ratio = 5.57f * 1.1f;
const unsigned int motor_torque = 1300.f * wheel_gear_ratio; // Nm
const unsigned int brake_torque = -20000; //-13000; // Nm
const unsigned int mass = 8100; // kg
const unsigned int force_weight = (unsigned int)((float)mass * 9.81f); //N
const unsigned int motor_power = 240000; // watt
const signed int force_roll_resistance = -0.0060f * force_weight; // N on concrete or gravel
const float c_drag = -.6;

float speed = 0;
unsigned long last_update = 0;

const int MAX_FWD_SPEED = 80;
const int MAX_RWD_SPEED = 15;
const int MAX_FWD_CH2 = 255;
const int MAX_RWD_CH2 = 104; // 128 - 128 * 15/80

const int GEAR_P =  0;
const int GEAR_D =  1;
const int GEAR_R = -1;
int gear = GEAR_D;

void mode_2ch_realistic_lights()
{
  /*
     J1Y+: motor
     J1Y-: brake
     J2X: steering  ==> CH2
  */

  if(button_state_changed[BUTTON_J1_S] == BUTTON_PRESSED)
    --gear;
  if(button_state_changed[BUTTON_J2_S] == BUTTON_PRESSED)
    ++gear;

  gear = min(max(gear, -1), 1);

  update_LEDs(gear == GEAR_D ? 64 : 0,
              gear == GEAR_P ? 64 : 0,
              gear == GEAR_R ? 64 : 0);

  steering();

  if(gear != GEAR_P)
  {
    int middle = 0;
    int j1y = analogToChannel(CH2, 512, 0, -512);
  
    int force_traction = 0;
    const int d = 2;
    bool breaking = false;
    if(j1y > middle + d)
      force_traction = map(j1y, middle+d, 512, 0, motor_torque);
    else if (j1y < middle - d)
    {
      breaking = true;
      force_traction = map(j1y, middle-d, -512, 0, brake_torque);
    }

    int force_drag = c_drag * speed * speed;
  
    int force = force_traction + force_drag + force_roll_resistance;    
    float acceleration = (float)force/(float)mass;
  
    unsigned long now = micros();
    float dt = (float)(now - last_update) / 100000.0f;
  
    last_update = now;
    
    speed += dt * acceleration;

    if(breaking)
      speed = max((unsigned int)0, speed);
    
    if(speed < 0)
      speed = 0;
    int speedInt = (int)speed;

    if(gear == GEAR_D)
    {
      speed = min(speed, MAX_FWD_SPEED);
      channel[CH2] = map(speedInt, 0, MAX_FWD_SPEED, 128, MAX_FWD_CH2);
    }
    else
    {
      speed = min(speed, MAX_RWD_SPEED);
      channel[CH2] = map(speedInt, 0, MAX_RWD_SPEED, 128, MAX_RWD_CH2);
    }

    #ifdef DEBUG_MODE
    Serial.printf("j1y: %3d traction: %5d force: %8d acc: %8.4f dt: %8.4f speed: %8.4f CH : %4d\n", j1y, force_traction, force, acceleration, dt, gear*speed, channel[CH2]);
    #endif
  }
  else
  {
    speed = 0;
    channel[CH2] = POT_POS_NULL;
  }

  steering();
  lights();

  tx((txCH1 ? TX_CH1 : 0) | TX_CH2 | TX_CH3 | (txCH4 ? TX_CH4 : 0));
  txCH1 = false;
  txCH4 = false;
}

int ch1ResetExpected = 1000;
int ch4ResetExpected = 2500;

void lights()
{
  if(button_state[BUTTON_HEADLIGHTS] == LOW && button_state_changed[BUTTON_HEADLIGHTS] == BUTTON_PRESSED)
  {
    channel[CH4] = 224;
    ch4ResetExpected = 2500;
    chronoResetCH4.restart(0);
    txCH4 = true;
  }
  if(button_state[BUTTON_BLINK_LEFT] == LOW && button_state_changed[BUTTON_BLINK_LEFT] == BUTTON_PRESSED)
  {
    channel[CH1] = 224;
    ch1ResetExpected = 300;
    chronoResetCH1.restart(0);
    txCH1 = true;
  }
  if(button_state[BUTTON_BLINK_RIGHT] == LOW && button_state_changed[BUTTON_BLINK_RIGHT] == BUTTON_PRESSED)
  {
    channel[CH1] = 32;
    ch1ResetExpected = 300;
    chronoResetCH1.restart(0);
    txCH1 = true;
  }
  if(button_state[BUTTON_HAZARD] == LOW && button_state_changed[BUTTON_HAZARD] == BUTTON_PRESSED)
  {
    channel[CH1] = 224;
    ch1ResetExpected = 2500;
    chronoResetCH1.restart(0);
    txCH1 = true;
  }

  if (!txCH1 && chronoResetCH1.isRunning() && (txCH1 = chronoResetCH1.hasPassed(ch1ResetExpected)))
  {
    channel[CH1] = POT_POS_NULL;
    chronoResetCH1.stop();
  }
  if (!txCH4 && chronoResetCH4.isRunning() && (txCH4 = chronoResetCH4.hasPassed(ch4ResetExpected)))
  {
    channel[CH4] = POT_POS_NULL;
    chronoResetCH4.stop();
  }
}

void spiWritePot(int pin, byte command, byte value)
{
  digitalWrite(pin, LOW);
  SPI.transfer(command);
  SPI.transfer(value);
  digitalWrite(pin, HIGH);
}

void tx(int what)
{
  SPI.beginTransaction(SPIStandard);
  if (what & TX_CH1) spiWritePot(PIN_POT_14, POT_COMMAND_WRITE_A, channel[CH1]);
  if (what & TX_CH2) spiWritePot(PIN_POT_23, POT_COMMAND_WRITE_A, channel[CH2]);
  if (what & TX_CH3) spiWritePot(PIN_POT_23, POT_COMMAND_WRITE_B, channel[CH3]);
  if (what & TX_CH4) spiWritePot(PIN_POT_14, POT_COMMAND_WRITE_B, channel[CH4]);
  //  if(what & CH5) spiWritePot(  UNUSED  ,       UNUSED       ,    UNUSED   );
  if (what & TX_CH6) spiWritePot(PIN_POT_67, POT_COMMAND_WRITE_A, channel[CH6]);
  if (what & TX_CH7) spiWritePot(PIN_POT_67, POT_COMMAND_WRITE_B, channel[CH7]);
  SPI.endTransaction();

  #ifdef DEBUG_TX
  Serial.printf("# | value\n");
  if (what & TX_CH1) Serial.printf("1 | %3d\n", channel[CH1]);
  if (what & TX_CH2) Serial.printf("2 | %3d\n", channel[CH2]);
  if (what & TX_CH3) Serial.printf("3 | %3d\n", channel[CH3]);
  if (what & TX_CH4) Serial.printf("4 | %3d\n", channel[CH4]);
  if (what & TX_CH6) Serial.printf("6 | %3d\n", channel[CH6]);
  if (what & TX_CH7) Serial.printf("7 | %3d\n", channel[CH7]);
  #endif
}

void update_buttons()
{
  #ifdef DEBUG_BUTTONS
  Serial.printf("# | pin | state | change\n");
  #endif
  for(int i = 0; i < 6; ++i)
  {
    button_state_last[i] = button_state[i];
    button_state[i] = digitalRead(BUTTON_PINS[i]);

    if(button_state_last[i] == button_state[i])
      button_state_changed[i] = BUTTON_NO_CHANGE;
    else if (button_state[i] == HIGH)
      button_state_changed[i] = BUTTON_RELEASED;
    else
      button_state_changed[i] = BUTTON_PRESSED;

    #ifdef DEBUG_BUTTONS
    Serial.printf("%1d | %3d | %s | %s\n", i, BUTTON_PINS[i], button_state[i] == LOW ? "  low" : " high", button_state_changed[i] == BUTTON_NO_CHANGE ? "no chg" : (button_state_changed[i] == BUTTON_PRESSED ? " press" : "  rlsd"));
    #endif
  }
}

void halt()
{
  chronoResetCH1.stop();
  chronoResetCH4.stop();

  for(int c = 0; c < CH7; c++)
    channel[c] = POT_POS_NULL;
  
  tx(TX_CHALL);
}

bool check_menu_enter_exit()
{
  return button_state[BUTTON_BLINK_LEFT] == LOW && button_state_changed[BUTTON_BLINK_LEFT] == BUTTON_PRESSED &&
         button_state[BUTTON_BLINK_RIGHT] == LOW && button_state_changed[BUTTON_BLINK_RIGHT] == BUTTON_PRESSED;
}

void mode_change()
{
  if(check_menu_enter_exit())
  {
    byte g = LED_GREEN;
    byte r = LED_RED;
    halt();

    Chrono blink = Chrono(Chrono::MILLIS);

    const int low = 16;
    const int high = 255-16;
    const int toggle = low ^ high;

    LED_GREEN = 255; LED_RED = 255;
    update_LEDs();
    do
    {
      update_buttons();
    }
    while(button_state_changed[BUTTON_BLINK_LEFT] != BUTTON_RELEASED && button_state_changed[BUTTON_BLINK_RIGHT] != BUTTON_RELEASED);

    LED_GREEN = low; LED_RED = low;
    update_LEDs();

    bool breakFromMenu = false;
    while(!check_menu_enter_exit())
    {
      bool LEDtoggled = blink.hasPassed(1000, true);
      if(LEDtoggled)
      {
        LED_RED ^= toggle;
        LED_GREEN ^= toggle;
      //  update_LEDs();
      }

      int oldMode = mode;

      if(button_state[BUTTON_J1_S] == LOW && button_state_changed[BUTTON_J1_S] == BUTTON_PRESSED)
        mode++;

      if(button_state[BUTTON_J2_S] == LOW && button_state_changed[BUTTON_J2_S] == BUTTON_PRESSED)
        mode--;

      if(mode < 0)
        mode = 0;
      if(mode > MODE_MAX)
        mode = MODE_MAX;

      if(mode != oldMode || LEDtoggled)
        update_LEDs(mode == MODE_4CH ? 128 : 0, mode == MODE_2CH_LIGHTS ? 128 : 0, mode == MODE_2CH_REALISTIC_LIGHTS ? 128 : 0);

      delay(50);
      update_buttons();
      breakFromMenu =check_menu_enter_exit();
    }
     
    delay(50);
    update_buttons();

    LED_GREEN = g;
    LED_RED = r;
    update_LEDs();
  }
}
//*/
#ifdef DEBUG
int serial_read_int()
{
  int number = 0;
  byte n = 0;
  char c = Serial.peek();
  bool negative = (c == '-');

  if(negative)
    Serial.read();

  while(n < 4)
  {
    c = Serial.read();
    if(c == '\r' || c == '\n')
      break;
    number = number * 10 + ('0' - c);
    ++n;
  }
  return (negative ? -number : number);
}

void serial()
{
  static bool continuousPrint = false;
  if (Serial.available() > 0)
  {
    int command = Serial.peek();
    if(command == 'h')
    {
      Serial.read();
      Serial.println("commands:\np print channel info\nt $1 $2 trim channel $1 for $2 steps\n");
    }
    // save trim
    // s
    else if(command == 's')
    {
      Serial.read();
    }
    else if(command == 'c')
    {
      Serial.read();
      continuousPrint = !continuousPrint;
      Serial.printf("Battery: ~ %d mV\n", checkBattery9V(true));
      Serial.println("CH | PIN | value | raw | center | trim\n");
    }
    // print channel infos
    // p
    else if(command == 'p')
    {
      Serial.read();
      /*
       * CH | PIN | value | center | trim
       * 
       */
      //while(Serial.peek() != 13);
      // print
      Serial.printf("Battery: ~ %d mV\n", checkBattery9V(true));
      Serial.println("CH | PIN | value | raw | center | trim\n");
      for(int i = 0; i < 4; ++i)
        Serial.printf("%2d | %3d | %5d | %6d | %6d | %4d\n", i + 1, JOYSTICK_CH_PINS[i], channel[i], analogReadJoystick(i), joystick_center[i], channel_trim[i]);
    }
    // trim channel3 for -3 steps:
    // t 2 -3
    else if(command == 't')
    {
      Serial.read();
      Serial.print("trim\nchannel [1..4]: ");
      int ch = '1' - Serial.read();
      if(ch == 0 || ch == 1 || ch == 2 || ch == 3)
      {
        Serial.printf("\ncurrent trim: %4d\nnew trim: ", channel_trim[ch]);
        channel_trim[ch] = serial_read_int();
      }
      else
        Serial.printf("\nunknown channel %2d\n", ch);
    }
    /* unfinished:
    // send value 73 on channel 3:
    // v 3 73
    else if(command == 'v')
    {
      Serial.read();
      int ch = CH1;//serial_read_int();
    //  int v = serial_read_int();

      channel[ch] = 160;
      tx(TX_CH1 );
      delay(1000);
      channel[ch] = 40;
      tx(TX_CH1 );
      delay(1000);
      channel[ch] = POT_POS_NULL;
      tx(TX_CH1 );
      Serial.print("done");
    }
    */
    Serial.print("\n");
  }

  if(continuousPrint)
  {
    for(int i = 0; i < 4; ++i)
      Serial.printf("%2d | %3d | %5d | %6d | %6d | %4d\n", i + 1, JOYSTICK_CH_PINS[i], channel[i], analogReadJoystick(i), joystick_center[i], channel_trim[i]);
    Serial.print("\n");
  }
}
#endif

void loop() {
  int start = millis();

  mode_change();
  update_buttons();
  
  switch(mode)
  {
    case MODE_2CH_LIGHTS:
      mode_2ch_lights();
      break;
    case MODE_2CH_REALISTIC_LIGHTS:
      mode_2ch_realistic_lights();
      break;
    default:
    case MODE_4CH:
      mode_4ch();
      break;
  }

  checkBattery9V();

  #ifdef DEBUG
  serial();
  #endif

  int end = millis() - start;
  if(end > 0 && end <= 25)
    delay(25 - end);
}

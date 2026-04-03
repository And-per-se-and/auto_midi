#include <Encoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Audio.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define LED_DATA 22
#define LED_CLOCK 24
#define LED_LATCH 25

#define BUTTON_DATA 26
#define BUTTON_CLOCK 16
#define BUTTON_LOAD 17

#define LED_1 3
#define LED_2 4
#define LED_3 5
#define LED_4 6
#define LED_5 7
#define LED_6 0
#define LED_7 1
#define LED_8 2

#define BTN_LOW  0
#define BTN_MID  1
#define BTN_HI   2
#define BTN_1    3
#define BTN_2    4
#define BTN_3    5
#define BTN_4    6
#define BTN_5    7
#define BTN_6    8
#define BTN_7    9
#define BTN_8    10
#define BTN_MENU 11

#define DEBOUNCE_MS 5

#define GRAPH_Y 26
#define GRAPH_H 12

Encoder enc0(30, 31) ;
Encoder enc1(32, 33) ;
Encoder enc2(34, 35) ;
Encoder enc3(36, 37) ;
Encoder enc4(38, 39) ;
Encoder enc5(40, 41) ;
Encoder enc6(42, 43) ;
Encoder enc7(44, 45) ;
Encoder enc8(46, 47) ;

unsigned long lastFrameTime = 0 ;
float currentFPS = 0 ;
float smoothedFPS = 0 ;
#define FPS_SMOOTH 0.95f

Encoder* encoders[9] = { &enc0, &enc1, &enc2, &enc3, &enc4, &enc5, &enc6, &enc7, &enc8 } ;
int encoder_speed = 2 ;
long lastEncPos[18] = {0} ;
int  function_value[18]     { 0, 64, 63, 63, 63, 63, 63, 63, 63,   63, 63, 0, 0, 0, 63, 63, 0, 63 } ;
int  function_reset[18]     { 0, 64, 63, 63, 63, 63, 63, 63, 63,   63, 63, 0, 0, 0, 63, 63, 0, 63 } ;
int  function_direction[18] { 1, 1, 1, 1, -1, -1, 1, -1, 1,        1, 1, 1, 1, 1, 1, 1, 1, 1 } ;
bool function_graphType[18] { 0, 1, 1, 1, 1, 1, 1, 1, 1,           1, 1, 0, 0, 0, 1, 1, 0, 1 } ;
int  cc_encoder[18]         { 16, 17, 18, 19, 120, 121, 122, 123, 0,   20, 21, 22, 23, 124, 125, 126, 127, 0 } ;

bool booting = true ;
bool booted = false ;
int boot_time_seconds = 5 ;
bool scroll_LEDs = true ;
bool done = false ;

uint16_t stableButtons = 0x0FFF ;
uint16_t lastRawButton = 0x0FFF ;
unsigned long lastChangeTime = 0 ;
uint16_t lastButtons = 0x0FFF ;
uint16_t held = 0 ;
uint16_t buttons = 0x0FFF ;
int  button[12] = {0} ;
bool LED_ON[8] = {false} ;
bool page = 0 ;
int  function[8] { 0 } ;
bool function_type[8] = { 0, 0, 0, 0, 0, 0, 0, 1 } ;
int  function_flash[4] { 0 } ;
int  react_band = -1 ;
bool control[8] = { false, false, false, false, false, false, false, false } ;
bool control_type[8] = { 0, 0, 0, 0, 0, 0, 0, 1 } ;

int cc_reactive[3] = { 43, 44, 42 } ;
int cc_control[8] = { 60, 46, 62, 61, 41, 45, 71, 59 } ;

bool labelUpdated[9]     = { false, false, false, false, false, false, false, false, false } ;
bool graphUpdated[9]     = { false, false, false, false, false, false, false, false, true  } ;
bool controlUpdated[4]   = { false, false, false, false } ;
bool functionUpdated[4]  = { false, false, false, false } ;
bool spectrumUpdated = true ;

int csPins[] = {0, 1, 2, 3, 4, 5, 6, 9, 14} ;
const char* label[] = 
{
      "LUMA",
      "BLEND",
      "HUE",
      "SATURATION",
      "X OFFSET",
      "Y OFFSET",
      "SCALE",
      "ANGLE",
      "AUDIO"
} ;
const char* label_B[] = 
{
      "VALUE",
      "BLUR",
      "BLOOM",
      "SHARPEN",
      "HUE MODULO",
      "HUE LFO",
      "HUE OFFSET",
      "DELAY TIME",
      "VIDEO"
} ;
const char* boot_label[] = 
{
      "A",
      "U",
      "T",
      "O",
      "W",
      "A A A",
      "V",
      "E",
      ":)\n    &"
} ;
const char* control_label[] = 
{
      "INVERT LUMA",
      "TOROIDAL",
      "INVERT HUE",
      "INVERT SAT",
      "H MIRROR",
      "V MIRROR",
      "WET/DRY",
      "AUDIO & VIDEO"
} ;
const char* function_label[] = 
{
      "0x",
      "2x",
      "5x",
      "10x"
} ;
const char* function_labelB[] = 
{
      "0x",
      "2x",
      "5x",
      "10x"
} ;

Adafruit_SSD1306 displays[9] {
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 0),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 1),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 2),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 3),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 4),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 5),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 6),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 9),
  Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 15, -1, 14)
} ;

AudioControlSGTL5000 audioShield ;
AudioInputI2S audioIn ;
AudioAnalyzeFFT256 fft ;
AudioConnection patchCord1(audioIn, 0, fft, 0) ;

float smoothedLevels[128] = { 0 } ;
float peakLevels[32] = { 0 } ;
unsigned long peakHoldStart[32] = { 0 } ;
#define PEAK_HOLD_MS 1000
#define PEAK_DECAY   0.0012f
#define ATTACK  0.8
#define DECAY 0.09

uint16_t readButtons() {
  digitalWrite(BUTTON_LOAD, LOW) ;
  delayMicroseconds(10) ;
  digitalWrite(BUTTON_LOAD, HIGH) ;
  delayMicroseconds(10) ;

  uint16_t result = 0 ;
  for(int i = 15 ; i > -1 ; i--)
  {
    result |= (digitalRead(BUTTON_DATA) << i) ;
    digitalWrite(BUTTON_CLOCK, HIGH) ;
    delayMicroseconds(10) ;
    digitalWrite(BUTTON_CLOCK, LOW) ;
    delayMicroseconds(10) ;
  }
  return result ;
}

void setLEDs(byte ledByte)
{
  digitalWrite(LED_LATCH, LOW) ;
  shiftOut(LED_DATA, LED_CLOCK, MSBFIRST, ledByte) ;
  digitalWrite(LED_LATCH, HIGH) ;
}

void clearGarbageColumn(int screenIndex)
{
  digitalWrite(csPins[screenIndex], LOW) ;
  digitalWrite(15, HIGH) ;
  for(int set = 0 ; set < 8 ; set++)
  {
    displays[screenIndex].ssd1306_command(0xB0 + set) ;
    displays[screenIndex].ssd1306_command(0x7E) ;
    displays[screenIndex].ssd1306_command(0x10) ;
    SPI.transfer(0x00) ;
    SPI.transfer(0x00) ;
  }
  digitalWrite(csPins[screenIndex], HIGH) ;
}

void clearScreen(int display_id) 
{
  displays[display_id].fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_BLACK) ;
  displays[display_id].display() ;
}

void updateLabel(int display_id, const char* text, int textSize) 
{
  displays[display_id].fillRect(0, 0, SCREEN_WIDTH, 20, SSD1306_BLACK) ;
  displays[display_id].setTextColor(SSD1306_WHITE) ;
  displays[display_id].setTextSize(textSize) ;

  int16_t x1, y1 ;
  uint16_t w, h ;
  displays[display_id].getTextBounds(text, 0, 0, &x1, &y1, &w, &h) ;
  int xPos = (SCREEN_WIDTH - w) / 2 ;
  displays[display_id].setCursor(xPos, 0) ;
  displays[display_id].println(text) ;
  displays[display_id].display() ;
}

void updateControlLabel(int display_id, const char* text, int textSize, int type, int value) 
{
  displays[display_id].fillRect(0, 47, SCREEN_WIDTH, 9, SSD1306_BLACK) ;
  displays[display_id].setTextColor(SSD1306_WHITE) ;
  displays[display_id].setTextSize(textSize) ;

  displays[display_id].setCursor(6, 48) ;
  displays[display_id].println(text) ;
  if (type == 0)
  {
    if (value == false)
    {
      displays[display_id].fillRect(83, 47, 19, 9, SSD1306_WHITE) ;
      displays[display_id].setTextColor(SSD1306_BLACK) ;
    }
    else
    {
      displays[display_id].setTextColor(SSD1306_WHITE) ;
    }
    displays[display_id].setCursor(84, 48) ;
    displays[display_id].println("OFF") ;
    if (value == true)
    {
      displays[display_id].fillRect(107, 47, 14, 9, SSD1306_WHITE) ;
      displays[display_id].setTextColor(SSD1306_BLACK) ;
    }
    else
    {
      displays[display_id].setTextColor(SSD1306_WHITE) ;
    }
    displays[display_id].setCursor(108, 48) ;
    displays[display_id].println("ON") ;
  }
  else if (type == 1)
  {
    displays[display_id].setTextColor(SSD1306_BLACK) ;
    if (value == 1)
    {
      displays[display_id].fillRect(87, 47, 32, 9, SSD1306_WHITE) ;
      displays[display_id].setTextColor(SSD1306_BLACK) ;
    }
    else
    {
      displays[display_id].setTextColor(SSD1306_WHITE) ;
    }
    displays[display_id].setCursor(89, 48) ;
    displays[display_id].println("RESET") ;
  }
  displays[display_id].display() ;
}

void updateFunctionLabel(int display_id, const char* text, int textSize, int type, int function_id) 
{
  displays[display_id].fillRect(0, 47, SCREEN_WIDTH, 9, SSD1306_BLACK) ;
  displays[display_id].setTextColor(SSD1306_WHITE) ;
  displays[display_id].setTextSize(textSize) ;

  for(int i = 0 ; i < 4 ; i++)
  {
    if (function[function_id] == i)
    {
      displays[display_id].fillRect(13 + (28 * i), 47, 15 + (6 * (i == 3)), 9, SSD1306_WHITE) ;
      displays[display_id].setTextColor(SSD1306_BLACK) ;
    }
    displays[display_id].setCursor(14 + (28 * i), 48) ;
    displays[display_id].println(function_label[i]) ;
    if (function[function_id] == i)
    {
      displays[display_id].setTextColor(SSD1306_WHITE) ;
    }
  }
  displays[display_id].display() ;
}

void updateGraph(int display_id, int value, int function_id) 
{
  displays[display_id].fillRect(0, 23, SCREEN_WIDTH, 17, SSD1306_BLACK) ;

  if (function_direction[function_id] == -1)
  {
    value = 127 - value ;
  }

  int left  = constrain(map(value, 1, 63, 1, 63), 1, 63) ;
  int right = constrain(map(value, 64, 126, 63, 127), 63, 126) ;

  if (function_graphType[function_id] == 0)
  {
    displays[display_id].fillRect(1, 26, constrain(value, 1, 126), 12, SSD1306_WHITE) ;
  }
  else
  {
    displays[display_id].fillRect(left, 26, right - left + 1, 12, SSD1306_WHITE) ;
  }
  displays[display_id].drawLine(1, 23, 1, 40, SSD1306_WHITE) ;
  displays[display_id].drawLine(126, 23, 126, 40, SSD1306_WHITE) ;
  displays[display_id].drawLine(63, 23, 63, 40, SSD1306_WHITE) ;
  displays[display_id].drawLine(1, 32, 126, 32, SSD1306_WHITE) ;
  displays[display_id].display() ;
}

void drawSpectrum() {
  if (page == 0) {  
    if (fft.available()) {
      displays[8].fillRect(0, 23, 128, 41, SSD1306_BLACK) ;
      unsigned long now = millis() ;

      for (int i = 0 ; i < 32 ; i++) {
        float level = fft.read((i * 4) + 1) ;

        float compensation = 1.0f + (float)i * 0.18f ;
        level *= compensation ;

        if (level > smoothedLevels[i]) {
          smoothedLevels[i] += (level - smoothedLevels[i]) * ATTACK ;
        } else {
          smoothedLevels[i] += (level - smoothedLevels[i]) * DECAY ;
        }

        if (smoothedLevels[i] >= peakLevels[i]) {
          peakLevels[i] = smoothedLevels[i] ;
          peakHoldStart[i] = now ;
        } else if (now - peakHoldStart[i] > PEAK_HOLD_MS) {
          peakLevels[i] = max(0.0f, peakLevels[i] - PEAK_DECAY) ;
        }

        int barHeight  = constrain((int)(smoothedLevels[i] * 40 * 8), 0, 40) ;
        int peakHeight = constrain((int)(peakLevels[i]    * 40 * 8), 0, 40) ;

        if (barHeight > 0) {
          displays[8].fillRect(1 + i * 4, 63 - barHeight, 3, barHeight, SSD1306_WHITE) ;
        }
        if (peakHeight > barHeight && peakHeight > 0) {
          displays[8].drawFastHLine(1 + i * 4, 63 - peakHeight, 3, SSD1306_WHITE) ;
        }
      }
      displays[8].display() ;
    }
  }
}

int fps_loop = 0 ;
void drawFPS() {
  fps_loop++ ;
  if (fps_loop > 300)
  {
    fps_loop = 0 ;
    displays[8].fillRect(96, 0, 32, 9, SSD1306_BLACK) ;
    displays[8].setTextColor(SSD1306_WHITE) ;
    displays[8].setTextSize(1) ;
    displays[8].setCursor(97, 1) ;
    displays[8].println((int)smoothedFPS) ;
    displays[8].display() ;
  }
}

unsigned long lastMidiWatchdog = 0 ;
#define MIDI_WATCHDOG_MS 2000
bool midiInitialized = false ;

void initializeAllMidiControls() {
  usbMIDI.sendControlChange(16, 64, 1) ;
  usbMIDI.send_now() ;
  for(byte cc = 17 ; cc <= 127 ; cc++) {
    usbMIDI.sendControlChange(cc, 63, 1) ;
    if (cc % 8 == 0) { 
      usbMIDI.send_now() ;
      delayMicroseconds(500) ;
    }
  }
  usbMIDI.send_now() ;
}

void setup() 
{ 
  Serial.begin(115200) ;

  AudioMemory(32) ;
  audioShield.enable() ;
  audioShield.inputSelect(AUDIO_INPUT_LINEIN) ;
  audioShield.lineInLevel(1) ;

  Serial.println("Audio Setup") ;

  pinMode(12, INPUT) ;

  pinMode(BUTTON_DATA, INPUT) ;
  pinMode(BUTTON_CLOCK, OUTPUT) ;
  pinMode(BUTTON_LOAD, OUTPUT) ;
  digitalWrite(BUTTON_CLOCK, LOW) ;
  digitalWrite(BUTTON_CLOCK, HIGH) ;
  readButtons() ;
  readButtons() ;
  for(int i = 0 ; i < 12 ; i++) { button[i] = 0 ; }

  Serial.println("Buttons initialized") ;

  pinMode(LED_DATA, OUTPUT) ;
  pinMode(LED_CLOCK, OUTPUT) ;
  pinMode(LED_LATCH, OUTPUT) ;
  setLEDs(0x00) ;
  delay(50) ;
  setLEDs(0xFF) ;
  delay(100) ;
  setLEDs(0x00) ;

  Serial.println("LEDs initialized") ;

  for (int i = 0 ; i < 9 ; i++) 
  {
    pinMode(csPins[i], OUTPUT) ;
    digitalWrite(csPins[i], HIGH) ;
  }

  pinMode(29, OUTPUT) ;
  digitalWrite(29, LOW) ;
  delay(250) ;
  digitalWrite(29, HIGH) ;
  delay(250) ;

  for (int i = 0 ; i < 9 ; i++) 
  {
    displays[i].begin(SSD1306_SWITCHCAPVCC, 0, false, true) ;
    displays[i].ssd1306_command(0x00) ;
    displays[i].ssd1306_command(0x11) ;
    displays[i].clearDisplay() ;
    displays[i].display() ;
    updateLabel(i, boot_label[i], 4) ;
  }

  Serial.println("Screens initialized") ;

  delay(50) ;
}

unsigned long boot_flag = millis() ;

void loop() 
{
  usbMIDI.read() ;

  unsigned long now = micros() ;
  unsigned long delta = now - lastFrameTime ;

  static int fps_calc_counter = 0 ;
  if (++fps_calc_counter >= 10) {
    fps_calc_counter = 0 ;
    if (delta > 0) {
      currentFPS = 10000000.0f / delta ;
      smoothedFPS = (smoothedFPS * FPS_SMOOTH) + (currentFPS * (1.0f - FPS_SMOOTH)) ;
    }
    lastFrameTime = now ;
  }

  if ((booting) & (!booted)) //CHASE LEDS and BOOT
  {
    if (scroll_LEDs)
    {
      for(int i = 0 ; i < 8 ; i++)
      {
        digitalWrite(25, LOW) ;
        shiftOut(22, 24, MSBFIRST, 1 << i) ;
        digitalWrite(25, HIGH) ;
        delay(75) ;
      }
      if (((millis() - boot_flag) > (1000 * boot_time_seconds)) and (!booted))
      {
        scroll_LEDs = false ;
        booting = false ;

        // Change these in the boot finalized block
        digitalWrite(LED_LATCH, LOW) ;
        delay(100) ;          // was 1000
        setLEDs(0xFF) ;
        digitalWrite(LED_LATCH, HIGH) ;
        delay(100) ;          // was 1000
        setLEDs(0x00) ;

        booted = true ;
        initializeAllMidiControls() ;
        for (int i = 0 ; i < 9 ; i++) 
        {
          clearScreen(i) ;
        }
        Serial.println("Boot finalized") ;
      }
    }
  }

  if (!booting)
  {

    if (booted)
    {
      // MIDI WATCHDOG ////////////////////////////////////////////////////////
      if (!midiInitialized) {
      uint16_t rawCheck = readButtons() ;
      bool anyEncoder = false ;
      for(int i = 0 ; i < 8 ; i++) {
        if (encoders[i]->read() != 0) { anyEncoder = true ; break ; }
      }
      bool anyButton = (~rawCheck & 0x0FFF) != 0 ;
      
      if (anyEncoder || anyButton) {
        initializeAllMidiControls() ;
        midiInitialized = true ;
      }
      return ; // don't process anything until initialized
    }
      if (millis() - lastMidiWatchdog > MIDI_WATCHDOG_MS) {
        lastMidiWatchdog = millis() ;
        usbMIDI.read() ;
        usbMIDI.sendControlChange(0, 0, 1) ; // CC0 bank select, safe no-op
        usbMIDI.send_now() ;
        if (!midiInitialized) {
          initializeAllMidiControls() ;
          midiInitialized = true ;
        }
      }
      /////////////////////////////////////////////////////////////////////////
      //DEBOUNCE BUTTONS
      static unsigned long lastButtonRead = 0 ;
      if (micros() - lastButtonRead > 5000) {
        uint16_t rawButtons = readButtons() ;
        lastButtonRead = micros() ;

        if (rawButtons != lastRawButton) {
          lastChangeTime = millis() ;
          lastRawButton = rawButtons ;
        }

        if (millis() - lastChangeTime > DEBOUNCE_MS) {
          stableButtons = rawButtons ;
        }
      }

      uint16_t mask     = 0x0FFF ;
      uint16_t pressed  = (~stableButtons &  lastButtons) & mask ;
      uint16_t released = ( stableButtons & ~lastButtons) & mask ;
      uint16_t held     = (~stableButtons & ~lastButtons) & mask ;

      byte ledByte = 0 ;

      //MENU GROUP
      if (released & (1 << BTN_MENU))
      {
        page = !(page) ;        
        for(int i = 0 ; i < 9 ; i++) { labelUpdated[i] = false ; }
        for(int i = 0 ; i < 8 ; i++) { graphUpdated[i] = false ; }
        for(int i = 0 ; i < 4 ; i++) { controlUpdated[i] = false ; }
        for(int i = 0 ; i < 4 ; i++) { functionUpdated[i] = false ; }

        if (page == 0) { 
          LED_ON[LED_5] = false ;
        } 
        else if (page == 1) {  
          LED_ON[LED_5] = true ;
          displays[8].fillRect(0, 23, 128, 41, SSD1306_BLACK) ;
        }
        for(int i = 0 ; i < 4 ; i++) { function_flash[i] = 0 ; }
        for(int i = 0 ; i < 4 ; i++) { LED_ON[LED_1 + i] = false ; }
      }

      for(int i = 0 ; i < 3 ; i++) { LED_ON[LED_6 + i] = false ; }

      //CONTROL GROUP
      for(int i = 0 ; i < 4 ; i++)
      {
        int temp_control = (4 * page) + i ;

        if (released & (1 << (BTN_1 + i)))
        {
          if (temp_control == 7)
          {
            memcpy(function_value, function_reset, sizeof(function_value)) ;
            for(int j = 0 ; j < 9 ; j++) { labelUpdated[j] = false ; }
            for(int j = 0 ; j < 8 ; j++) { graphUpdated[j] = false ; }
            for(int j = 0 ; j < 4 ; j++) { controlUpdated[j] = false ; }
            usbMIDI.sendControlChange(cc_control[7], 127, 1) ; usbMIDI.send_now() ;
          }
          else
          {
            control[temp_control] = !control[temp_control] ;
            usbMIDI.sendControlChange(cc_control[temp_control], control[temp_control] ? 127 : 0, 1) ;
            usbMIDI.send_now() ;
          }
          controlUpdated[i] = false ;
        }
      }

      //FUNCTION GROUP
      for(int i = 0 ; i < 4 ; i++)
      {
        int temp_function = (4 * page) + i ;
        if (released & (1 << (BTN_5 + i)))
        {
          if (function[temp_function] > 0)
          {
            usbMIDI.sendControlChange(16 + (16 * function[temp_function]) + temp_function, 0, 1) ; usbMIDI.send_now() ;
          }
          function[temp_function]++ ;
          if (function[temp_function] == 4) { function[temp_function] = 0 ; }
          else
          {
            usbMIDI.sendControlChange(16 + (16 * function[temp_function]) + temp_function, 127, 1) ; usbMIDI.send_now() ;
          }

          functionUpdated[i] = false ;
          function_flash[i] = 1500 ;
          LED_ON[LED_1 + i] = min(1, (function[temp_function])) ;
        }
      }

      for(int i = 0 ; i < 4 ; i++)
      {
        int temp_function = (4 * page) + i ;
        function_flash[i] = max(0, function_flash[i] - (function[temp_function] * (3 + (3 * (LED_ON[LED_1 + i] == false))))) ;
        if (function_flash[i] == 0)
        {
          function_flash[i] = 1500 ;
          if ((page + i >= page) && (i < (page + 4)))
          {
            LED_ON[LED_1 + i] = (1 - LED_ON[LED_1 + i]) * min(1, function[temp_function]) ;
          }
        }
      }

      //REACTIVE GROUP
      if (page == 0)
      {
        for(int i = 0 ; i < 3 ; i++)
        {
          if (released & (1 << BTN_LOW + i))
          {
            for(int ii = 0 ; ii < 3 ; ii++)
            {
              usbMIDI.sendControlChange(cc_reactive[ii], 0, 1) ; usbMIDI.send_now() ;
            }
            if (react_band == i) { react_band = -1 ; } else { react_band = i ; usbMIDI.sendControlChange(cc_reactive[i], 127, 1) ; usbMIDI.send_now() ; }
          }
        }
        if (!(held & (1 << (BTN_MENU))))
        {
          drawSpectrum() ;
        }
      }

      if ((react_band > -1) && (page == 0)) { LED_ON[LED_6 + react_band] = true ; }

      //ENCODERS GROUP
      for(int i = 0 ; i < 8 ; i++)
      {
        long pos = encoders[i]->read() ;
        int temp_function = i + (9 * page) ;

        if (pos == lastEncPos[temp_function]) continue ;
        if (temp_function >= 18) { continue ; }

        function_value[temp_function] += encoder_speed * ((pos > lastEncPos[temp_function])
          ? -(function_direction[temp_function])
          :  (function_direction[temp_function])) ;
        function_value[temp_function] = constrain(function_value[temp_function], 0, 127) ;
        lastEncPos[temp_function] = pos ;
        if (i < 8) { graphUpdated[i] = false ; }
        usbMIDI.sendControlChange(cc_encoder[temp_function], function_value[temp_function], 1) ;
        usbMIDI.send_now() ;
      }

      //SETTLE UP
      lastButtons = stableButtons ;

      static byte lastLedByte = 0xFF ;
      for(int i = 0 ; i < 8 ; i++)
      {
        if (LED_ON[i]) ledByte |= (1 << i) ;
      }
      if (ledByte != lastLedByte) {
        setLEDs(ledByte) ;
        lastLedByte = ledByte ;
      }

      //DRAW
      done = false ;
      
      if (!done)
      {
        //LABEL GROUP
        for(int i = 0 ; i < 9 ; i++)
        {
          if (labelUpdated[i] == false)
          { 
            if (page == 0) {
              updateLabel(i, label[i], 2) ;
            } else {
              updateLabel(i, label_B[i], 2) ;
            }
            labelUpdated[i] = true ; 
            done = true ; 
            return ;
          }
        }
      }
      if (!done)
      {
        //CONTROL GROUP
        for(int i = 0 ; i < 4 ; i++)
        {
          if (controlUpdated[i] == false)
          { 
            int temp_control = (4 * page) + i ;
            updateControlLabel(i, control_label[temp_control], 1, control_type[temp_control], control[temp_control]) ;
            controlUpdated[i] = true ; 
            done = true ; 
            return ;
          }
        }
      }
      if (!done)
      {
        //FUNCTION GROUP
        for(int i = 0 ; i < 4 ; i++)
        {
          if (functionUpdated[i] == false)
          { 
            int temp_function = (4 * page) + i ;
            updateFunctionLabel(i + 4, "null", 1, function_type[temp_function], temp_function) ;
            functionUpdated[i] = true ; 
            done = true ; 
            return ;
          }
        }
      }
      if (!done)
      {
        //GRAPH GROUP
        for(int i = 0 ; i < 8 ; i++)
        {
          if (graphUpdated[i] == false)
          { 
            int temp_function = i + (9 * page) ;
            updateGraph(i, function_value[temp_function], temp_function) ;
            graphUpdated[i] = true ; 
            done = true ; 
            return ;
          }
        }
        drawFPS() ;
      }
    }
  }
}

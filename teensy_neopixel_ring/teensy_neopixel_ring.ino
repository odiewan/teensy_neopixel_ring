
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdio.h>
#include <Adafruit_GFX.h>
#include <led_pulse_train.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <serialPrint.h>
#include <neopixel_effects.h>

#define SS_SWITCH               24
#define SS_NEOPIX               6

#define SEESAW_ADDR             0x36

#define EEPROM_SIZE             512
#define EEPROM_DELAY            5

#define WHITE               0x00FFFFFF
#define OFF                 0x00000000
#define RED                 0x00FF0000
#define GREEN               0x0000FF00
#define BLUE                0x000000FF
#define ORANGE              0x00FF4100


#define BTN_PIN                 3
#define NPXL_PIN                2

#define NUM_NEOPIXELS           16

#define LED_ON_TIME             50
#define LED_OFF_TIME            750

#define DIR_DOWN                -1
#define DIR_NONE                0
#define DIR_UP                  1

#define LOOP_DELAY             100

#define SER_WAIT_TICKS          5
#define SER_WAIT_DELAY          250

#define SETUP_DELAY             1000
#define NUM_BYTES               32


#define NPX_ROT_DIR_STOP        0
#define NPX_ROT_DIR_CCW         1
#define NPX_ROT_DIR_CW          2

#define NPX_INC_MAX             64
#define NPX_AMP_MAX             255
#define NPX_AMP_MID             127
#define NPX_CALL_DELAY_CYCLES   15
#define NPX_CALL_INTV           8

#define DEF_NPX_INC             1
#define DEF_NPX_AMP_MIN         0
#define DEF_NPX_AMP_MAX         31

#define QUARTER_PI              float(PI / 4.0)

enum op_modes {
  OP_MD_BOOT,
  OP_MD_SETUP,
  OP_MD_ON,
  OP_MD_PATTERN_A,
  OP_MD_PATTERN_B,
  OP_MD_PATTERN_C,
  OP_MD_PATTERN_D,
  OP_MD_PATTERN_E,
  OP_MD_SET_NPX_MODE,
  OP_MD_SET_MAX,
  OP_MD_SET_MIN,
  OP_MD_SET_INT_R,
  OP_MD_SET_INT_G,
  OP_MD_SET_INT_B,
  NUM_OP_MODES,
  };

String op_mode_strs[] = {
  "BOOT",
  "SETUP",
  "ON",
  "OPT_A",
  "OPT_B",
  "OPT_C",
  "OPT_D",
  "OPT_E",
  "SET_NPX_M",
  "SET_MAX",
  "SET_MIN",
  "SET_INT_R",
  "SET_INT_G",
  "SET_INT_B",
  };


enum npx_modes {
  NPX_MD_OFF,
  NPX_MD_ON,
  NPX_MD_ASYC_SINE,
  NPX_MD_WHEEL,
  NPX_MD_WHEEL_SINGLE,
  NPX_MD_WHITE_SINE,
  NPX_MD_RED_SINE,
  NPX_MD_GREEN_SINE,
  NPX_MD_BLUE_SINE,

  NPX_MD_WHITE_STATIC,
  NPX_MD_RED_STATIC,
  NPX_MD_GREEN_STATIC,
  NPX_MD_BLUE_STATIC,
  NPX_MD_ORANGE_STATIC,
  NPX_MD_YELLOW_STATIC,
  NPX_MD_PURPLE_STATIC,

  NPX_MD_WHITE_STATIC_SNG,
  NPX_MD_RED_STATIC_SNG,
  NPX_MD_GREEN_STATIC_SNG,
  NPX_MD_BLUE_STATIC_SNG,
  NPX_MD_ORANGE_STATIC_SNG,
  NPX_MD_YELLOW_STATIC_SNG,
  NPX_MD_PURPLE_STATIC_SNG,

  NPX_MD_RGB_3PX,
  NPX_MD_FIRE_3PX,
  NPX_MD_AFTERBURNER,
  NUM_NPX_MODES,
  };

String npx_mode_strs[] = {
  "OFF",
  "ON",
  "ASYC sine",
  "WHL All",
  "WHL Single",
  "Wht sine",
  "Rd sine",
  "Grn sine",
  "Blu sine",
  "Wht ",
  "Rd",
  "Grn",
  "Blu",
  "Org",
  "Ylw",
  "Prpl",
  "Wht StSng",
  "Rd StSng",
  "Grn StSng",
  "Blu StSng",
  "Org StSng",
  "Ylw StSng",
  "Prp StSng",
  "RGB_3PX",
  "FIRE_3PX",
  "Afterburner",

  };

enum eeprom_registers {
  EE_REG_NEOPIXEL_MODE,
  EE_REG_AMP_MAX,
  EE_REG_AMP_MIN,
  EE_REG_R_INT,
  EE_REG_G_INT,
  EE_REG_B_INT,
  NUM_EEPROM_REG,
  };

uint8_t eeprom[EEPROM_SIZE] = {};

uint8_t eeprom_live[EEPROM_SIZE] = {};
char inByteBuffer[NUM_BYTES] = {};



int x;
int dir;
int npxlMode;
int npxlModeReq;
int opMd;
bool setting_mode;
int btn_dwn_tmr;
int32_t npxIdx;
int32_t enc_ovr_tmr;
int32_t enc_ovr_cmd;


int def_count = 0;
int dir_up_count = 0;
int dir_dn_count = 0;
int amp_sel = 0;

bool npxEnAry[NUM_NEOPIXELS];

uint32_t npxClrAry[NUM_NEOPIXELS];



String inStr = "";

int64_t iCount = 0;

Adafruit_seesaw ss;
Adafruit_NeoPixel ring(NUM_NEOPIXELS, NPXL_PIN, NEO_GRB + NEO_KHZ800);
seesaw_NeoPixel encNpxl = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t enc_position;
int32_t new_position;
int32_t enc_delta;

bool nxplEn;

bool btnDIN;
bool btnDINShadow;
bool btnShortPress;
bool btnShortShadow;
bool btnLongPress;
bool btnOState;
bool btnStateR;
bool btnStateG;
bool btnStateB;
bool btnStateRG;
bool enc_ovr;


neopixel_color npxR;
neopixel_color npxG;
neopixel_color npxB;
uint8_t curR;
uint8_t curG;
uint8_t curB;





uint32_t npxColor;
uint8_t _wheelPos;
uint8_t thrPos;

int npxExState;

int npx_op_mode_mtrx[NUM_OP_MODES] = {
  NPX_MD_OFF,               //  OP_MD_BOOT
  NPX_MD_OFF,               //  OP_MD_SETUP
  NPX_MD_ON,                //  OP_MD_ON
  NPX_MD_BLUE_STATIC_SNG,     //  OP_MD_PATTERN_A
  NPX_MD_WHEEL,             //  OP_MD_PATTERN_B
  NPX_MD_WHEEL_SINGLE,      //  OP_MD_PATTERN_C
  NPX_MD_FIRE_3PX,           //  OP_MD_PATTERN_D
  NPX_MD_AFTERBURNER,           //  OP_MD_PATTERN_E
  NPX_MD_WHEEL_SINGLE,      //  OP_MD_SET_NPX_MO
  NPX_MD_WHITE_STATIC_SNG,  //  OP_MD_SET_MAX
  NPX_MD_ORANGE_STATIC_SNG, //  OP_MD_SET_MIN
  NPX_MD_RED_STATIC_SNG,    //  OP_MD_SET_INT_R
  NPX_MD_GREEN_STATIC_SNG,  //  OP_MD_SET_INT_G
  NPX_MD_BLUE_STATIC_SNG,   //  OP_MD_SET_INT_B
  };


//=================================================================================================
void ledToggle() {
  static bool bit = false;
  bit = !bit;
  digitalWrite(LED_BUILTIN, bit);
  }

//-----------------------------------------------------------------------------
String recvWithEndMarker() {
  static int bCnt = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0) {
    rc = Serial.read();
    if (bCnt < NUM_BYTES - 1) {
      if (rc != endMarker) {
        inByteBuffer[bCnt] = rc;
        }
      bCnt++;
      }
    else {
      serPrntNL("buffer overflow");
      }
    }

  if (bCnt > 0) {
    serPrntVNL("Rx'ed ", bCnt, " bytes");
    inByteBuffer[bCnt] = '\0';
    inStr = inByteBuffer;
    }
  else
    inStr = "";

  bCnt = 0;

  return inStr;
  }

//=================================================================================================
void readEEPROM() {
  serPrntNL("readEEPROM()");
  String tmpStr;
  int byte_read_cnt = 0;

  for (int i = 0; i < NUM_EEPROM_REG; i++) {
    eeprom[i] = EEPROM.read(i);
    eeprom_live[i] = eeprom[i];
    tmpStr = "readEEPROM()[";
    tmpStr += i;
    tmpStr += "]:";
    tmpStr += eeprom_live[i];
    byte_read_cnt++;
    serPrntNL(tmpStr);
    }
  // memcpy(eeprom_live, eeprom, sizeof(eeprom_live));

  // calcColorRange();

  serPrntNL();
  Serial.print(F("Read "));
  Serial.print(byte_read_cnt);
  Serial.print(F(" of "));
  Serial.print(EEPROM_SIZE);
  Serial.println(F(" bytes from EEPROM"));
  // tmpStr = "Read ";
  // tmpStr += byte_read_cnt;
  // tmpStr += " of ";
  // tmpStr += EEPROM_SIZE;
  // tmpStr += " bytes from EEPROM";
  // serPrntNL(tmpStr);
  }


//=================================================================================================
void serPrintLiveEEPROM() {
  String tmpStr;
  for (int i = 0; i < NUM_EEPROM_REG; i++) {
    tmpStr = "serPrintLiveEEPROM()[";
    tmpStr += i;
    tmpStr += "]:";
    tmpStr += eeprom_live[i];
    tmpStr += " e:";
    tmpStr += eeprom[i];
    serPrntNL(tmpStr);
    }
  }

//=================================================================================================
void writeEEPROM() {
  serPrntNL("writeEEPROM()");
  String tmpStr;
  int byte_write_cnt = 0;

  for (int i = 0; i < NUM_EEPROM_REG; i++) {

    byte_write_cnt += writeEepromReg(i);
    delay(EEPROM_DELAY);
    }

  if (byte_write_cnt > 0) {
    tmpStr = "Wrote ";
    tmpStr += byte_write_cnt;
    tmpStr += " of ";
    tmpStr += EEPROM_SIZE;
    tmpStr += " bytes from EEPROM";
    }
  else {
    tmpStr = "No changes to EEPROM";
    }

  serPrntNL(tmpStr);
  }

//=================================================================================================
int writeEepromReg(uint16_t nIdx) {
  serPrnt("writeEepromReg():");
  String tmpStr;
  if (eeprom_live[nIdx] != eeprom[nIdx]) {
    eeprom[nIdx] = eeprom_live[nIdx];
    EEPROM.write(nIdx, eeprom[nIdx]);
    tmpStr = "Register changed: writeEepromReg[";
    tmpStr += nIdx;
    tmpStr += "]:";
    tmpStr += eeprom[nIdx];
    serPrntNL(tmpStr);
    return 1;
    }
  else {
    tmpStr = "Idx:";
    tmpStr += nIdx;
    tmpStr += ": No change to register";
    serPrntNL(tmpStr);
    return 0;
    }
  }

//=================================================================================================
void setup() {
  int ser_wait_cnt = 0;
  opMd = OP_MD_BOOT;
  npxlModeReq = NPX_MD_ASYC_SINE;
  enc_ovr = false;
  enc_ovr_cmd = 0;
  btnDIN = false;
  npxExState = 0;
  thrPos = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  randomSeed(analogRead(0));

  std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
  std::fill(std::begin(npxClrAry), std::end(npxClrAry), 0x00000000);


  Serial.begin(9600);
  ledPulseTrain(3);
  while (!Serial && ser_wait_cnt < SER_WAIT_TICKS) {
    ser_wait_cnt++;
    ledToggle();
    delay(SER_WAIT_DELAY);
    }
  serPrntNL("Serial OK");



  ledPulseTrain(4);
  serPrntNL("Read EEPROM contents");
  readEEPROM();

  npxlMode = eeprom_live[EE_REG_NEOPIXEL_MODE];

  ring.begin();


  for (int i = 0; i < ring.numPixels(); i++) {
    ring.fill(ring.Color(eeprom_live[EE_REG_AMP_MAX], 0, 0), 0, 5);
    ring.fill(ring.Color(0, eeprom_live[EE_REG_AMP_MAX], 0), 5, 10);
    ring.fill(ring.Color(0, 0, eeprom_live[EE_REG_AMP_MAX]), 10, ring.numPixels());

    }

  ring.show();
  encNpxl.show();


  ledPulseTrain(2);
  npxR = neopixel_color(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
  npxG = neopixel_color(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
  npxB = neopixel_color(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);

  ledPulseTrain(6);


  Serial.println("Looking for seesaw!");

  if (!ss.begin(SEESAW_ADDR) || !encNpxl.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while (1) delay(10);
    }
  Serial.println("seesaw started");
  encNpxl.setBrightness(20);
  encNpxl.show();
  delay(10);
  encNpxl.setPixelColor(0, 0x00ff0000);
  encNpxl.show();


  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  enc_position = ss.getEncoderPosition();
  new_position = enc_position;

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();

  encNpxl.setBrightness(20);
  encNpxl.setPixelColor(0, 0x00FF0000);
  encNpxl.show();

  delay(SETUP_DELAY);


  opMd = OP_MD_ON;
  serPrntNL("Setup done");

  }

//-----------------------------------------------------------------------------
void paramIncHandler(int nIdx, int nInc, int nUpLim, int nLoLim) {
  int _param = eeprom_live[nIdx];

  _param += nInc;

  if (_param > nUpLim)
    _param = nLoLim;
  else if (_param < nLoLim)
    _param = nUpLim;

  eeprom_live[nIdx] = _param;
  }

//=================================================================================================
void setAllNeoPixels(uint32_t nColor[]) {
  // ring.fill(ring.Color(cAll.r, cAll.g, cAll.b), 0, ring.numPixels());

  ring.clear();
  for (uint16_t i = 0; i < ring.numPixels(); i++) {
    if (npxEnAry[i]){
      ring.setPixelColor(i, nColor[i]);
      encNpxl.setPixelColor(0, nColor[0]);
    }
    else{
      ring.setPixelColor(i, 0);
      // encNpxl.setPixelColor(0, 0x00FF0000);

    }
  }
  encNpxl.show();
  ring.show();
}


//=================================================================================================
uint32_t Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return ring.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return ring.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
  WheelPos -= 170;
  return ring.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }


//-----------------------------------------------------------------------------
void handleSerIn() {
  if (recvWithEndMarker() > "") {

    if (inStr == "non") {
      nxplEn = true;
      serPrntNL("on: neopixel ring on");
      }

    else if (inStr == "noff") {
      nxplEn = false;
      serPrntNL("off: neopixel ring off");
      }

    else if (inStr == "save") {
      serPrntNL("save: save eeprom");
      writeEEPROM();
      btnStateG = true;
      serPrntNL("save: eeprom saved");
      }

    else if (inStr == "init") {
      serPrntNL("init: init eeprom");
      eeprom_live[EE_REG_NEOPIXEL_MODE] = 1;
      eeprom_live[EE_REG_AMP_MAX] = 127;
      eeprom_live[EE_REG_AMP_MIN] = 100;
      eeprom_live[EE_REG_R_INT] = 1;
      eeprom_live[EE_REG_G_INT] = 2;
      eeprom_live[EE_REG_B_INT] = 3;
      serPrntNL("writing to eeprom");
      delay(250);
      writeEEPROM();
      btnStateG = true;
      delay(250);
      serPrntNL("save: eeprom saved");
      serPrintLiveEEPROM();

      }
    else if (inStr == "read") {
      serPrntNL("read: read eeprom");
      readEEPROM();
      // calcColorRange();
      btnOState = true;
      serPrntNL("read: eeprom read");

      }
    else if (inStr == "prnt") {
      serPrntNL("prnt: read eeprom");
      serPrintLiveEEPROM();

      }


    inStr = "";
    }
  }

//=================================================================================================
void taskSerOut() {
  // String _tmpStr = "";

  Serial.print(F("iC:"));
  Serial.print(iCount);

  Serial.print(F(" btnDIN:"));
  Serial.print(btnDIN);

  Serial.print(F(" btnS:"));
  Serial.print(btnShortPress);

  Serial.print(F(" btnL:"));
  Serial.print(btnLongPress);

  Serial.print(F(" npxExSt:"));
  Serial.print(npxExState);

  // Serial.print(" " + opMd + ":");
  Serial.print("(" + op_mode_strs[opMd] + ")");

  // Serial.print(" " + npxlMode + ":");
  Serial.print("(" + npx_mode_strs[npxlMode] + ")");


  // Serial.print(F(" dEnc:"));
  // Serial.print(enc_delta);

  // _tmpStr += " enc_ovr_cmd:";
  // _tmpStr += enc_ovr_cmd;


  // _tmpStr += " enc_ovr:";
  // _tmpStr += enc_ovr;

  // _tmpStr += " _wPos:";
  // _tmpStr += _wheelPos;

  Serial.print(F(" thrPos:"));
  Serial.print(thrPos);

  Serial.print(F(" npx:"));
  Serial.print(npxIdx);



  Serial.print(F(" EE :max:"));
  Serial.print(eeprom_live[EE_REG_AMP_MAX]);
  // _tmpStr += " min:";
  // _tmpStr += eeprom_live[EE_REG_AMP_MIN];


  // _tmpStr += " dR:";
  // _tmpStr += eeprom_live[EE_REG_R_INT];
  // _tmpStr += " dG:";
  // _tmpStr += eeprom_live[EE_REG_G_INT];
  // _tmpStr += " dB:";
  // _tmpStr += eeprom_live[EE_REG_B_INT];

  // _tmpStr += " btnO";
  // _tmpStr += btnOState;

  // _tmpStr += " btnR";
  // _tmpStr += btnStateR;

  Serial.print(F(" dwn_tmr:"));
  Serial.print(btn_dwn_tmr);

  Serial.print(F(" set_md:"));
  Serial.print(setting_mode);


  // _tmpStr += " npxClr:";
  // _tmpStr += npxColor;

  Serial.print(F(" r:"));
  Serial.print((npxColor & 0x00FF0000) >> 16);
  Serial.print(F(" g:"));
  Serial.print((npxColor & 0x0000FF00) >> 8);
  Serial.print(F(" b:"));
  Serial.println((npxColor & 0x000000FF));

  // serPrntNL(_tmpStr);

  }

//=================================================================================================
void taskNpxModeHandler() {
  if (btnDIN == true) {
    if (btnLongPress == true) {
      npxExState = 2;
    }
    else {
      npxlMode = NPX_MD_PURPLE_STATIC;
      npxExState = 3;
    }
  }
  else {
    npxlMode = npx_op_mode_mtrx[opMd];
    npxExState = 1;
  }
}

//=================================================================================================
void enNpxl(int32_t nIdx) {
  static int32_t _tmpIdx = 0;

  _tmpIdx = ring.numPixels() - npxIdx;
  _tmpIdx = (_tmpIdx & 0x0F);
  npxEnAry[_tmpIdx] = true;
  }


//=================================================================================================
uint32_t packColor(uint8_t nR, uint8_t nG, uint8_t nB) {
  uint32_t tmpColor = 0;
  tmpColor = (0xFF & nR) << 16;
  tmpColor += (0xFF & nG) << 8;
  tmpColor += (0xFF & nB);

  return tmpColor;
  // return ring.Color(eeprom_live[EE_REG_AMP_MAX], eeprom_live[EE_REG_AMP_MAX], eeprom_live[EE_REG_AMP_MAX]);
  }

//=================================================================================================
uint32_t genColor(uint8_t nR, uint8_t nG, uint8_t nB) {
  static float _r = 0;
  static float _g = 0;
  static float _b = 0;

  _r = nR / 255.0 * eeprom_live[EE_REG_AMP_MAX];
  _g = nG / 255.0 * eeprom_live[EE_REG_AMP_MAX];
  _b = nB / 255.0 * eeprom_live[EE_REG_AMP_MAX];

  //---do a 5 sample average
  curR = _r/5 + (4 * (curR)/5);
  curG = _g/5 + (4 * (curG)/5);
  curB = _b/5 + (4 * (curB)/5);

  npxColor = packColor(_r, _g, _b);

  return npxColor;
}

//=============================================================================
void setColorAry(uint32_t nColor) {
  std::fill(std::begin(npxClrAry), std::end(npxClrAry), nColor);
}

//=============================================================================
void setColorAry(uint8_t nR, uint8_t nG, uint8_t nB) {
  npxColor = packColor(nR, nG, nB);
  setColorAry(npxColor);
}


//=============================================================================
void doAfterBurner() {
  uint32_t abColor = 0;
  static uint8_t tmpR = 0;
  static uint8_t tmpG = 0;
  static uint8_t tmpB = 0;

  static uint8_t tmpRr = 0;
  static uint8_t tmpGr = 0;
  static uint8_t tmpBr = 0;

  thrPos += enc_delta;

  if (thrPos > 255)
    thrPos = 255;
  else if (thrPos < 0)
    thrPos = 0;


  if (thrPos > 250) {
    tmpR = 200;
    tmpG = 200;
    tmpB = 255;
  }
  else if (thrPos > 245 && thrPos <= 250) {
    tmpR = 0;
    tmpG = 200;
    tmpB = 255;
  }
  else if (thrPos > 240 && thrPos <= 245) {
    tmpR = 0;
    tmpG = 150;
    tmpB = 200;
  }
  else if (thrPos > 235 && thrPos <= 240) {
    tmpR = 255;
    tmpG = 65;
    tmpB = 0;
  }
  else if (thrPos > 230 && thrPos <= 235) {
    tmpR = 127;
    tmpG = 32;
    tmpB = 0;
  }
  else {
    tmpR = 0;
    tmpB = 0;
    tmpG = 0;
  }

  for (uint16_t i = 0; i < ring.numPixels(); i++) {
    tmpRr = random(tmpR - 3, tmpR + 3);
    tmpBr = random(tmpB - 3, tmpB + 3);
    tmpGr = random(tmpG - 3, tmpG + 3);
    npxClrAry[i] = packColor(tmpRr, tmpBr, tmpGr);
  }

}

//=============================================================================
void enOneNpxl(int32_t nIdx) {
  std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
  enNpxl(nIdx);
}

//=============================================================================
void taskNeopixelRing() {

  static int _tmpIdx = 0;
  static uint8_t tmpR = 0;
  static uint8_t tmpG = 0;
  static uint8_t tmpB = 0;


  std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);

  switch (npxlMode) {
      default:
      case NPX_MD_OFF:
        tmpR = 1;
        tmpG = 1;
        tmpB = 1;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_ON:
        tmpR = eeprom_live[EE_REG_AMP_MAX] + enc_position;
        tmpG = eeprom_live[EE_REG_AMP_MAX] + enc_position;
        tmpB = eeprom_live[EE_REG_AMP_MAX] + enc_position;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_ASYC_SINE:
        tmpR = npxR.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
        tmpG = npxG.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
        tmpB = npxB.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_WHEEL:
        _wheelPos++;
        // npxColor = Wheel(_wheelPos);
        // std::fill(std::begin(npxClrAry), std::end(npxClrAry), npxColor);
        setColorAry(Wheel(_wheelPos));
        break;

      case NPX_MD_WHEEL_SINGLE:
        npxIdx--;
        enOneNpxl(npxIdx);

        _wheelPos++;
        setColorAry(Wheel(_wheelPos));
        break;

      case NPX_MD_RED_SINE:
        tmpR = npxR.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
        tmpG = 0;
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_GREEN_SINE:
        tmpR = 0;
        tmpG = npxG.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_BLUE_SINE:
        tmpR = 0;
        tmpG = 0;
        tmpB = npxB.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
        setColorAry(tmpR, tmpG, tmpB);

        break;

        //=====================================================
      case NPX_MD_WHITE_STATIC:
        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = eeprom_live[EE_REG_AMP_MAX];
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);

        break;

      case NPX_MD_RED_STATIC:
        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = 0;
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);

        break;

      case NPX_MD_GREEN_STATIC:
        tmpR = 0;
        tmpG = eeprom_live[EE_REG_AMP_MIN];
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_BLUE_STATIC:
        tmpR = 0;
        tmpG = 0;
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);
        break;


      case NPX_MD_ORANGE_STATIC:
        tmpR = 0;
        tmpG = 0;
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_YELLOW_STATIC:
        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = eeprom_live[EE_REG_AMP_MAX];
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_PURPLE_STATIC:
        tmpR = 0;
        tmpG = eeprom_live[EE_REG_AMP_MAX];
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);
        break;


      case NPX_MD_WHITE_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = eeprom_live[EE_REG_AMP_MAX];
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_RED_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = 0;
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_GREEN_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = 0;
        tmpG = eeprom_live[EE_REG_AMP_MIN];
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_BLUE_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = 0;
        tmpG = 0;
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);
        break;


      case NPX_MD_ORANGE_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = 255;
        tmpG = 165;
        tmpB = 0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_YELLOW_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = eeprom_live[EE_REG_AMP_MAX];
        tmpB =  0;
        setColorAry(tmpR, tmpG, tmpB);
        break;

      case NPX_MD_PURPLE_STATIC_SNG:
        enOneNpxl(npxIdx);

        tmpR = eeprom_live[EE_REG_AMP_MAX];
        tmpG = 0;
        tmpB = eeprom_live[EE_REG_AMP_MAX];
        setColorAry(tmpR, tmpG, tmpB);
        break;


      case NPX_MD_RGB_3PX:
        _tmpIdx++;

        std::fill(std::begin(npxClrAry), std::end(npxClrAry), 0);
        npxClrAry[_tmpIdx & 0xf] = 0x00ff0000;
        npxClrAry[(_tmpIdx + 5) & 0xf] = 0x0000ff00;
        npxClrAry[(_tmpIdx + 10) & 0xf] = 0x000000ff;

        break;
      case NPX_MD_FIRE_3PX:

        for (uint16_t i = 0; i < ring.numPixels(); i++){
          tmpR = random(200, 255);   //Random RED value
          tmpG = random(0, 65);   //Random GREEN value
          tmpB = 0;
          npxClrAry[i] = genColor(tmpR, tmpG, tmpB);

        }

        break;

      case NPX_MD_AFTERBURNER:
        doAfterBurner();
        break;
    }

      setAllNeoPixels(npxClrAry);

  }

//=================================================================================================
void taskModeHandler() {
  static bool setting_mode_shadow;

  if (setting_mode) {
    if (setting_mode_shadow != setting_mode)
      opMd = OP_MD_SET_NPX_MODE;
    else {
      if (opMd >= NUM_OP_MODES)
        opMd = OP_MD_SET_NPX_MODE;
      }
    }
  else {
    if (setting_mode_shadow != setting_mode)
      opMd = OP_MD_ON;
    else {
      if (opMd > OP_MD_SET_NPX_MODE - 1)
        opMd = OP_MD_ON;
      }
    }

  setting_mode_shadow = setting_mode;
  }


//=================================================================================================
int32_t knob_to_npx_id() {
  static int32_t _idx = 0;
  if (enc_delta != 0) {
    enc_ovr = true;
    enc_ovr_tmr = 100;

    }

  if (enc_ovr) {
    enc_ovr_cmd = (int32_t)ring.numPixels() - (enc_position % (int32_t)ring.numPixels());
    _idx = (enc_ovr_cmd & 0x0F);

    enc_ovr_tmr--;
    if (enc_ovr_tmr == 0)
      enc_ovr = false;
    }
  return _idx;
  }

//=================================================================================================
void handle_button_press() {
  btnDIN = !ss.digitalRead(SS_SWITCH);
  if (btnDIN) {
    btn_dwn_tmr++;
    }
  else if (btnDINShadow != btnDIN) {
    if (btn_dwn_tmr > 10) {
      btnLongPress = true;
      btnShortPress = false;
      }
    else {
      btnLongPress = false;
      btnShortPress = true;
      }
    btn_dwn_tmr = 0;
    }
  else {
    btnLongPress = false;
    btnShortPress = false;
    btn_dwn_tmr = 0;
    }

  //---handle button press in each op mode-------------------------------------
  switch (opMd) {
      default:
        break;

      case OP_MD_ON:
      case OP_MD_PATTERN_A:
      case OP_MD_PATTERN_B:
      case OP_MD_PATTERN_C:
      case OP_MD_PATTERN_D:
      case OP_MD_PATTERN_E:
        if (btnLongPress) {
          setting_mode = true;
          btnStateR = true;

          }
        if (btnShortPress)
          opMd++;
        break;

      case OP_MD_SET_NPX_MODE:
      case OP_MD_SET_MAX:
      case OP_MD_SET_MIN:
      case OP_MD_SET_INT_R:
      case OP_MD_SET_INT_G:
      case OP_MD_SET_INT_B:
        if (btnLongPress) {
          setting_mode = false;
          btnStateRG = true;
          writeEEPROM();
          btnStateRG = true;
          }
        if (btnShortPress)
          opMd++;
        break;
    }

  btnDINShadow = btnDIN;
  }

//=================================================================================================
void handleEncoder() {
  new_position = ss.getEncoderPosition();

  //--- calculate encoder data
  if (enc_position != new_position) {

    enc_delta = new_position - enc_position;
    enc_position = new_position;
    }
  else
    enc_delta = 0;

  switch (opMd) {
      default:
        break;

      case OP_MD_ON:
      case OP_MD_PATTERN_A:
      case OP_MD_PATTERN_B:
      case OP_MD_PATTERN_C:
        npxIdx = knob_to_npx_id();
        break;


      case OP_MD_SET_NPX_MODE:
        paramIncHandler(EE_REG_NEOPIXEL_MODE, enc_delta, NUM_NPX_MODES, 0);
        npxIdx = map(eeprom_live[EE_REG_NEOPIXEL_MODE], 0, NUM_NPX_MODES, 0, ring.numPixels());
        break;

      case OP_MD_SET_MAX:
        paramIncHandler(EE_REG_AMP_MAX, enc_delta, 255, 1);
        npxIdx = map(eeprom_live[EE_REG_AMP_MAX], 0, 255, 0, ring.numPixels());
        break;

      case OP_MD_SET_MIN:
        paramIncHandler(EE_REG_AMP_MIN, enc_delta, 64, 1);
        npxIdx = map(eeprom_live[EE_REG_AMP_MIN], 0, 64, 0, ring.numPixels());
        break;

      case OP_MD_SET_INT_R:
        paramIncHandler(EE_REG_R_INT, enc_delta, 64, 1);
        npxIdx = map(eeprom_live[EE_REG_R_INT], 1, 64, 0, ring.numPixels());
        break;

      case OP_MD_SET_INT_G:
        paramIncHandler(EE_REG_G_INT, enc_delta, 64, 1);
        npxIdx = map(eeprom_live[EE_REG_G_INT], 1, 64, 0, ring.numPixels());
        break;

      case OP_MD_SET_INT_B:
        paramIncHandler(EE_REG_B_INT, enc_delta, 64, 1);
        npxIdx = map(eeprom_live[EE_REG_B_INT], 1, 64, 0, ring.numPixels());
        break;

    }
  }

//=================================================================================================
void loop() {
  handle_button_press();
  handleEncoder();
  //handleSerIn();

  taskModeHandler();
  taskNpxModeHandler();

  // if (iCount % NPX_CALL_INTV == 0 && iCount > NPX_CALL_DELAY_CYCLES) {
  taskNeopixelRing();

  // }

// if (iCount % SER_WAIT_TICKS == 0)
  taskSerOut();

  iCount++;
  delay(LOOP_DELAY);

  }

#include <bluefruit.h>
#include <CC2500.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <Nffs.h>

#define SLOPE_FILENAME "/slope.txt"
#define INTERCEPT_FILENAME "/intercept.txt"

#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

NffsFile file;

ble_gap_evt_adv_report_t* global_report;

int newValue = 0;
uint16_t value ;
long notif_millis = millis();
long periph_notif_millis = millis();
long cc2500_recv_time = 0;
int waitTime = 0;
int showReadings = 0;
int alertCount = 0;
int periph_connected = 0;
int lowBatt = 0;
int missCount = 0;
int connecting = 0;

//cgms
//
struct readings {
  long seconds;
  long rawcounts;
  int glucose;
};
struct readings readings_arr[3];
struct readings reading;

int message_len = 0;
int GLUCOSE = 0;
int EST_GLUCOSE = 0;
int SLOPE = 700;
int INTERCEPT = 30000;
long ISIG;
long OLD_ISIG;

uint8_t message[12];
int len = 0;
//
//
//cc2500
//
uint8_t cgms_battery = 0xff;
uint8_t txid = 0x00;
long rawcount1 = 0;
long rawcount2 = 0;
int SPI_DELAY_LOCAL = 0; //0 works for rfduino
int firstTime = true;
//
//
#define BTLE_BATTERY             0x07 //btle battery
#define TRANSMITTER_FULL_PACKET  0x0F //Transmitter Full Packet

typedef struct _Dexcom_packet
{
  uint32_t dest_addr;
  uint32_t src_addr;
  uint8_t  port;
  uint8_t  device_info;
  uint8_t  txId;
  uint16_t raw;
  uint16_t filtered;
  uint8_t  battery;
  uint8_t  unknown;
  uint8_t  checksum;
  uint8_t  RSSI;
  uint8_t  LQI;
} Dexcom_packet;

uint8_t packet[40];
uint8_t oldPacket[40];

static const int NUM_CHANNELS = 4;
static uint8_t nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
int8_t fOffset[NUM_CHANNELS] = {0xe4, 0xe3, 0xe2, 0xe2};
const int GDO0_PIN = PIN_A4;     // the number of the GDO0_PIN pin

CC2500 cc2500;
byte buffer[20];

//
// end cc2500
//

//
//mi
//
//https://github.com/vshymanskyy/miband2-python-test
#include <Crypto.h>  //https://github.com/rweather/arduinolibs
#include <AES.h>
#include <string.h>

AES128 aes128;

byte enc_buffer[18];

uint8_t AUTH_SEND_KEY = 0x01;
uint8_t AUTH_REQUEST_RANDOM_AUTH_NUMBER  = 0x02;
uint8_t AUTH_SEND_ENCRYPTED_AUTH_NUMBER = 0x03;
uint8_t AUTH_RESPONSE  = 0x10;
uint8_t AUTH_SUCCESS = 0x01;
uint8_t AUTH_FAIL = 0x04;
uint8_t AUTH_BYTE = 0x8;

uint8_t SECRET_KEY[18]  = {0x01, 0x08, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
const uint8_t SHORT_SECRET_KEY[16]  = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45};

const uint8_t PAIRING_KEY[18] = {0x01, 0x08, 0xC8, 0x2A, 0xE3, 0x40, 0xEB, 0x1A, 0x5F, 0x1A, 0x36, 0x33, 0x59, 0x4B, 0x2B, 0xA4, 0x43, 0x4A};
//
//first time run with a new mi band, change to 0 for full pairing
//
int authenticated = 1;
int amazfit_cor=0;
int important_alert=0;
//
uint8_t CONFIRM[2] = {0x02, 0x08};

const uint8_t AUTH_SERVICE_UUID[] = { 0x00, 0x07, 0x10, 0xaf, 0x09, 0x00, 0x18, 0x21, 0x12, 0x35, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00};

BLEClientService authService(0xFEE1);
BLEClientCharacteristic authNotif(AUTH_SERVICE_UUID);

//
BLEClientService       AlertNotifService(0x1811);
BLEClientCharacteristic NewAlertCharacteristic(0x2a46);

int Paired = 0;
//uint16_t conn_handle1;
//
// end mi
//

//
// peripheral
//
const uint8_t SERVICE_UUID[16] = {0x3, 0x4E, 0x53, 0xD7, 0xD1, 0xC4, 0x3C, 0x8A, 0x60, 0x40, 0x85, 0xC8, 0xE, 0xA9, 0x44, 0x63};
BLEService        cgms = BLEService(SERVICE_UUID);

const uint8_t SLOPE_CHARACTERISTIC_UUID[16]     = {0x1, 0x0, 0x0, 0x0, 0xD1, 0xC4, 0x3C, 0x8A, 0x60, 0x40, 0x85, 0xC8, 0xE, 0xA9, 0x44, 0x63}; //6344a90e-c885-4060-8a3c-c4d100000001
const uint8_t INTERCEPT_CHARACTERISTIC_UUID[16] = {0x2, 0x0, 0x0, 0x0, 0xD1, 0xC4, 0x3C, 0x8A, 0x60, 0x40, 0x85, 0xC8, 0xE, 0xA9, 0x44, 0x63};
const uint8_t BATTERY_CHARACTERISTIC_UUID[16]   = {0x3, 0x0, 0x0, 0x0, 0xD1, 0xC4, 0x3C, 0x8A, 0x60, 0x40, 0x85, 0xC8, 0xE, 0xA9, 0x44, 0x63};
const uint8_t ISIG_CHARACTERISTIC_UUID[16]      = {0x4, 0x0, 0x0, 0x0, 0xD1, 0xC4, 0x3C, 0x8A, 0x60, 0x40, 0x85, 0xC8, 0xE, 0xA9, 0x44, 0x63};

BLECharacteristic isig = BLECharacteristic(ISIG_CHARACTERISTIC_UUID);
BLECharacteristic slope = BLECharacteristic(SLOPE_CHARACTERISTIC_UUID);
BLECharacteristic intercept = BLECharacteristic(INTERCEPT_CHARACTERISTIC_UUID);
BLECharacteristic battery = BLECharacteristic(BATTERY_CHARACTERISTIC_UUID);

//
// end peripheral
//

#define FPU_EXCEPTION_MASK 0x0000009F

void setup()
{
  Serial.begin(115200);
  // Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 1
  Bluefruit.begin(1, 1);

  // Initialize Nffs
  Nffs.begin();
  readSlope();
  readIntercept();

  // Set max power. Accepted s are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit CGMS");

  // Callbacks for Peripheral
  Bluefruit.setConnectCallback(prph_connect_callback);
  Bluefruit.setDisconnectCallback(prph_disconnect_callback);

  setup_cc2500();
  // Set up and start advertising
  startAdv();
  setup_mi();
  setupCGMS();
  initReadings();
  Scheduler.startLoop(loop_cc2500);
  //this sets up nffs for the first time
  if (!authenticated) {
    writeSlope();
    writeIntercept();
  }
}

void setup_mi() {
  authService.begin();

  // set up callback for receiving measurement
  authNotif.setNotifyCallback(authNotif_callback);
  authNotif.begin();

  AlertNotifService.begin();
  NewAlertCharacteristic.begin();
  Bluefruit.Scanner.setRxCallback(scan_callback);


  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);
  Bluefruit.Central.setConnectCallback(cent_connect_callback);
  Bluefruit.Scanner.restartOnDisconnect(false);
  //Bluefruit.Scanner.filterRssi(-80);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
}

void setup_cc2500() {
  //need delay or cc2500 won't be ready
  delay(1000);
  pinMode(GDO0_PIN, INPUT);
  Serial.println("CC2500 Init Start");
  cc2500.init();
  Serial.println("CC2500 Init Done");
  memset(&packet, 0, sizeof(packet));
  OLD_ISIG = 0x00;
}

void setupCGMS(void)
{
  Serial.println("SetupCGMS");
  cgms.begin();
  isig.setProperties(CHR_PROPS_NOTIFY);
  isig.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  isig.setUserDescriptor("Isig");
  isig.setFixedLen(5);
  isig.setCccdWriteCallback(cccd_callback1);  // Optionally capture CCCD updates
  isig.begin();
  uint8_t hrmdata[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
  isig.notify(hrmdata, 5);                   // Use .notify instead of .write!
  //
  Serial.println("slope");
  uint8_t tmpValue[2] = {0x00, 0x00};
  slope.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  slope.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  slope.setUserDescriptor("Slope");
  slope.setFixedLen(2);
  slope.begin();
  slope.write(tmpValue, 2);
  //
  Serial.println("intercept");
  intercept.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  intercept.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  intercept.setUserDescriptor("Intercept");
  intercept.setFixedLen(2);
  intercept.begin();
  intercept.write(tmpValue, 2);
  //
  Serial.println("battery");
  battery.setProperties(CHR_PROPS_NOTIFY);
  battery.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  battery.setUserDescriptor("Battery");
  battery.setCccdWriteCallback(cccd_callback2);
  battery.setFixedLen(1);
  battery.begin();
  battery.notify(tmpValue, 1);
  Serial.println("Done SetupCGMS");
}

void cccd_callback1(BLECharacteristic& chr, uint16_t cccd_)
{
  // Display the raw request packet
  Serial.print("CCCD Updated: ");
  Serial.print(cccd_);
  Serial.println("");
}

void cccd_callback2(BLECharacteristic& chr, uint16_t cccd_)
{
  // Display the raw request packet
  Serial.print("CCCD Updated: ");
  Serial.print(cccd_);
  Serial.println("");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(cgms);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  peripheral_comm();

  /* Clear exceptions and PendingIRQ from the FPU unit */
  // See: https://devzone.nordicsemi.com/f/nordic-q-a/15243/high-power-consumption-when-using-fpu
  // possibly triggered by low level errors that I've since resolved
  // ex. closing a connection that was already closed
  // leaving this here anyway
  __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);
  waitForEvent();
  delay(2000);
}

void loop_cc2500()
{
  //cc2500
  Serial.println("CC2500 Listen");
  waitTime = RxData_RF();
  Serial.print("Next Start:");
  Serial.println(int(298 - waitTime) * 1000);
  firstTime = false;
  cc2500.SendStrobe(CC2500_CMD_SPWD);
  Serial.println("SendStrobe CC2500_CMD_SIDLE");
  cc2500.SendStrobe(CC2500_CMD_SIDLE);

  waitForEvent();
  delay(int(298) * 1000);
  Bluefruit.Scanner.stop();
  connecting = 0;
}

void peripheral_comm() {
  if (millis() - periph_notif_millis > 10000) {
    Serial.println("peripheral_comm");
    periph_notif_millis = millis();

    if (OLD_ISIG != ISIG) {
      OLD_ISIG = ISIG;
      missCount = 0;
      handle_isig();
    } else {
      missCount++;
    }

    if (missCount > 65) {
      missCount = 0;
      missedReadingsMsg();
    }
    if ( Bluefruit.connected() ) {

      //send isig to phone
      //only send these after we've received a value
      if (!firstTime) {
        uint8_t ch[5];
        ch[0] = txid;
        ch[1] = (int)((ISIG >> 24) & 0xFF) ;
        ch[2] = (int)((ISIG >> 16) & 0xFF) ;
        ch[3] = (int)((ISIG >> 8) & 0XFF);
        ch[4] = (int)((ISIG & 0XFF));

        if ( !isig.notify(ch, 5)) {
          Serial.println("isig characteristic update failed");
        };

        if (!battery.notify8(cgms_battery)) {
          Serial.println("battery characteristic update failed");
        };
      }
      //
      //get slope and intercept from phone
      //
      uint8_t slopevalue[2];
      slope.read(slopevalue, 2);
      int newSlope = slopevalue[1] | slopevalue[0] << 8;
      if ( newSlope != SLOPE && newSlope != 0) {
        Serial.print("Slope updated:");
        Serial.println(newSlope);
        SLOPE = newSlope;
        writeSlope();
      }

      uint8_t interceptValue[2];
      intercept.read(interceptValue, 2);
      int newIntercept = interceptValue[1] | interceptValue[0] << 8;
      if (newIntercept != INTERCEPT && newIntercept != 0) {
        Serial.print("Intercept updated:");
        Serial.println(newIntercept);
        INTERCEPT = newIntercept;
        writeIntercept();
      }
    }
    //send any new messages to the MI
    if (newValue ) {
      if (!Bluefruit.Central.connected() && !connecting) {
        Bluefruit.Scanner.start(20);
        Serial.println("Scanning ...");
      }
    }
  }
}

void missedReadingsMsg() {
  message[0] = 0x03;
  message[1] = 0x01;
  message[2] = 0x4d;//M
  message[3] = 0x69;//i
  message[4] = 0x73;//s
  message[5] = 0x73;//s
  message[6] = 0x20;
  message[7] = 0x20;
  message[8] = 0x20;
  message[9] = 0x20;
  message[10] = 0x20;
  message[11] = 0x20;

  message_len = 12;
  newValue = 1;
}

void handle_isig() {
  Serial.println("handle_isig");

  newValue = 0;
  Serial.print("ISIG:"); Serial.println(ISIG);
  Serial.print("INTERCEPT:"); Serial.println(INTERCEPT);
  Serial.print("SLOPE:"); Serial.println(SLOPE);
  GLUCOSE = ((ISIG - INTERCEPT) / SLOPE);
  addReading(ISIG, GLUCOSE);
  Serial.println(GLUCOSE);


  float Slope = getSlopeGlucose();
  int timeToLimit = 0;
  char c_glucose[10];

  // what glucose MIGHT be right now, assuming 15 minute delay
  EST_GLUCOSE = GLUCOSE + (Slope * 15);

  //stop estimating if it's really low or really high.
  if (EST_GLUCOSE < 40 || EST_GLUCOSE > 300) {
    EST_GLUCOSE = GLUCOSE;
  }

  //rising, how long until 180
  if (Slope > 0 && GLUCOSE < 180) {
    timeToLimit = abs((180 - GLUCOSE) / Slope);
    //since the dex is ~15 minutes behind reality
    timeToLimit = timeToLimit - 15;
  }

  //falling, how long until 80
  if (Slope < 0 && GLUCOSE > 80) {
    timeToLimit = abs((GLUCOSE - 80) / Slope);
    timeToLimit = timeToLimit - 15;
  }

  if (timeToLimit < 0) {
    timeToLimit = 1;
  }

  if (timeToLimit > 99) {
    timeToLimit = 0;
  }

  Serial.print("AlertCount:"); Serial.println(alertCount);

  uint8_t msgType = 0x05;  //double vibrate then go away
  message[2] = 0xFF;
  message[3] = 0x20;
  message[4] = 0x20;
  message[5] = 0x20;
  message[6] = 0x20;
  message[7] = 0x20;
  message[8] = 0x20;
  message[9] = 0x20;
  message[10] = 0x20;
  message[11] = 0x20;

  //
  if (EST_GLUCOSE < 80  && alertCount == 0 && EST_GLUCOSE > 65) {
    if (alertCount == 0) {
      msgType = 0x03;
      alertCount++;
    } else {
      msgType = 0x05;
      alertCount++;
      if (alertCount >= 6) {
        alertCount = 0;
      }
    }
  }

  if (EST_GLUCOSE < 60) {
    if (alertCount == 0) {
      msgType = 0x03;
      alertCount++;
    } else {
      msgType = 0x05;
      alertCount++;
      if (alertCount >= 4) {
        alertCount = 0;
      }
    }
  }

  if (EST_GLUCOSE > 180 ) {
    if (alertCount == 0) {
      msgType = 0x03;
      alertCount++;
    } else {
      msgType = 0x05;
      alertCount++;
      if (alertCount >= 24) {
        alertCount = 0;
      }
    }
  }

  if (EST_GLUCOSE > 80 && EST_GLUCOSE < 180) {
    alertCount = 0;
    msgType = 0x05;
  }

  if (abs(Slope) >= 3) {
    msgType = 0x03;
  }


  sprintf(c_glucose, "%d", EST_GLUCOSE);
  message[2] = c_glucose[0];
  message[3] = c_glucose[1];
  if (strlen(c_glucose) > 2) {
    message[4] = c_glucose[2];
  }

  if ((EST_GLUCOSE < 90 || EST_GLUCOSE > 160) || (sqrt(pow(Slope, 2)) > 1.5)) {
    showReadings = 0;
    if (timeToLimit == 0 || timeToLimit == 1) {
      // to do add slope to message
      if (abs(Slope) > 0.1) {
        float value = abs(Slope);
        int left_part, right_part;
        char buffer[50];
        sprintf(buffer, "%2.1lf", value);
        sscanf(buffer, "%d.%d", &left_part, &right_part);
        sprintf(c_glucose, "%d", left_part);
        message[6] = c_glucose[0];
        message[7] = 0x2e; //decimal point
        sprintf(c_glucose, "%d", right_part);
        message[8] = c_glucose[0];
      }
    } else {
      message[6] = 0x20;
      sprintf(c_glucose, "%d", timeToLimit);
      message[7] = c_glucose[0];
      message[8] = c_glucose[1];
      message[9] = 0x20;
    }
  } else {
    //send notification only once
    if (showReadings != 1) {
      //send Good
      message[2] = 0x47;
      message[3] = 0x6F;
      message[4] = 0x6F;
      message[5] = 0x64;
      showReadings = 1;
    } else {
      message[2] = 0xff;
    }
  }

  if (readings_arr[0].glucose != 0 && readings_arr[1].glucose != 0) {
    if (abs(readings_arr[0].glucose - readings_arr[1].glucose) > 25) {
      message[11] = 0x3f;  //?
    }
  }

  if (Slope > 0) {
    message[10] = 0x2b; //+
  };
  if (Slope < 0) {
    message[10] = 0x2d; //-
  }

  Serial.print("Slope:"); Serial.println(Slope);
  Serial.print("TimeToLimit:"); Serial.println(timeToLimit);

  //dont do sms here
  if (amazfit_cor){
    if (msgType==0x03){
      important_alert=1;
      }
   msgType = 0x05;
  }

  message[0] = msgType;
  message[1] = 0x01;

  if (message[2] != 0xff) {
    newValue = 1;
    message_len = 12;
  }
}


void addReading(long rawcounts, int glucose) {

  Serial.println("addReading");

  //move everything in the array over 1 place
  //then add the new values
  for (int i = 2; i > 0; i-- ) {
    readings_arr[i].seconds = readings_arr[i - 1].seconds;
    readings_arr[i].rawcounts = readings_arr[i - 1].rawcounts;
    readings_arr[i].glucose = readings_arr[i - 1].glucose;
  }

  readings_arr[0].seconds = millis() / 1000;
  readings_arr[0].rawcounts = rawcounts;
  readings_arr[0].glucose = glucose;

}

float getSlopeGlucose() {
  Serial.println("getSlopeGlucose");
  int counter = 0;
  float sumx = 0.0, sumy = 0.0, sum1 = 0.0, sum2 = 0.0;

  for (int i = 0; i < 3; i++ ) {
    if (readings_arr[i].glucose > 20) {
      counter++;
      sumx = sumx + (float)(readings_arr[i].seconds / 60);
      sumy = sumy + (float)(readings_arr[i].glucose);
    }
  }

  float xmean = sumx / counter;
  float ymean = sumy / counter;

  for (int i = 0; i < counter; i ++) {
    if (readings_arr[i].glucose > 20) {
      sum1 = sum1 + ( (float)(readings_arr[i].seconds / 60) - xmean) * ((float)(readings_arr[i].glucose) - ymean);
      sum2 = sum2 + pow(((float)(readings_arr[i].seconds / 60) - xmean), 2);
    }
  }

  // derive the least squares equation
  if (sum2 == 0) {
    return 0;
  }
  Serial.print(sum1); Serial.print(":"); Serial.println(sum2);
  return sum1 / sum2;
};


/*------------------------------------------------------------------*/
/* Peripheral
  ------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  Serial.println("prph_scan_callback ... ");
  char peer_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, peer_name, sizeof(peer_name));
  periph_connected = 1;
  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);

}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  periph_connected = 0;
  Serial.println();
  Serial.println("[Prph] Disconnected");
}

//
//cc2500
//
void swap_channel(uint8_t channel, uint8_t newFSCTRL0)
{
  cc2500.WriteReg(REG_CHANNR, channel);
  delay(SPI_DELAY_LOCAL);
  cc2500.WriteReg(REG_FSCTRL0, newFSCTRL0);

  cc2500.SendStrobe(CC2500_CMD_SRX);
  delay(SPI_DELAY_LOCAL);
  Serial.println("ReadStatusReg(REG_MARCSTATE)");
}

long RxData_RF(void)
{
  int Delay = 0;
  int packetFound = 0;
  int channel = 0;
  int crc = 0;
  int lqi = 0;
  uint8_t PacketLength;
  uint8_t freqest;
  Dexcom_packet Pkt;
  long timeStart = 0;
  int continueWait = false;
  Serial.print("Start Listening:");
  Serial.println(millis());
  digitalWrite(LED_RED, HIGH);   // turn the LED on (HIGH is the voltage level)
  cc2500_recv_time = 0;
  while (!packetFound && channel < 4) {
    //
    continueWait = false;
    Serial.println("Swap_channel");
    swap_channel(nChannels[channel], fOffset[channel]);
    timeStart = millis();
    //
    Serial.print("Channel:");
    Serial.println(channel);

    //wait on this channel until we receive something
    //if delay is set, wait for a short time on each channel
    Serial.print("Delay:");
    Serial.println(Delay);
    while (!digitalRead(GDO0_PIN) && (millis() - timeStart < Delay) || (!digitalRead(GDO0_PIN) && Delay == 0))
    {
      delay(1);
      //if (periph_connected) {
      //  peripheral_comm();
      //}
    }
    if (digitalRead(GDO0_PIN)) {
      Serial.println(digitalRead(GDO0_PIN));
      Serial.println("Got packet");
      Serial.println("ReadReg CC2500_REG_RXFIFO");
      PacketLength = cc2500.ReadReg(CC2500_REG_RXFIFO);
      //
      Serial.print("Wait:");
      Serial.println(millis() - timeStart);
      Serial.println(PacketLength);
      //if packet length isn't 18 skip it
      if (PacketLength == 18) {
        //keep the values around in oldPacket so there's something to return over BLE
        //if packet capture fails crc check
        memcpy(oldPacket, packet, 18);
        //
        Serial.println("ReadBurstReg CC2500_REG_RXFIFO");
        cc2500.ReadBurstReg(CC2500_REG_RXFIFO, packet, PacketLength);
        memcpy(&Pkt, packet, 18);
        //you should have
        //first byte, not in this array, packet length (18)
        //packet#  sample  comment
        //0        FF      dest addr
        //1        FF      dest addr
        //2        FF      dest addr
        //3        FF      dest addr
        //4        CA      xmtr id
        //5        4C      xmtr id
        //6        62      xmtr id
        //7        0       xmtr id
        //8        3F      port
        //9        3       hcount
        //10       93      transaction id
        //11       93      raw isig data
        //12       CD      raw isig data
        //13       1D      filtered isig data
        //14       C9      filtered isig data
        //15       D2      battery
        //16       0       fcs(crc)
        //17       AD      fcs(crc)
        //
        //
        Serial.println("ReadStatusReg LQI");
        Pkt.LQI = cc2500.ReadStatusReg(REG_LQI);
        crc = Pkt.LQI & 0x80;

        //packet is good
        if (crc == 128) {
          cc2500_recv_time = millis();
          //packet has the correct transmitter id
          Serial.print(packet[4], HEX);
          Serial.print(":");
          Serial.print(packet[5], HEX);
          Serial.print(":");
          Serial.print(packet[6], HEX);
          Serial.print(":");
          Serial.println(packet[7], HEX);
          //
          Serial.print(packet[8], HEX);
          Serial.print(":");
          Serial.print(packet[9], HEX);
          Serial.print(":");
          Serial.print(packet[10], HEX);
          Serial.print(":");
          Serial.print(packet[11], HEX);
          Serial.print(":");
          Serial.print(packet[12], HEX);
          Serial.print(":");
          Serial.print(packet[13], HEX);
          Serial.print(":");
          Serial.println(packet[14], HEX);
          //
          Serial.print("Battery:");
          Serial.println(packet[15], HEX);
          cgms_battery = packet[15];
          txid = packet[10];

          Serial.println("ReadStatusReg RSSI");
          Pkt.RSSI = cc2500.ReadStatusReg(REG_RSSI);
          lqi = (int)(Pkt.LQI & 0x7F);
          Serial.print("RSSI:");

          if ((int)Pkt.RSSI >= 128)
            Serial.println((((int)Pkt.RSSI - 256) / 2 - 73), DEC);
          else
            Serial.println(((int)Pkt.RSSI / 2 - 73), DEC);

          //
          convertFloat();
          freqest = cc2500.ReadStatusReg(REG_FREQEST);
          Serial.print("Old Offset:");
          Serial.println(fOffset[channel], DEC);
          fOffset[channel] = fOffset[channel] + freqest;
          Serial.print("Offset:");
          Serial.print(fOffset[channel], DEC); Serial.print(":");
          Serial.print(freqest, DEC); Serial.print(":"); Serial.println(freqest, HEX);

          packetFound = 1;
          break;
        } else {
          memcpy(packet, oldPacket, 18);
          continueWait = false;
          Serial.println("CRC Failed");
        }
      } else {
        memcpy(packet, oldPacket, 18);
        continueWait = true;
        Serial.println("Packet length issue");
        //go to next channel and camp out there
        if (channel < 3) {
          channel++;
        } else {
          channel = 0;
        }
        Delay = 0;
      }  //packet length=18
    }

    if (!continueWait) {
      channel++;
      Delay = 600;
    }

    // Make sure that the radio is in IDLE state before flushing the FIFO
    // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
    Serial.println("SendStrobe CC2500_CMD_SIDLE");
    cc2500.SendStrobe(CC2500_CMD_SIDLE);
    delay(SPI_DELAY_LOCAL);

    // Flush RX FIFO
    Serial.println("SendStrobe CC2500_CMD_SFRX");
    cc2500.SendStrobe(CC2500_CMD_SFRX);
  }

  // Make sure that the radio is in IDLE state before flushing the FIFO
  // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
  Serial.println("SendStrobe CC2500_CMD_SIDLE");
  cc2500.SendStrobe(CC2500_CMD_SIDLE);
  delay(SPI_DELAY_LOCAL);

  // Flush RX FIFO
  Serial.println("SendStrobe CC2500_CMD_SFRX");
  cc2500.SendStrobe(CC2500_CMD_SFRX);

  Serial.print("End Listening:");
  Serial.println(millis());

  digitalWrite(LED_RED, LOW);
  //add one additional second(per channel) to the delay
  return channel - 1;
}// Rf RxPacket

void cc2500_recv() {
  Serial.println ("cc2500_recv interrupt");
}

//String based method to turn the ISIG packets into a binary string
//also need to add back the left hand zeros if any
//much more elegant method in dexterity, but on arduino I get a bad value every 24 hours or so
//due to (assumed)loss of leading zeros
void convertFloat() {
  String result;
  result = lpad(String(packet[11], BIN), 8);
  result += lpad(String(packet[12], BIN), 8);
  result += lpad(String(packet[13], BIN), 8);
  result += lpad(String(packet[14], BIN), 8);

  char ch[33];
  char reversed[31];
  result.toCharArray(ch, 33);

  int j = 0;
  for (int i = 31; i >= 0; i--) {
    reversed[j] = ch[i];
    j++;
  }

  String reversed_str = String(reversed);
  String exp1 = reversed_str.substring(0, 3);
  int exp1_int = (int)binStringToLong(exp1);

  String mantissa1 = reversed_str.substring(3, 16);
  long mantissa1_flt = binStringToLong(mantissa1);

  String exp2 = reversed_str.substring(16, 19);
  int exp2_int = (int)binStringToLong(exp2);

  String mantissa2 = reversed_str.substring(19, 32);
  long mantissa2_flt = binStringToLong(mantissa2);


  rawcount1 = (mantissa1_flt * pow(2, exp1_int) * 2);
  rawcount2 = mantissa2_flt * pow(2, exp2_int);

  Serial.print("Raw ISIG:");
  Serial.print(rawcount2);
  Serial.print(":");
  Serial.println(rawcount1);

  //unfiltered value
  ISIG = rawcount2;
}

//add zeros back to a string representation of a binary number
//ex.  111 should be 00000111 or the calculations in convertFloat
//would go haywire
String lpad(String str, int length) {
  String zeroes;
  int len = length - str.length();
  while (len > 0) {
    zeroes = zeroes + "0";
    len--;
  }
  return zeroes + str;
}
//
// end cc2500
//

//
// start mi
//
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  uint8_t len = 0;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  //prevent multiple simultaneous connection attempts
  if (! Bluefruit.Central.connected() && !connecting) {
    //mi band is FA:AB:33:E3:12:2D
    Serial.println(report->peer_addr.addr[5],HEX);
    if (report->peer_addr.addr[5] == 0xFB) {  //amazfit cor
        //if (report->peer_addr.addr[5] == 0xFA) {  //dons mi
      //if (report->peer_addr.addr[5] == 0xF8) {  //karins
      // Connect to device
      Serial.println("Found device");
      connecting = 1;
      Bluefruit.Central.connect(report);
      (void) report;
    }
  }
}

void cent_connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  connecting = 0;
  //conn_handle1 = conn_handle;
  if (AlertNotifService.discover(conn_handle)) {
    Serial.println("AlertNotifService discovered");
  }
  if (NewAlertCharacteristic.discover()) {
    Serial.println("NewAlertCharacteristic discovered");
  }

  if (authService.discover(conn_handle) )
  {
    Serial.println("Found Auth Service");
    if (authNotif.discover()) {
      Serial.println("AuthNotif found");

      if (authNotif.enableNotify()) {
        Serial.println("Nofitications enabled");

        //this starts the pairing request and the mi band will buzz and expect you to push the button
        if (authenticated) {
          if ( authNotif.write(CONFIRM, 2) )
          {
            Serial.println("Request Random");
          } else
          {
            Serial.println("Request Random Failed");
          }
        } else {
          if ( authNotif.write(SECRET_KEY, 18) )
          {
            Serial.println("Key Sent");
          } else
          {
            Serial.println("Alert Failed");
          }
        }

      } else {
        Serial.println("Notify enable Failed");
      }
    }
  }
}

void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Paired = 0;
  connecting = 0;
  Serial.println("Disconnected");
}

void authNotif_callback(BLEClientCharacteristic * chr, uint8_t* data, uint16_t len)
{
  Serial.print("AuthNotif: ");

  for (int i = 0; i < len; i++) {
    Serial.print(data[i], HEX); Serial.print(":");
  }
  Serial.println("");

  //step 1 of paring
  if (data[0] == 0x10 && data[1] == 0x01 && data[2] == 0x01) {
    Serial.println("Next Part");

    if ( authNotif.write(CONFIRM, 2) )
    {
      Serial.println("Confirm Sent");
    } else
    {
      Serial.println("Confirm Failed");
    }
  }

  //Step 2 of pairing
  if (data[0] == 0x10 && data[1] == 0x02 && data[2] == 0x01) {
    Serial.println("This is the key");

    //AES encryption
    byte plain[] = {data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19]}; //16 chars == 16 bytes
    crypto_feed_watchdog();
    aes128.setKey(SHORT_SECRET_KEY, aes128.keySize());
    aes128.encryptBlock(buffer, plain);
    Serial.println("AES ECB Encrypted");

    enc_buffer[0] = 0x03; enc_buffer[1] = 0x08;
    for (int i = 0; i < 16; i++) {
      enc_buffer[i + 2] = buffer[i];
    }

    if ( authNotif.write(enc_buffer, 18) )
    {
      Serial.println("encrypted key Sent");
    } else
    {
      Serial.println("encrypted key Failed");
    }
  }

  //Step 3 of pairing
  if (data[0] == 0x10 && data[1] == 0x03 && data[2] == 0x01) {
    Paired = 1;
    Serial.println("Paired");
    authenticated = 1;
    Serial.println("Write message");
    Serial.println(GLUCOSE);

    if (newValue) {
      for (int i = 0; i < message_len; i++) {
        Serial.print(message[i], HEX); Serial.print(":");
      }

      NewAlertCharacteristic.write_resp( message, 12 ) ;
      newValue = 0;
    //} else {
    //  Serial.println("Already sent, skip");
    //  Bluefruit.Scanner.stop();
    }

    Serial.print("Battery ");

    int vbat = readVBAT();
    Serial.println(vbat);
    if (vbat < 2900  && lowBatt == 0) {
      delay(1000);
      message[0] = 0x03;
      message[1] = 0x01;
      message[2] = 0x42;//B
      message[3] = 0x61;//a
      message[4] = 0x74;//t
      message[5] = 0x74;//t
      delay(3000);
      NewAlertCharacteristic.write_resp( message, 6) ;
      lowBatt == 1;
    }
    
    //2200 never alerts
    if (vbat > 2900  && lowBatt == 1) {
      //reset although plugging in to charge probably cleared this anyway
      lowBatt == 0;
    }
  }
}

long binStringToLong(String binary) {
  long result = 0;
  long power = 1;
  char ch[14];

  binary.toCharArray(ch, binary.length() + 1);
  for (int j = binary.length() - 1; j >= 0; j--) {
    if (ch[j] == '1') {
      result = result + power;
    }
    power = power * 2;
  }
  return result;
}

void initReadings() {
  reading.seconds = 0;
  reading.glucose = 0;
  reading.rawcounts = 0;
  for (int i = 0; i < 3; i++ ) {
    readings_arr[i].seconds = reading.seconds;
    readings_arr[i].glucose = reading.glucose;
    readings_arr[i].rawcounts = reading.rawcounts;
  }
}

// Write the state to the local file system
void writeSlope() {
  NffsFile file;
  if (file.open(SLOPE_FILENAME, FS_ACCESS_WRITE)) {
    char buffer[12];
    itoa(SLOPE, buffer, 10);
    Serial.println();

    for (int i = 0; i < 3; i++) {
      Serial.print(buffer[i]);
    }
    Serial.println();
    file.write(buffer, sizeof(buffer));
    file.close();
  }
}

// Read the state from the local file system
void readSlope() {
  NffsFile file;
  int value = 1; // set the default to return
  if (Nffs.testFile(SLOPE_FILENAME)) {
    file.open(SLOPE_FILENAME, FS_ACCESS_READ);
    if (file.exists()) { //yes, we did just test this, but double check
      Serial.println("Value from " SLOPE_FILENAME);
      uint32_t readLen;
      char buffer[12] = { 0 }; // buffer starts as an empty char array (C string)
      readLen = file.read(buffer, 10);
      buffer[readLen] = 0; // drop any last character and make sure the buffer contains a C string
      value = atoi(buffer); // convert the string in the file to a number
      SLOPE = value;
      Serial.println(value);
      file.close();
    }
  } else {
    Serial.println("State file does not yet exist");
  }
}

void writeIntercept() {
  NffsFile file;
  if (file.open(INTERCEPT_FILENAME, FS_ACCESS_WRITE)) {
    char buffer[12];
    itoa(INTERCEPT, buffer, 10);
    Serial.println();
    for (int i = 0; i < 4; i++) {
      Serial.print(buffer[i]);
    }
    Serial.println();
    file.write(buffer, sizeof(buffer));
    file.close();
  }
}

// Read the state from the local file system
void readIntercept() {
  NffsFile file;
  int value = 1; // set the default to return
  if (Nffs.testFile(INTERCEPT_FILENAME)) {
    file.open(INTERCEPT_FILENAME, FS_ACCESS_READ);
    if (file.exists()) { //yes, we did just test this, but double check
      Serial.println("Value from " INTERCEPT_FILENAME);
      uint32_t readLen;
      char buffer[12] = { 0 }; // buffer starts as an empty char array (C string)
      readLen = file.read(buffer, 10);
      buffer[readLen] = 0; // drop any last character and make sure the buffer contains a C string
      value = atoi(buffer); // convert the string in the file to a number
      INTERCEPT = value;
      Serial.println(value);
      file.close();
    }
  } else {
    Serial.println("State file does not yet exist");
  }
}

int readVBAT(void) {
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

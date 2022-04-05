#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/adc.h"
#include "soc/sens_reg.h"
#include <soc/sens_struct.h>
#include <LiquidCrystal.h>
#include "peripherals.h"
#include "sound_data.h"
#include "XT_DAC_Audio.h"
#include "BluetoothA2DPSink.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define debug(x)     Serial.print(x)
#define debugln(x)   Serial.println(x)
#define addressIndex 0

XT_Wav_Class zero(b1);
XT_Wav_Class one(b2);
XT_Wav_Class two(b3);
XT_Wav_Class three(b4);
XT_Wav_Class four(b5);
XT_Wav_Class five(b6);
XT_Wav_Class six(b7);
XT_Wav_Class seven(b8);

// date time
// Network credentials
const char *ssid     = "NSUT-Campus";
const char *password = "";
//Week Days
const String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//Month names
const String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

XT_DAC_Audio_Class DacAudio(25, 0);
XT_Sequence_Class Sequence;

hw_timer_t* timerF = NULL;
portMUX_TYPE timerMuxADC = portMUX_INITIALIZER_UNLOCKED;

BluetoothA2DPSink a2dp_sink;
bool bluetooth_started = false;
const char* bluetooth_name = "NSUTB";

static const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = 44100, // corrected by info from bluetooth
        .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
};

String to_string(int n){
  if(n<10)
    return "0" + String(n);
  
  return String(n);
}

void print_temperature(float temp_input){
  String temp_string = String(temp_input, 0) + String(char(223)) + "C";
  uint8_t len = temp_string.length();
  printStr(temp_string, 19 - len, 1);
}

void IRAM_ATTR offtime() {
  cnt = true;
}

void bitReverse(int* X, int M);
void copy_array(struct complex* out, float* inp, int M);
void fft(struct complex* Y, int M, struct complex* w);
void absolute(struct complex* inp, int M, float* out);
void normalize(float* input_abs_data);
void fft_data();
void twid(struct complex* Y, int M);
void display_data();
float to_temperature();
void adc_init_aux();
void adc_init_mic();
void adc_read();
void lcd_init();
void printStr(String text, int index, int line);
void printChr(int character, int index, int line);
void clearLine(int line);
float ewma() {
  current_ip_tt = lambda_tt * val_tt * 1.0 + (1 - lambda_tt) * previous_ip_tt;
  previous_ip_tt = current_ip_tt;
  return previous_ip_tt;
}
void dac_init();
int time_init(){
      // Connect to Wi-Fi
  printStr("Connecting to ",0,0);
  printStr(String(ssid),0,1);
  WiFi.begin(ssid, password);
  longp = millis();
  while (millis() - longp < wifi_timeout) {
    for(int i=0;i<4;i++)
    {
      delay(500);
      printStr(".",i,2);
      if(WiFi.status() == WL_CONNECTED)
      {
        timeClient.begin();
        timeClient.setTimeOffset(19800);
        return 0;
      }
    }
  }
}
void setup() {
  // put your setup code here, to run once:
  lcd_init();
  adc_init_mic();
  dac_init();

    time_init();

    timerF = timerBegin(2, 2, true);      //40 Mhz clock
    timerAttachInterrupt(timerF, &offtime, true);
    // Fire Interrupt at 10k - fs
    timerAlarmWrite(timerF, 40000000 / fft_fs, true);
    timerAlarmEnable(timerF);
    bitReverse(bit_reverse, fft_size);
    twid(twiddle, fft_size);

  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(aud_trig, INPUT);
  delay(1000);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly
  if (current_location == HOME_TEMP) {
    timerAlarmDisable(timerF);
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();
    String formattedTime = timeClient.getFormattedTime();
    int hh = timeClient.getHours();
    int mm = timeClient.getMinutes(); 
    String weekDay = weekDays[timeClient.getDay()];
    //Get a time structure
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon+1;
    String currentMonthName = months[currentMonth-1];
    int currentYear = ptm->tm_year+1900;
    //Print complete date:
    String currentDate = to_string(currentYear) + "-" + to_string(currentMonth) + "-" + to_string(monthDay);
    String currentTime = to_string(hh) + ":" + to_string(mm);
    temperature = to_temperature();

    lcd.clear();

    printStr("Temperature: ", 1, 1);
    printStr("Time: ", 1, 2);
    printStr("Date: ", 1, 3);

    if(WiFi.status() == WL_CONNECTED)
    { 
      printStr(currentTime, 14, 2);
      printStr(currentDate, 9, 3);
    }
    else
    {
      printStr("-", 14, 2);
      printStr("-", 14, 3);
    }

    if(!sensor_nc){
      print_temperature(temperature);
    }
    else
    {
      printStr("-", 14, 1); // Sensor not found
    }
    
    if(!digitalRead(up)){
      timerAlarmEnable(timerF);
      delay(10);
      longp = millis();
      while(!digitalRead(up)){
          if(millis()-longp > delay_to_switch){
            lcd.clear();
            printStr("MIC FFT MODE", 3 ,0);
            current_location = MIC_FFT;
            delay(500);
            lcd.clear();
          }
      }
      delay(10);
    }
    delay(2000);
  }
  else if ((current_location == MIC_FFT) || (current_location == AUX_FFT)) {
    timerAlarmEnable(timerF);
    if (cnt) {
      if (current_location == MIC_FFT) {
        adc_init_mic();
      }
      else if (current_location == AUX_FFT) {
        adc_init_aux();
      }
      adc_read();
      fft_in[point] = float(adc_reading - 1950) / 4096;
      point++;
      if (point == fft_size) {
        fft_data();
        point = 0;
        display_data();
        if (!digitalRead(up)) {
          delay(10);
          screen++;
          screen %= 8;
          longp = millis();
          while (!digitalRead(up)) {
            if (millis() - longp > delay_to_switch) {
              if (current_location == AUX_FFT) {
                current_location = LOON;
                screen = 0;
                lcd.clear();
                printStr("Balloon Target Mode", 0, 0);
              }
              else if (current_location == MIC_FFT) {
                current_location = AUX_FFT;
                screen = 0;
                lcd.clear();
                printStr("AUX FFT Mode", 3, 0);
                delay(1000);
                lcd.clear();
              }
              break;
            }
          }
          delay(10);
        }
        if (!digitalRead(down)) {
          delay(10);
          lcd.clear();
          lcd.setCursor(8, 1);
          longp = millis();
          while (!digitalRead(down)) {
            printStr(String(156.25 * screen * 16, 0), 3, 1);
            printStr("-", 9, 1);
            printStr(String(156.25 * (screen + 1) * 16, 0), 11, 1);
            if (millis() - longp > 1500) {
              screen++;
              screen %= 8;
              longp = millis();
              clearLine(1);
            }
          }
          delay(10);
          lcd.clear();
        }
        if (current_location == MIC_FFT || current_location == AUX_FFT) {
          timerAlarmEnable(timerF);
          for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 16; j++) {
              lcd.setCursor(j + 2, i);
              if (display_xy[i][j + (16 * screen)] == 0) {
                lcd.print(" ");
              }
              else {
                lcd.write(byte(display_xy[i][j + (16 * screen)]) - 1);
              }
            }
          }
        }
      }
      cnt = false;
    }
  }
  else if (current_location == LOON) {
    timerAlarmEnable(timerF);
    if (!digitalRead(up)) {
      delay(10);
      longp = millis();
      while (!digitalRead(up)) {
        if (millis() - longp > delay_to_switch) {
          lcd.clear();
          current_location = TALKING_TOM;
          printStr("Talkative Tom mode", 1, 0);
          adc_init_mic();
        }
      }
      delay(10);
    }
    DacAudio.FillBuffer();
    curr_state = digitalRead(aud_trig);
    if (curr_state != prev_state) {

      Sequence.RemoveAllPlayItems();
      randNumber = random(0, 8);
      switch (randNumber) {
      case 0:
        Sequence.AddPlayItem(&zero);
        DacAudio.Play(&Sequence);
        break;
      case 1:
        Sequence.AddPlayItem(&one);
        DacAudio.Play(&Sequence);
        break;
      case 2:
        Sequence.AddPlayItem(&two);
        DacAudio.Play(&Sequence);
        break;
      case 3:
        Sequence.AddPlayItem(&three);
        DacAudio.Play(&Sequence);
        break;
      case 4:
        Sequence.AddPlayItem(&four);
        DacAudio.Play(&Sequence);
        break;
      case 5:
        Sequence.AddPlayItem(&five);
        DacAudio.Play(&Sequence);
        break;
      case 6:
        Sequence.AddPlayItem(&six);
        DacAudio.Play(&Sequence);
        break;
      case 7:
        Sequence.AddPlayItem(&seven);
        DacAudio.Play(&Sequence);
        break;
      default:
        Sequence.RemoveAllPlayItems();
        break;
      }
      prev_state = curr_state;
    }
  }
  else if (current_location == TALKING_TOM) {
    timerAlarmEnable(timerF);
    // debugln("Talkative Tom Mode");
    /**
     * @brief Speech detect
     *
     */
    float sx = ewma();
    while (sx < 30) {
      sx = ewma();
      adc_read();
      debugln(sx);
      val_tt = abs((adc_reading >> 4) - 125);
      if(!digitalRead(up))
        break;
    }
    timerAlarmWrite(timerF, 40000000 / fft_fs_tt, true);
    /**
     * @brief Record the samples
     *
     */
    while (ewma() > 0.22 && i_tt < max_size_tt) {
      if (cnt) {
        adc_read();
        buff[i_tt] = adc_reading >> 4;
        i_tt++;
        cnt = false;
      }
      if (i_tt % 750 == 0) {
        val_tt = abs((adc_reading >> 4) - 125);
      }
    }
    rec_index_tt = i_tt;
    i_tt = 0;
    timerAlarmWrite(timerF, 40000000 / (25000), true);
    while (i_tt < rec_index_tt) {
      if (cnt) {
        SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, buff[i_tt] / 4 , RTC_IO_PDAC1_DAC_S);
        i_tt++;
        cnt = false;
      }
    }
    i_tt = 0;
    for(int i=0;i<max_size_tt;i++)
      buff[i] = 0;
    val_tt = 0;
    adc_reading = 0;

  if (!digitalRead(up)) 
    {
      delay(10);
      longp = millis();
      while (!digitalRead(up))
      {
        if (millis() - longp > delay_to_switch) 
        {
          lcd.clear();
          current_location = HOME_TEMP;
        }
      }
      delay(10);
    }
  }
}

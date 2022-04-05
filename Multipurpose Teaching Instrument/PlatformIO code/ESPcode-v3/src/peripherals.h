#define LCD_RESET_PIN 19 
#define LCD_ENABLE_PIN 18
#define LCD_DATA_PIN4 4
#define LCD_DATA_PIN5 21
#define LCD_DATA_PIN6 17
#define LCD_DATA_PIN7 5
#define aux 33
#define mic 32
#define up 22
#define down 15
#define thermistor 34
#define aud_trig 16

//talkative tom
uint16_t fft_fs_tt = 20000;
const int32_t max_size_tt = 55000;
byte buff[max_size_tt] = { 0 };
int i_tt, rec_index_tt = 0;
float current_ip_tt = 0;
float previous_ip_tt = 0;
float lambda_tt = 0.01; //0.017
uint8_t val_tt = 0;
//
bool mic_aux = false;
bool curr_state = 0;
bool prev_state = 0;
long randNumber;
bool sensor_nc = false;
long unsigned int longp = 0;
uint8_t screen = 0;
int display_xy[4][128] = { 0 };
volatile uint16_t adc_reading = 0;
static const adc1_channel_t ADC_channel = ADC1_CHANNEL_5;
static const adc1_channel_t ADC_channel_2 = ADC1_CHANNEL_4;
uint16_t point = 0;
volatile bool cnt = false;
const uint16_t fft_max_size = 256;
uint16_t fft_size = 256;
int length_r = (fft_size / 2) + 1;
uint16_t fft_fs = 40000;
float fft_in[fft_max_size], samples_abs[fft_max_size], samples_norm[fft_max_size] = { 0 };
float temp_real, temp_imag;
int steps, lower, upper, skip;
struct complex {
  float real, imag;
};
struct complex twiddle[fft_max_size >> 1];
struct complex samples[fft_max_size] = {};
int bit_reverse[fft_max_size];
uint16_t i, j;
enum location { HOME_TEMP, MIC_FFT, AUX_FFT, LOON, TALKING_TOM, RADIO, BLUETOOTH};
location current_location = HOME_TEMP;
float R2 = 10000;
float logR1, R1;
float temp, tempF = 0;
float c1 = 0.001125308852122;
float c2 = 0.000234711863267;
float c3 = 0.000000085663516;
float temperature = 0;
uint16_t delay_to_switch = 2000;
uint16_t wifi_timeout = 30000;

float to_temperature() {
  temp = 0;
  sensor_nc = false;
  for (int i = 0; i < 256;i++) {
    temp = temp + analogRead(thermistor);
  }
  temp = temp / 256;
  if (temp > 3900) {
    sensor_nc = true;
  }
  R1 = R2 / ((4096 / (float)temp) - 1);
  logR1 = log(R1);
  tempF = (1.0 / (c1 + c2 * logR1 + c3 * logR1 * logR1 * logR1));
  tempF = tempF - 273.15;
  return tempF;
}

LiquidCrystal lcd(LCD_RESET_PIN, LCD_ENABLE_PIN, LCD_DATA_PIN4, LCD_DATA_PIN5, LCD_DATA_PIN6, LCD_DATA_PIN7);

byte a[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
};
byte b[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
};
byte c[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
};
byte d[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte e[] = {
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte f[] = {
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte g[] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte h[] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
void adc_init_aux() {
  pinMode(aux, INPUT);
  adcAttachPin(aux);
  analogSetAttenuation(ADC_11db);
  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << ADC_channel), SENS_SAR1_EN_PAD_S);
}
void adc_init_mic() {
  pinMode(mic, INPUT);
  adcAttachPin(mic);
  analogSetAttenuation(ADC_11db);
  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << ADC_channel_2), SENS_SAR1_EN_PAD_S);
}
void adc_read() {
  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0);
  adc_reading = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
}
void lcd_init() {
  lcd.begin(20, 4);
  lcd.clear();
  lcd.createChar(0, a);
  lcd.createChar(1, b);
  lcd.createChar(2, c);
  lcd.createChar(3, d);
  lcd.createChar(4, e);
  lcd.createChar(5, f);
  lcd.createChar(6, g);
  lcd.createChar(7, h);
}
void printStr(String text, int index, int line) {
  lcd.setCursor(index, line);
  lcd.print(text);
}

void printChr(int character, int index, int line) {
  lcd.setCursor(index, line);
  lcd.write(character);
}

void clearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                    ");
}

void bitReverse(int* X, int M) {
  X[0] = 0;
  int i, j;
  for (i = M >> 1;i >= 1;i = i / 2) {
    for (j = 0;j < (M / i) >> 1;j++) {
      X[j + (M / (2 * i))] = X[j] + i;
    }
  }
}

void copy_array(struct complex* out, float* inp, int M) {
  for (i = 0;i < M / 2;i++) {
    out[i].real = inp[bit_reverse[i]];
    out[i + M / 2].real = inp[bit_reverse[i] + 1];
    out[i].imag = 0;
    out[i + M / 2].imag = 0;
  }
}
void fft(struct complex* Y, int M, struct complex* w) {
  steps = 1;
  do {
    steps *= 2;

    for (i = 0;i < M / steps;i++) {
      for (j = 0;j < steps / 2;j++) {
        lower = i * steps + j + steps / 2;
        upper = i * steps + j;
        skip = M * j / steps;

        temp_real = Y[lower].real * w[skip].real - Y[lower].imag * w[skip].imag;
        temp_imag = Y[lower].real * w[skip].imag + Y[lower].imag * w[skip].real;

        Y[lower].real = Y[upper].real - temp_real;
        Y[lower].imag = Y[upper].imag - temp_imag;
        Y[upper].real = Y[upper].real + temp_real;
        Y[upper].imag = Y[upper].imag + temp_imag;
      }
    }
  } while (steps != M);
}
void absolute(struct complex* inp, int M, float* out) {
  int i;
  for (i = 0;i < M;i++)
    out[i] = (inp[i].real * inp[i].real + inp[i].imag * inp[i].imag);
}
void normalize(float* input_abs_data) {
  float maximum_element = input_abs_data[0];
  for (int i = 1; i < fft_size; i++) {
    if (maximum_element < input_abs_data[i]) {
      maximum_element = input_abs_data[i];
    }
  }
  for (int i = 0; i < fft_size; i++) {
    input_abs_data[i] = input_abs_data[i] / maximum_element;
    input_abs_data[i] = int(32 * input_abs_data[i]);
  }
}
void fft_data() {
  copy_array(samples, fft_in, fft_size);
  fft(samples, fft_size, twiddle);
  absolute(samples, fft_size, samples_abs);
  normalize(samples_abs);
}

void twid(struct complex* Y, int M) {
  for (i = 0;i < M >> 1;i++) {
    Y[i].real = cos(2 * PI * i / M);
    Y[i].imag = sin(2 * PI * i / M);
  }
}

void display_data() {
  for (int k = 0; k < (fft_size / 2); k++) {
    int q, r = 0;
    q = int(samples_abs[k]) / 8;
    r = int(samples_abs[k]) % 8;
    if (q == 0) {
      display_xy[3][k] = r;
      display_xy[2][k] = 0;
      display_xy[1][k] = 0;
      display_xy[0][k] = 0;
    }
    else if (q == 1) {
      display_xy[3][k] = 8;
      display_xy[2][k] = r;
      display_xy[1][k] = 0;
      display_xy[0][k] = 0;
    }
    else if (q == 2) {
      display_xy[3][k] = 8;
      display_xy[2][k] = 8;
      display_xy[1][k] = r;
      display_xy[0][k] = 0;
    }
    else if (q == 3) {
      display_xy[3][k] = 8;
      display_xy[2][k] = 8;
      display_xy[1][k] = 8;
      display_xy[0][k] = r;
    }
    else if (q == 4) {
      display_xy[3][k] = 8;
      display_xy[2][k] = 8;
      display_xy[1][k] = 8;
      display_xy[0][k] = 8;
    }
  }
}
void dac_init() {
  pinMode(25, ANALOG);
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
  SET_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);
}
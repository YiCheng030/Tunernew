#include <Arduino.h>
#include <Adafruit_GFX.h>  // 核心圖形庫
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h>  //Hardware-specific library for ST7789
#include <Fonts/FreeMonoBold9pt7b.h>  // 字型FreeMonoBold9pt7b
#include <Fonts/FreeSansBold9pt7b.h>  // 字型FreeSansBold9pt7b
#include <Fonts/FreeSerif9pt7b.h>  // 字型FreeSerif9pt7b
#include <I2S.h>
#include <SPI.h>
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
#define C0   16.35
#define C0U  17.32
#define D0   18.35
#define D0U  19.44
#define E0   20.60
#define F0   21.83
#define F0U  23.13
#define G0   24.50
#define G0U  25.96
#define A0   27.50
#define A0U  29.14
#define B0   30.87
#define C1   32.70
#define C1U  34.65
#define D1   36.70
#define D1U  38.89
#define E1   41.20
#define F1   43.66
#define F1U  46.25
#define G1   49.00
#define G1U  51.91
#define A1   55.00
#define A1U  58.27
#define B1   61.74
#define C2   65.40
#define C2U  69.30
#define D2   73.42
#define D2U  77.78
/*------------------------------------六弦吉他------------------------------------*/
#define E2   82.41    //第六弦 (最粗)
#define F2   87.31
#define F2U  92.50
#define G2   98.00
#define G2U  103.83
#define A2   110.00   //第五弦
#define A2U  116.54
#define B2   123.47
#define C3   130.81
#define C3U  138.59
#define D3   146.83   //第四弦
#define D3U  155.56
#define E3   164.81
#define F3   174.61
#define F3U  185.00
#define G3   196.00   //第三弦
#define G3U  207.65
#define A3   220.00
#define A3U  233.08
#define B3   246.94   //第二弦
#define C4   261.63
#define C4U  277.18
#define D4   293.66
#define D4U  311.13
#define E4   329.63   //第一弦 (最細)
#define F4   349.23
#define F4U  369.99
#define G4   392.00
#define G4U  415.30
/*--------------------------------------------------------------------------------*/
#define A4   440.00   //基準音
/*--------------------------------------------------------------------------------*/
#define A4U  466.18
#define B4   493.90
#define C5   523.26
#define C5U  554.38
#define D5   587.34
#define D5U  622.27
#define E5   659.27
#define F5   698.47
#define F5U  740.00
#define G5   784.01
#define G5U  830.63
#define A5   880.00
#define A5U  932.35
#define B5   987.79
#define C6   1046.53
#define C6U  1108.76
#define D6   1174.69
#define D6U  1244.54
#define E6   1318.54
#define F6   1396.95
#define F6U  1480.01
#define G6   1568.02
#define G6U  1661.26
#define A6   1760.04
#define A6U  1864.70
#define B6   1975.58
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
#define CS   4 // TFT CS PIN腳
#define DC   6 // TFT DC(A0、RS) PIN腳
#define RST  7 // TFT RES(Reset) PIN腳
#define MOSI 8 // TFT MOSI PIN腳
#define SCK  9 // TFT SCK PIN腳

#define samples 1024
#define fs 4000
#define TotalWire 400
#define bin(x) sqrt(data_of_N_FFT[x].real * data_of_N_FFT[x].real + data_of_N_FFT[x].imag * data_of_N_FFT[x].imag) / 256.0
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
const int buttonPin = 5;
int N_FFT = 0;                // 傅立葉變換的點數  
int M_of_N_FFT = 0;           // 蝶形運算的級數，N = 2^M  
int signalFrequency[samples];
int x;
float binmax, peakmax;
float intonation = 0, intonation_old;
int string;
float freq = 0;
int PaintingWire;
int j, j2;
int i, i2;
int color;

typedef float ElemType;     // 原始資料序列的資料型別,可以在這裡設定
typedef struct{             // 定義複數結構體
  ElemType real,imag;  
}complex_of_N_FFT,*ptr_complex_of_N_FFT;  

ptr_complex_of_N_FFT data_of_N_FFT = NULL; // 開闢儲存單元，原始資料與負數結果均使用之

Adafruit_ST7735 tft = Adafruit_ST7735(CS, DC, MOSI, SCK, RST);
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
#define Black tft.color565(0, 0, 0)         //黑
#define White tft.color565(255, 255, 255)   //白
#define Gray  tft.color565(128, 128, 128)   //灰
#define Red   tft.color565(255, 0, 0)       //紅
#define Green tft.color565(0, 255, 0)       //綠
#define Blue  tft.color565(0, 0, 255)       //藍
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void InputData(void){
  int count;
  for(count = 0; count < N_FFT; count++){
    int sample = 0; 
    while ((sample == 0) || (sample == -1) ) {
      sample = I2S.read();
    }
    // convert to 18 bit signed
    sample >>= 14; 
    signalFrequency[count] = sample;
    data_of_N_FFT[count].real = (signalFrequency[count]);
    data_of_N_FFT[count].imag = 0.0;
  }
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
// 變址
void ChangeSeat(complex_of_N_FFT *DataInput){
  int nextValue, nextM, i, k, j = 0;
  complex_of_N_FFT temp;
  
  nextValue = N_FFT / 2;            // 變址運算，即把自然順序變成倒位序，採用雷德演算法  
  nextM = N_FFT - 1;
  for(i = 0; i < nextM; i++){
    if(i < j){                      // 如果i<j,即進行變址    
      temp = DataInput[j];
      DataInput[j] = DataInput[i];
      DataInput[i] = temp;
    }
    k = nextValue;                  // 求j的下一個倒位序  
    while(k <= j){                  // 如果k<=j,表示j的最高位為1  
      j = j - k;                    // 把最高位變成0  
      k = k / 2;                    // k/2，比較次高位，依次類推，逐個比較，直到某個位為0  
    }
    j = j + k;                      // 把0改為1  
  }
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
// FFT運算函式  
void FFT(void){  
  int L = 0, B = 0, J = 0, K = 0;
  int step = 0, KB = 0;
  
  ElemType angle;
  complex_of_N_FFT W, Temp_XX;
  
  ChangeSeat(data_of_N_FFT); // 變址
  for(L = 1; L <= M_of_N_FFT; L++){
    step = 1 << L; // 2^L  
    B = step >> 1; // B = 2^(L - 1)  
    for(J = 0; J < B; J++){
        angle = (double)J / B;
        W.imag = -sin(angle * PI);    // 用C++該函式課宣告為inline
        W.real =  cos(angle * PI);    // 用C++該函式課宣告為inline
      for(K = J; K < N_FFT; K = K + step){
        KB = K + B;
        Temp_XX.real = data_of_N_FFT[KB].real * W.real - data_of_N_FFT[KB].imag * W.imag;
        Temp_XX.imag = data_of_N_FFT[KB].real * W.imag + data_of_N_FFT[KB].imag * W.real;

        data_of_N_FFT[KB].real = data_of_N_FFT[K].real - Temp_XX.real;  
        data_of_N_FFT[KB].imag = data_of_N_FFT[K].imag - Temp_XX.imag;  

        data_of_N_FFT[K].real = data_of_N_FFT[K].real + Temp_XX.real;  
        data_of_N_FFT[K].imag = data_of_N_FFT[K].imag + Temp_XX.imag;  
      }
    }
  }
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
// 初始化FFT程式
// N_FFT是 FFT的點數，必須是2的次方
void Init_FFT(int N_of_FFT){
  int count = 0;
  int temp_N_FFT = 1;
  N_FFT = N_of_FFT;               // 傅立葉變換的點數 ，必須是 2的次方
  M_of_N_FFT = 0;                 // 蝶形運算的級數，N = 2^M
  
  for(count = 0; temp_N_FFT < N_FFT; count++){
    temp_N_FFT = 2 * temp_N_FFT;
    M_of_N_FFT++;
  }

  data_of_N_FFT = (ptr_complex_of_N_FFT)malloc(N_FFT * sizeof(complex_of_N_FFT));
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
// 結束 FFT運算，釋放相關記憶體
void Close_FFT(void){
  if(data_of_N_FFT != NULL){
    free(data_of_N_FFT);          // 釋放記憶體
    data_of_N_FFT = NULL;
  }
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void Find_peaks(){
  int count1, count2 = 1, count3;
  int peak[count2];
  peakmax = 0;
  binmax = 0;
  Init_FFT(samples);
  InputData();        // 輸入原始資料
  FFT();              // 進行 FFT計算
  for(count1 = 12; count1 <= (samples * 0.5); count1++){
    if(bin(count1) > binmax)
      binmax = bin(count1);
  }
  for(count1 = 12; count1 <= (samples * 0.5); count1++){
    if(bin(count1) >= binmax * 0.3){
      if(bin(count1) >= bin(count1 - 1)){
        for(count3 = 0; count3 < 11; count3++){
          if(bin(count1 + count3) > peakmax){
            peakmax = bin(count1 + count3);
            x = count1 + count3;
          }
        }
        peak[count2] = x;
        // Serial.print(peak[count2]);
        // Serial.print("\t");
        count2++;
      }
    }
  }
  // Serial.println(peak[0]);
  freq = ((fs / (samples * 1.0)) * peak[0]);
  // Serial.println(freq);
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
bool InRange(float frequency, float low_limit, float high_limit){
  if(frequency < high_limit && frequency > low_limit)
    return true;
  else
    return false;
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
float AudioPitch(float Musical_Alphabet, float Low_Musical_Alphabet, float High_Musical_Alphabet){
  float y;
  if(Musical_Alphabet > Low_Musical_Alphabet){ //上半
    y = ((Musical_Alphabet - Low_Musical_Alphabet) * 0.5);
    intonation = map(freq, 0, y, 0, (TotalWire * 0.5));
  }
  else if(Musical_Alphabet < High_Musical_Alphabet){ //下半
    y = ((High_Musical_Alphabet - Musical_Alphabet) * 0.5);
    intonation = map(freq, 0, y, (TotalWire * 0.5), TotalWire);
  }
  return intonation;
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void Tuner_Interfaz(){
  if(intonation > 400.0)
    intonation = intonation_old;
  else
    intonation_old = intonation;
  while(PaintingWire < intonation){
    j  = 54 * (sin(PI * PaintingWire / (TotalWire * 0.5)));
    i  = 54 * (cos(PI * PaintingWire / (TotalWire * 0.5)));
    j2 = 50 * (sin(PI * PaintingWire / (TotalWire * 0.5)));
    i2 = 50 * (cos(PI * PaintingWire / (TotalWire * 0.5)));
    tft.drawLine(i2 + 80, j2 + 65, i + 80, j + 65, Red);
    PaintingWire++;
  }

  while(PaintingWire > intonation){
    j  = 54 * (sin(PI * PaintingWire / (TotalWire * 0.5)));
    i  = 54 * (cos(PI * PaintingWire / (TotalWire * 0.5)));
    j2 = 50 * (sin(PI * PaintingWire / (TotalWire * 0.5)));
    i2 = 50 * (cos(PI * PaintingWire / (TotalWire * 0.5)));
    tft.drawLine(i2 + 80, j2 + 65, i + 80, j + 65, Gray);
    PaintingWire--;
  }
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void MusicalAlphabet(){
  Find_peaks();
  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(6, 120);
  tft.print(freq);tft.print("Hz");

  tft.setTextSize(4);
  tft.setTextColor(Black, White);
  tft.setCursor(70, 53);
  if(InRange(freq, ((G1 - F1U) * 0.5), ((A2 - G2U) * 0.5)))
    string = 6;
  else if(InRange(freq, ((A2 - G2U) * 0.5), ((D3 - C3U) * 0.5)))
    string = 5;
  else if(InRange(freq, ((D3 - C3U) * 0.5), ((G3 - F3U) * 0.5)))
    string = 4;
  else if(InRange(freq, ((G3 - F3U) * 0.5), ((B3 - A3U) * 0.5)))
    string = 3;
  else if(InRange(freq, ((B3 - A3U) * 0.5), ((E4 - D4U) * 0.5)))
    string = 2;
  else if(InRange(freq, ((E4 - D4U) * 0.5), ((E5 - D5U) * 0.5)))
    string = 1;

  Tuner_Interfaz();
  Close_FFT();        // 結束 FFT運算，釋放相關記憶體
  tft.setTextSize(3);
  tft.setTextColor(White, Black);
  tft.setCursor(6, 6);
  tft.print(string);
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  // 設備初始化
  I2S.begin(I2S_PHILIPS_MODE, fs, 32);
  Wire.begin();
  tft.initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  tft.setRotation(1);  // 螢幕轉向
  tft.fillScreen(Black);  // 設定螢幕背景為黑色
  tft.fillCircle(80, 65, 53, Gray);
  tft.fillCircle(80, 65, 48, White);

  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(137, 66);
  tft.print("-50");

  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(143, 58);
  tft.print("50");
  
  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(72, 120);
  tft.print("-25");

  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(75, 3);
  tft.print("25");

  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(18, 62);
  tft.print("0");
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void loop(){
  MusicalAlphabet();
}
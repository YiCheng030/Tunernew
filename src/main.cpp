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
#define A4   440.00   //基準音
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
  for(count1 = 18; count1 <= (samples * 0.5); count1++){
    if(bin(count1) > binmax)
      binmax = bin(count1);
  }
  for(count1 = 18; count1 <= (samples * 0.5); count1++){
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
void precise(){
  if(intonation >= 48 && intonation <= 52)
    color = Green;
  else
    color = White;
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
float AudioPitch(float Musical_Alphabet, float Front_or_later_Musical_Alphabet){
  float y;
  if(Musical_Alphabet > Front_or_later_Musical_Alphabet){
    y = ((Musical_Alphabet - Front_or_later_Musical_Alphabet) * 0.5);
    intonation = map(freq, 0, y, 0, (TotalWire * 0.5));
    return intonation;
  }
  else{
    y = ((Front_or_later_Musical_Alphabet - Musical_Alphabet) * 0.5);
    intonation = map(freq, 0, y, (TotalWire * 0.5), TotalWire);
    return intonation;
  }
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
  /*第六弦*/
  if(InRange(freq, 77.8, A2)){
    string = 6;
    if(InRange(freq, 77.8, F2)){
      intonation = map(freq, 77.8, F2, 0, TotalWire); //E2
      tft.print("E ");
    }
    else if(InRange(freq, E2, F2U)){
      intonation = map(freq, E2, F2U, 0, TotalWire); //F2
      tft.print("F ");
    }
    else if(InRange(freq, F2, G2)){
      intonation = map(freq, F2, G2, 0, TotalWire); //F2U
      tft.print("F#");
    }
    else if(InRange(freq, F2U, G2U)){
      intonation = map(freq, F2U, G2U, 0, TotalWire); //G2
      tft.print("G ");
    }
    else if(InRange(freq, G2, A2)){
      intonation = map(freq, G2, A2, 0, TotalWire); //G2U
      tft.print("G#");
    }
  }
  /*第五弦*/
  else if(InRange(freq, G2U, D3)){
    string = 5;
    if(InRange(freq, G2U, A2U)){
      intonation = map(freq, G2U, A2U, 0, TotalWire); //A2
      tft.print("A ");
    }
    else if(InRange(freq, A2, B2)){
      intonation = map(freq, A2, B2, 0, TotalWire); //A2U
      tft.print("A#");
    }
    else if(InRange(freq, A2U, C3)){
      intonation = map(freq, A2U, C3, 0, TotalWire); //B2
      tft.print("B ");
    }
    else if(InRange(freq, B2, C3U)){
      intonation = map(freq, B2, C3U, 0, TotalWire); //C3
      tft.print("C ");
    }
    else if(InRange(freq, C3, D3)){
      intonation = map(freq, C3, D3, 0, TotalWire); //C3U
      tft.print("C#");
    }
  }
  /*第四弦*/
  else if(InRange(freq, C3U, G3)){
    string = 4;
    if(InRange(freq, C3U, D3U)){
      intonation = map(freq, C3U, D3U, 0, TotalWire); //D3
      tft.print("D ");
    }
    else if(InRange(freq, D3, E3)){
      intonation = map(freq, D3, E3, 0, TotalWire); //D3U
      tft.print("D#");
    }
    else if(InRange(freq, D3U, F3)){
      intonation = map(freq, D3U, F3, 0, TotalWire); //E3
      tft.print("E ");
    }
    else if(InRange(freq, E3, F3U)){
      intonation = map(freq, E3, F3U, 0, TotalWire); //F3
      tft.print("F ");
    }
    else if(InRange(freq, F3, G3)){
      intonation = map(freq, F3, G3, 0, TotalWire); //F3U
      tft.print("F#");
    }
  }
  /*第三弦*/
  else if(InRange(freq, F3U, B3)){
    string = 3;
    if(InRange(freq, F3U, G3U)){
      intonation = map(freq, F3U, G3U, 0, TotalWire); //G3
      tft.print("G ");
    }
    else if(InRange(freq, G3, A3)){
      intonation = map(freq, G3, A3, 0, TotalWire); //G3U
      tft.print("G#");
    }
    else if(InRange(freq, G3U, A3U)){
      intonation = map(freq, G3U, A3U, 0, TotalWire); //A3
      tft.print("A ");
    }
    else if(InRange(freq, A3, B3)){
      intonation = map(freq, A3, B3, 0, TotalWire); //A3U
      tft.print("A#");
    }
  }
  /*第二弦*/
  else if(InRange(freq, A3U, E4)){
    string = 2;
    if(InRange(freq, A3U, C4)){
      intonation = map(freq, A3U, C4, 0, TotalWire); //B3
      tft.print("B ");
    }
    else if(InRange(freq, B3, C4U)){
      intonation = map(freq, B3, C4U, 0, TotalWire); //C4
      tft.print("C ");
    }
    else if(InRange(freq, C4, D4)){
      intonation = map(freq, C4, D4, 0, TotalWire); //C4U
      tft.print("C#");
    }
    else if(InRange(freq, C4U, D4U)){
      intonation = map(freq, C4U, D4U, 0, TotalWire); //D4
      tft.print("D ");
    }
    else if(InRange(freq, D4, E4)){
      intonation = map(freq, D4, E4, 0, TotalWire); //D4U
      tft.print("D#");
    }
  }
  /*第一弦*/
  else if(InRange(freq, D4U, A4)){
    string = 1;
    if(InRange(freq, D4U, F4)){
      intonation = map(freq, D4U, F4, 0, TotalWire); //E4
      tft.print("E ");
    }
    else if(InRange(freq, E4, F4U)){
      intonation = map(freq, E4, F4U, 0, TotalWire); //F4
      tft.print("F ");
    }
    else if(InRange(freq, F4, G4)){
      intonation = map(freq, F4, G4, 0, TotalWire); //F4U
      tft.print("F#");
    }
    else if(InRange(freq, F4U, G4U)){
      intonation = map(freq, B2, C3U, 0, TotalWire); //G4
      tft.print("G ");
    }
    else if(InRange(freq, G4, A4)){
      intonation = map(freq, G4, A4, 0, TotalWire); //G4U
      tft.print("G#");
    }
  }
  else{
    tft.setTextSize(4);
    tft.setTextColor(Black, White);
    tft.setCursor(45, 53);
    tft.print("   ");
  }
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
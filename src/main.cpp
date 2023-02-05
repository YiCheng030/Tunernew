#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <Adafruit_GFX.h>  // 核心圖形庫
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h>  //Hardware-specific library for ST7789
#include <Fonts/FreeMonoBold9pt7b.h>  // 字型FreeMonoBold9pt7b
#include <Fonts/FreeSansBold9pt7b.h>  // 字型FreeSansBold9pt7b
#include <Fonts/FreeSerif9pt7b.h>  // 字型FreeSerif9pt7b
#include <I2S.h>
#include <SPI.h>
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
#define battery 16 //分壓器
#define CS   4 // TFT CS PIN腳
#define DC   6 // TFT DC(A0、RS) PIN腳
#define RST  7 // TFT RES(Reset) PIN腳
#define MOSI 8 // TFT MOSI PIN腳
#define SCK  9 // TFT SCK PIN腳
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
#define Black tft.color565(0, 0, 0)         //黑
#define White tft.color565(255, 255, 255)   //白
#define Gray  tft.color565(128, 128, 128)   //灰
#define Red   tft.color565(255, 0, 0)       //紅
#define Green tft.color565(0, 255, 0)       //綠
#define Blue  tft.color565(0, 0, 255)       //藍
#define Yellow tft.color565(255, 255, 0)    //黃
#define Green_Yellow tft.color565(173, 255, 47) //綠黃
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
#define samples 1024
#define fs 4000
#define THRESHOLD 0.1 // 峰值閾值
#define TotalWire 400
float bin[samples]; //= sqrt(data_of_N_FFT[x].real * data_of_N_FFT[x].real + data_of_N_FFT[x].imag * data_of_N_FFT[x].imag) / 256.0;
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
float C[8]  = {16.352, 32.704, 65.408, 130.816, 261.632, 523.264, 1046.528, 2093.056};
float CU[8] = {17.324, 34.649, 69.297, 138.595, 177.189, 554.379, 1108.758, 2217.515};
float D[8]  = {18.354, 36.709, 73.418, 146.836, 293.672, 587.344, 1174.688, 2349.376};
float DU[8] = {19.446, 38.892, 77.784, 155.567, 311.135, 622.269, 1244.538, 2489.076};
float E[8]  = {20.602, 41.204, 82.409, 164.818, 329.636, 659.271, 1318.542, 2637.084};
float F[8]  = {21.827, 43.655, 87.309, 174.618, 349.237, 698.473, 1396.947, 2793.893};
float FU[8] = {23.125, 46.250, 92.501, 185.002, 370.003, 740.007, 1480.013, 2960.027};
float G[8]  = {24.500, 49.001, 98.001, 196.002, 392.005, 784.010, 1568.019, 3136.039};
float GU[8] = {25.957, 51.914, 103.829, 207.657, 415.315, 830.629, 1661.258, 3322.517};
float A[8]  = {27.501, 55.001, 110.003, 220.005, 440.010, 880.021, 1760.042, 3520.084};
float AU[8] = {29.136, 58.272, 116.544, 233.087, 466.175, 932.350, 1864.699, 3729.398};
float B[8]  = {30.868, 61.737, 123.474, 246.947, 493.895, 987.790, 1975.580, 3951.160};
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
const int buttonPin = 5;
int N_FFT = 0;                // 傅立葉變換的點數  
int M_of_N_FFT = 0;           // 蝶形運算的級數，N = 2^M  
int signalFrequency[samples];
int x;
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
float binmax, peakmax;
float Proportion, intonation = 0, intonation_old;
int Octave;
float freq = 0;
int PaintingWire;
int j, j2;
int i, i2;
long BADC;
int BVal;

typedef float ElemType;     // 原始資料序列的資料型別,可以在這裡設定
typedef struct{             // 定義複數結構體
  ElemType real,imag;  
}complex_of_N_FFT,*ptr_complex_of_N_FFT;  

ptr_complex_of_N_FFT data_of_N_FFT = NULL; // 開闢儲存單元，原始資料與負數結果均使用之

Adafruit_ST7735 tft = Adafruit_ST7735(CS, DC, MOSI, SCK, RST);
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
  for(x = 0; x < (samples * 0.5); x++)
    bin[x] = sqrt(data_of_N_FFT[x].real * data_of_N_FFT[x].real + data_of_N_FFT[x].imag * data_of_N_FFT[x].imag) / 256.0;
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
  float max_magnitude = 0.0f;
  double max = 0;
  int count;
  int index = 0;
  Init_FFT(samples);
  InputData();        // 輸入原始資料
  FFT();              // 進行 FFT計算
  // 找到 FFT 計算結果中的最大振幅
  for(count = 10; count <= (samples * 0.5); count++){
    float magnitude = fabs(bin[count]);
    if(magnitude > max_magnitude)
      max_magnitude = magnitude;
  }
  // 將 FFT 計算結果中振幅小於閾值的元素變為 0
  for(count = 10; count < (samples * 0.5); count++) {
    float magnitude = fabs(bin[count]);
    if(magnitude < max_magnitude * THRESHOLD)
      bin[count] = 0.0f;
  }
  // 尋找峰值
  if(!(index == 0)){
    for(count = 10; count < (samples * 0.5); count++){
      double mag = bin[count];
      if(mag > max)
        index = count;
    }
  }
  freq = ((fs / (samples * 1.0)) * index);
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
bool InRange(float frequency, float L_Musical_Alphabet, float Musical_Alphabet, float Musical_Alphabet2, float H_Musical_Alphabet){
  float ml, mu;
  ml = ((Musical_Alphabet - L_Musical_Alphabet) * 0.5);
  mu = ((H_Musical_Alphabet - Musical_Alphabet2) * 0.5);
  if(frequency < (H_Musical_Alphabet - mu) && frequency > (L_Musical_Alphabet - ml))
    return true;
  else
    return false;
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
float AudioPitch(float Musical_Alphabet, float L_Musical_Alphabet, float H_Musical_Alphabet){
  float ml, mu;
  ml = ((Musical_Alphabet - L_Musical_Alphabet) * 0.5);
  mu = ((H_Musical_Alphabet - Musical_Alphabet) * 0.5);
  if(freq > (L_Musical_Alphabet + ml)){ //上半
    Proportion = map(freq, (L_Musical_Alphabet + ml), Musical_Alphabet, 0, (TotalWire * 0.5));
  }
  else if(freq < (H_Musical_Alphabet - mu)){ //下半
    Proportion = map(freq, (H_Musical_Alphabet - mu), Musical_Alphabet, (TotalWire * 0.5), TotalWire);
  }
  return Proportion;
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
void MusicalAlphabetJudge(){
  Find_peaks();
  tft.setTextSize(1);
  tft.setTextColor(White, Black);
  tft.setCursor(6, 120);
  tft.print(freq);tft.print("Hz  ");
/*--------------------------------------------------------------------------------*/
  if(InRange(freq, C[0], C[0], B[0], C[1]))      //八度0
    Octave = 0;
  else if(InRange(freq, B[0], C[1], B[1], C[2])) //八度1
    Octave = 1;
  else if(InRange(freq, B[1], C[2], B[2], C[3])) //八度2
    Octave = 2;
  else if(InRange(freq, B[2], C[3], B[3], C[4])) //八度3
    Octave = 3;
  else if(InRange(freq, B[3], C[4], B[4], C[5])) //八度4
    Octave = 4;
  else if(InRange(freq, B[4], C[5], B[5], C[6])) //八度5
    Octave = 5;
  else if(InRange(freq, B[5], C[6], B[6], C[7])) //八度6
    Octave = 6;
  else if(InRange(freq, B[6], C[7], B[7], B[7])) //八度7
    Octave = 7;
/*--------------------------------------------------------------------------------*/
  tft.setTextSize(4);
  tft.setCursor(70, 53);

  if(InRange(freq, C[Octave], C[Octave], C[Octave], CU[Octave]) && Octave == 0){  //C0
    intonation = AudioPitch(C[Octave], C[Octave], CU[Octave]);
    tft.setTextColor(Black, White);
    tft.print("C ");
  }
  else if(InRange(freq, B[Octave - 1], C[Octave], C[Octave], CU[Octave]) && Octave > 0){ //C
    intonation = AudioPitch(C[Octave], B[Octave - 1], CU[Octave]);
    tft.setTextColor(Black, White);
    tft.print("C ");
  }
  else if(InRange(freq, C[Octave], CU[Octave], CU[Octave], D[Octave])){ //C#
    intonation = AudioPitch(CU[Octave], C[Octave], D[Octave]);
    tft.setTextColor(Black, White);
    tft.print("C#");
  }
  else if(InRange(freq, CU[Octave], D[Octave], D[Octave], DU[Octave])){ //D
    intonation = AudioPitch(D[Octave], CU[Octave], DU[Octave]);
    if(Octave == 3 && intonation >= 40.0 && intonation <= 60.0)
      tft.setTextColor(Green, White);
    else
      tft.setTextColor(Black, White);
    tft.print("D ");
  }
  else if(InRange(freq, D[Octave], DU[Octave], DU[Octave], E[Octave])){ //D#
    intonation = AudioPitch(DU[Octave], D[Octave], E[Octave]);
    tft.setTextColor(Black, White);
    tft.print("D#");
  }
  else if(InRange(freq, DU[Octave], E[Octave], E[Octave], F[Octave])){  //E
    intonation = AudioPitch(E[Octave], DU[Octave], F[Octave]);
    if(Octave == 2 && Octave == 4 && intonation >= 40.0 && intonation <= 60.0)
      tft.setTextColor(Green, White);
    else
      tft.setTextColor(Black, White);
    tft.print("E ");
  }
  else if(InRange(freq, E[Octave], F[Octave], F[Octave], FU[Octave])){  //F
    intonation = AudioPitch(F[Octave], E[Octave], FU[Octave]);
    tft.setTextColor(Black, White);
    tft.print("F ");
  }
  else if(InRange(freq, F[Octave], FU[Octave], FU[Octave], G[Octave])){ //F#
    intonation = AudioPitch(FU[Octave], F[Octave], G[Octave]);
    tft.setTextColor(Black, White);
    tft.print("F#");
  }
  else if(InRange(freq, FU[Octave], G[Octave], G[Octave], GU[Octave])){ //G
    intonation = AudioPitch(G[Octave], FU[Octave], GU[Octave]);
    if(Octave == 3 && intonation >= 40.0 && intonation <= 60.0)
      tft.setTextColor(Green, White);
    else
      tft.setTextColor(Black, White);
    tft.print("G ");
  }
  else if(InRange(freq, G[Octave], GU[Octave], GU[Octave], A[Octave])){ //G#
    intonation = AudioPitch(GU[Octave], G[Octave], A[Octave]);
    tft.setTextColor(Black, White);
    tft.print("G#");
  }
  else if(InRange(freq, GU[Octave], A[Octave], A[Octave], B[Octave])){  //A
    intonation = AudioPitch(A[Octave], GU[Octave], B[Octave]);
    if(Octave == 2 && intonation > 40.0 && intonation < 60.0)
      tft.setTextColor(Green, White);
    else
      tft.setTextColor(Black, White);
    tft.print("A ");
  }
  else if(InRange(freq, A[Octave], B[Octave], B[Octave], C[Octave + 1]) && Octave <= 7){ //B
    intonation = AudioPitch(B[Octave], A[Octave], C[Octave + 1]);
    if(Octave == 3 && intonation >= 40.0 && intonation <= 60.0)
      tft.setTextColor(Green, White);
    else
      tft.setTextColor(Black, White);
    tft.print("B ");
  }

  Tuner_Interfaz();
  Close_FFT();        // 結束 FFT運算，釋放相關記憶體

  tft.setTextSize(3);
  tft.setTextColor(White, Black);
  tft.setCursor(6, 6);
  tft.print(Octave);
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void Power_display(){
  BADC = analogRead(battery);
  BVal = BADC * (3.3 / 4095) * 128;
  if(BVal >= 360){
    tft.fillRect(151, 9, 4, 8, Green);
    tft.fillRect(146, 9, 4, 8, Green);
    tft.fillRect(141, 9, 4, 8, Green);
    tft.fillRect(136, 9, 4, 8, Green);
  }
  if(BVal >= 345){
    tft.fillRect(151, 9, 4, 8, Black);
    tft.fillRect(146, 9, 4, 8, Green_Yellow);
    tft.fillRect(141, 9, 4, 8, Green_Yellow);
    tft.fillRect(136, 9, 4, 8, Green_Yellow);
  }
  if(BVal >= 330){
    tft.fillRect(151, 9, 4, 8, Black);
    tft.fillRect(146, 9, 4, 8, Black);
    tft.fillRect(141, 9, 4, 8, Yellow);
    tft.fillRect(136, 9, 4, 8, Yellow);
  }
  if(BVal >= 315){
    tft.fillRect(151, 9, 4, 8, Black);
    tft.fillRect(146, 9, 4, 8, Black);
    tft.fillRect(141, 9, 4, 8, Black);
    tft.fillRect(136, 9, 4, 8, Red);
  }
  else{
    tft.fillRect(151, 9, 4, 8, Black);
    tft.fillRect(146, 9, 4, 8, Black);
    tft.fillRect(141, 9, 4, 8, Black);
    tft.fillRect(136, 9, 4, 8, Black);
  }
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

  tft.drawRect(135, 8, 21, 10, White);
  tft.fillRect(156, 11, 2, 4, White);
}
/*-------------------------------------------------------------------------------------------------------------------------------------------*/
void loop(){
  MusicalAlphabetJudge();
  Power_display();
}
#include <arduinoFFT.h>  // Pustaka untuk FFT

// Definisikan pin RX dan TX untuk UART
#define RXD2 16
#define TXD2 17

// Definisikan pin LED (menggunakan GPIO 18)
#define LED_PIN 18 // Pin LED, gunakan GPIO 18 pada ESP32 S3

// Parameter EMG dan analisis
#define CHANNELS 8 // Jumlah channel EMG
#define SAMPLE_SIZE 128 // Jumlah sampel untuk analisis FFT
#define SAMPLING_FREQUENCY 1000.0 // Frekuensi sampling dalam Hz (1 ms per sampel)
#define VOLTAGE_REFERENCE 3.3 // Referensi tegangan untuk ADC
#define ADC_RESOLUTION 255.0 // Resolusi ADC untuk konversi data gForce 200

// Array untuk menyimpan data sinyal EMG gabungan
double vReal[SAMPLE_SIZE];
double vImag[SAMPLE_SIZE];

// Membuat objek untuk FFT
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLE_SIZE, SAMPLING_FREQUENCY);

// Buffer untuk menyimpan data dari 8 channel
byte dataBuffer[CHANNELS][SAMPLE_SIZE];
int ch = 0;
int bufferIndex = 0;

// Ambang batas untuk deteksi kelelahan
double thresholdMDF = 10.0; // Ambang batas MDF, lebih tahan noise
double thresholdMNF = 15.0; // Ambang batas MNF, menyesuaikan agar lebih stabil

unsigned long lastDisplayTime = 0; 
// Inisialisasi serial komunikasi
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // UART dengan OYMotion gForce 200
  pinMode(LED_PIN, OUTPUT); // Set pin LED sebagai output
  Serial.println("Starting...");
}

void loop() {
 
  // Cek apakah ada data masuk dari OYMotion
  if (Serial2.available() > 0) {
    unsigned long currentTime = millis(); // Dapatkan waktu saat ini

    while (Serial2.available() > 0 && bufferIndex < SAMPLE_SIZE) {
      
      dataBuffer[ch][bufferIndex] = Serial2.read();
      float voltage = (dataBuffer[ch][bufferIndex] / 255.0) * 3.3;  // Konversi ke volt
      
      Serial.print(voltage, 2);  // Tampilkan dalam format volt dengan 2 angka desimal
      Serial.print(", ");
      ch++;
      
      if (ch >= CHANNELS) {
        bufferIndex++;
        ch = 0;
        Serial.println();
      }

      // Jika buffer penuh, lakukan analisis
      if (bufferIndex >= SAMPLE_SIZE) {
        // Hitung rata-rata data dari semua channel
        for (int i = 0; i < SAMPLE_SIZE; i++) {
          double averageValue = 0.0;
          for (int ch = 0; ch < CHANNELS; ch++) {
            averageValue += (dataBuffer[ch][i] / ADC_RESOLUTION) * VOLTAGE_REFERENCE;
          }
          vReal[i] = averageValue / CHANNELS; // Nilai rata-rata per sampel
          vImag[i] = 0; // Inisialisasi array imajiner
        }

        // Cetak nilai input awal
        Serial.println("Input FFT (Rata-rata 8 Channel):");
        for (int i = 0; i < SAMPLE_SIZE; i++) {
          Serial.print(vReal[i]);
          Serial.print(i < SAMPLE_SIZE - 1 ? ", " : "\n");
        }

        // Analisis FFT
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();

        // Cetak nilai setelah FFT
        Serial.println("Hasil FFT (Magnitudo):");
        for (int i = 0; i < SAMPLE_SIZE / 2; i++) { // Hanya nilai frekuensi positif
          Serial.print(vReal[i]);
          Serial.print(i < (SAMPLE_SIZE / 2 - 1) ? ", " : "\n");
        }

        // Hitung MDF dan MNF
        double mdf = calculateMedianFrequency(vReal, SAMPLE_SIZE, SAMPLING_FREQUENCY);
        double mnf = calculateMeanFrequency(vReal, SAMPLE_SIZE, SAMPLING_FREQUENCY);


        if (currentTime - lastDisplayTime >= 850) {
          lastDisplayTime = currentTime; 
         Serial.print("Nilai Median Frequency: ");
         Serial.println(mdf);
         Serial.print("Nilai Mean Frequency: ");
         Serial.println(mnf);

        // Deteksi kelelahan dan kontrol LED
        if (mdf < thresholdMDF || mnf < thresholdMNF) {
          Serial.println("Lelah");
          digitalWrite(LED_PIN, HIGH); // Nyalakan LED jika lelah
        } else {
          Serial.println("Tidak Lelah");
          digitalWrite(LED_PIN, LOW); // Matikan LED jika tidak lelah
        }

        // Reset index buffer
        bufferIndex = 0;

        
        }
      }
    }
  }
}

// Fungsi untuk menghitung Median Frequency (MDF)
double calculateMedianFrequency(double* magnitudes, int size, double samplingRate) {
  double totalEnergy = 0;
  for (int i = 0; i < size / 2; i++) {
    totalEnergy += magnitudes[i];
  }

  double halfEnergy = totalEnergy / 2.0;
  double cumulativeEnergy = 0;

  for (int i = 0; i < size / 2; i++) {
    cumulativeEnergy += magnitudes[i];
    if (cumulativeEnergy >= halfEnergy) {
      return (i * (samplingRate / size));
    }
  }
  return 0;
}

// Fungsi untuk menghitung Mean Frequency (MNF)
double calculateMeanFrequency(double* magnitudes, int size, double samplingRate) {
  double sumMagnitudes = 0;
  double weightedSum = 0;

  for (int i = 0; i < size / 2; i++) {
    double frequency = i * (samplingRate / size);
    sumMagnitudes += magnitudes[i];
    weightedSum += magnitudes[i] * frequency;
  }

  return (sumMagnitudes != 0) ? (weightedSum / sumMagnitudes) : 0;
}

#include <Arduino.h>
#include <Wire.h>

// Teste de bancada do ESP32 com o GY-521/MPU6050.
// Nesta etapa nenhum pino de motor ou ESC e acionado.

namespace {

uint8_t mpuAddress = 0x68;
constexpr int kSdaPin = 21;
constexpr int kSclPin = 22;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kSamplePeriodUs = 5000;   // 200 Hz
constexpr uint32_t kPrintPeriodMs = 100;     // 10 Hz
constexpr uint16_t kCalibrationSamples = 1000;

constexpr uint8_t kRegSampleRateDivider = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutHigh = 0x3B;
constexpr uint8_t kRegPowerManagement1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

// Escalas configuradas abaixo: acelerometro +/-4 g e giroscopio +/-500 graus/s.
constexpr float kAccelScale = 8192.0f;
constexpr float kGyroScale = 65.5f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kComplementaryGyroWeight = 0.98f;

struct ImuRaw {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t temperature;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

struct GyroBias {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

GyroBias gyroBias;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
uint32_t previousSampleUs = 0;
uint32_t previousPrintMs = 0;
uint8_t consecutiveReadErrors = 0;
bool attitudeInitialized = false;

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
  return static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8) |
                              lowByte);
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  constexpr size_t kRegisterBytesToRead = 1;
  if (Wire.requestFrom(mpuAddress, kRegisterBytesToRead, true) !=
      kRegisterBytesToRead) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool readImu(ImuRaw &sample) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(kRegAccelXoutHigh);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  constexpr size_t kBytesToRead = 14;
  if (Wire.requestFrom(mpuAddress, kBytesToRead, true) != kBytesToRead) {
    return false;
  }

  uint8_t data[kBytesToRead];
  for (size_t index = 0; index < kBytesToRead; ++index) {
    data[index] = Wire.read();
  }

  sample.accelX = combineBytes(data[0], data[1]);
  sample.accelY = combineBytes(data[2], data[3]);
  sample.accelZ = combineBytes(data[4], data[5]);
  sample.temperature = combineBytes(data[6], data[7]);
  sample.gyroX = combineBytes(data[8], data[9]);
  sample.gyroY = combineBytes(data[10], data[11]);
  sample.gyroZ = combineBytes(data[12], data[13]);
  return true;
}

bool findMpuAddress() {
  bool foundAnyDevice = false;
  bool foundMpu = false;

  Serial.println("Procurando dispositivos no barramento I2C...");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t result = Wire.endTransmission(true);
    if (result == 0) {
      foundAnyDevice = true;
      Serial.printf("Dispositivo I2C encontrado em 0x%02X\n", address);
      if (address == 0x68 || address == 0x69) {
        mpuAddress = address;
        foundMpu = true;
      }
    }
  }

  if (!foundAnyDevice) {
    Serial.println("Nenhum dispositivo I2C respondeu.");
  } else if (!foundMpu) {
    Serial.println("Ha dispositivo I2C, mas nenhum MPU6050 em 0x68 ou 0x69.");
  }
  return foundMpu;
}

bool initializeMpu() {
  if (!findMpuAddress()) {
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readRegister(kRegWhoAmI, whoAmI)) {
    Serial.printf("ERRO: falha ao ler o MPU6050 em 0x%02X.\n", mpuAddress);
    return false;
  }

  Serial.printf("MPU6050 detectado em 0x%02X. WHO_AM_I=0x%02X\n", mpuAddress,
                whoAmI);
  if (whoAmI != 0x68) {
    Serial.println("ERRO: identificador inesperado; confira o modulo e a fiacao.");
    return false;
  }

  // Reset, selecao do relogio PLL, filtro passa-baixas e escalas dos sensores.
  if (!writeRegister(kRegPowerManagement1, 0x80)) {
    return false;
  }
  delay(100);

  const bool configured =
      writeRegister(kRegPowerManagement1, 0x01) &&
      writeRegister(kRegConfig, 0x03) &&
      writeRegister(kRegSampleRateDivider, 0x04) &&
      writeRegister(kRegGyroConfig, 0x08) &&
      writeRegister(kRegAccelConfig, 0x08);

  delay(50);
  return configured;
}

bool calibrateGyroscope() {
  Serial.println("Calibrando giroscopio por aproximadamente 2 segundos.");
  Serial.println("Mantenha o sensor completamente parado e nivelado.");

  int64_t sumX = 0;
  int64_t sumY = 0;
  int64_t sumZ = 0;
  uint16_t validSamples = 0;

  for (uint16_t attempt = 0; attempt < kCalibrationSamples; ++attempt) {
    ImuRaw sample{};
    if (readImu(sample)) {
      sumX += sample.gyroX;
      sumY += sample.gyroY;
      sumZ += sample.gyroZ;
      ++validSamples;
    }
    delay(2);
  }

  if (validSamples < kCalibrationSamples * 9 / 10) {
    Serial.println("ERRO: leituras insuficientes durante a calibracao.");
    return false;
  }

  gyroBias.x = static_cast<float>(sumX) / validSamples;
  gyroBias.y = static_cast<float>(sumY) / validSamples;
  gyroBias.z = static_cast<float>(sumZ) / validSamples;

  Serial.printf("Calibracao concluida: X=%.2f, Y=%.2f, Z=%.2f\n",
                gyroBias.x, gyroBias.y, gyroBias.z);
  return true;
}

void updateAttitude(const ImuRaw &sample, float deltaSeconds) {
  const float accelX = sample.accelX / kAccelScale;
  const float accelY = sample.accelY / kAccelScale;
  const float accelZ = sample.accelZ / kAccelScale;
  const float gyroX = (sample.gyroX - gyroBias.x) / kGyroScale;
  const float gyroY = (sample.gyroY - gyroBias.y) / kGyroScale;

  const float accelRoll = atan2f(accelY, accelZ) * kRadToDeg;
  const float accelPitch =
      atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * kRadToDeg;

  if (!attitudeInitialized) {
    rollDeg = accelRoll;
    pitchDeg = accelPitch;
    attitudeInitialized = true;
    return;
  }

  rollDeg = kComplementaryGyroWeight * (rollDeg + gyroX * deltaSeconds) +
            (1.0f - kComplementaryGyroWeight) * accelRoll;
  pitchDeg = kComplementaryGyroWeight * (pitchDeg + gyroY * deltaSeconds) +
             (1.0f - kComplementaryGyroWeight) * accelPitch;
}

void printSample(const ImuRaw &sample) {
  const float accelX = sample.accelX / kAccelScale;
  const float accelY = sample.accelY / kAccelScale;
  const float accelZ = sample.accelZ / kAccelScale;
  const float gyroX = (sample.gyroX - gyroBias.x) / kGyroScale;
  const float gyroY = (sample.gyroY - gyroBias.y) / kGyroScale;
  const float gyroZ = (sample.gyroZ - gyroBias.z) / kGyroScale;
  const float temperatureC = sample.temperature / 340.0f + 36.53f;

  Serial.printf(
      "roll:%.2f,pitch:%.2f,ax:%.3f,ay:%.3f,az:%.3f,gx:%.2f,gy:%.2f,"
      "gz:%.2f,temp:%.2f\n",
      rollDeg, pitchDeg, accelX, accelY, accelZ, gyroX, gyroY, gyroZ,
      temperatureC);
}

[[noreturn]] void stopWithError(const char *message) {
  while (true) {
    Serial.println();
    Serial.println(message);
    Serial.println(
        "Teste interrompido. Corrija a ligacao e pressione o botao EN/RESET.");
    Serial.flush();
    delay(1000);
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  // Da tempo para o Monitor Serial reabrir depois do upload.
  delay(2500);

  Serial.println();
  Serial.println("=== Teste de bancada ESP32 + GY-521/MPU6050 ===");
  Serial.println("Ligacao: VCC=3V3, GND=GND, SDA=GPIO21, SCL=GPIO22");

  Wire.begin(kSdaPin, kSclPin);
  // 100 kHz e mais tolerante a jumpers longos e contatos de protoboard.
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  if (!initializeMpu()) {
    stopWithError("ERRO: nao foi possivel configurar o MPU6050.");
  }
  if (!calibrateGyroscope()) {
    stopWithError("ERRO: falha na calibracao do MPU6050.");
  }

  previousSampleUs = micros();
  previousPrintMs = millis();
  Serial.println("Teste iniciado. Abra o Serial Plotter em 115200 baud.");
}

void loop() {
  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - previousSampleUs) < kSamplePeriodUs) {
    return;
  }

  float deltaSeconds = static_cast<uint32_t>(nowUs - previousSampleUs) * 1.0e-6f;
  previousSampleUs = nowUs;

  // Evita integrar um intervalo anormal depois de uma pausa longa.
  if (deltaSeconds > 0.05f) {
    deltaSeconds = kSamplePeriodUs * 1.0e-6f;
  }

  ImuRaw sample{};
  if (!readImu(sample)) {
    ++consecutiveReadErrors;
    if (consecutiveReadErrors >= 10) {
      stopWithError("ERRO: comunicacao com o MPU6050 foi perdida.");
    }
    return;
  }
  consecutiveReadErrors = 0;

  updateAttitude(sample, deltaSeconds);

  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - previousPrintMs) >= kPrintPeriodMs) {
    previousPrintMs = nowMs;
    printSample(sample);
  }
}

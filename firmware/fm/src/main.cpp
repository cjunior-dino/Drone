#include <Arduino.h>

#include "control_commands.h"
#include "imu.h"
#include "motor_output.h"
#include "receiver.h"
#include "safety.h"

// Teste de bancada do ESP32 com o GY-521/MPU6050 e um ESC sem helice.

namespace {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kSamplePeriodUs = 5000;  // 200 Hz
constexpr uint32_t kPrintPeriodMs = 100;    // 10 Hz

uint32_t previousSampleUs = 0;
uint32_t previousPrintMs = 0;
uint8_t consecutiveReadErrors = 0;

void printSample(const imu::Sample &sample) {
  const uint32_t nowMs = millis();
  const receiver::Frame receiverFrame = receiver::getFrame();
  const receiver::Diagnostics receiverDiagnostics = receiver::getDiagnostics();
  const control::Commands commands = control::fromReceiver(receiverFrame);
  const motor_output::Status motorStatus = motor_output::getStatus();

  Serial.printf(
      "state:%s,state_ms:%lu,outputs_enabled:%u,motor1_signal:%u,"
      "motor1_drive:%u,motor1_request:%.3f,motor1_us:%u,"
      "motor1_limit_us:%u,rx_ok:%u,ch1_us:%lu,ch2_us:%lu,"
      "ch3_us:%lu,"
      "ch4_us:%lu,roll_cmd:%.3f,pitch_cmd:%.3f,throttle_cmd:%.3f,"
      "yaw_cmd:%.3f,rx_reject_ch1:%lu,rx_reject_ch2:%lu,"
      "rx_reject_ch3:%lu,rx_reject_ch4:%lu,rx_failsafe_events:%lu,"
      "rx_recovery_events:%lu,rx_valid_ch1:%lu,rx_valid_ch2:%lu,"
      "rx_valid_ch3:%lu,rx_valid_ch4:%lu,rx_short_ch1:%lu,"
      "rx_short_ch2:%lu,rx_short_ch3:%lu,rx_short_ch4:%lu,"
      "rx_long_ch1:%lu,rx_long_ch2:%lu,rx_long_ch3:%lu,rx_long_ch4:%lu,"
      "rx_orphan_ch1:%lu,rx_orphan_ch2:%lu,rx_orphan_ch3:%lu,"
      "rx_orphan_ch4:%lu,roll:%.2f,"
      "pitch:%.2f,ax:%.3f,ay:%.3f,az:%.3f,gx:%.2f,gy:%.2f,gz:%.2f,"
      "temp:%.2f\n",
      safety::flightStateName(),
      static_cast<unsigned long>(safety::stateElapsedMs(nowMs)),
      safety::outputsEnabled() ? 1U : 0U,
      motorStatus.signalActive ? 1U : 0U,
      motorStatus.driveEnabled ? 1U : 0U, motorStatus.requestedThrottle,
      static_cast<unsigned int>(motorStatus.pulseUs),
      static_cast<unsigned int>(motorStatus.limitUs),
      receiverFrame.signalValid ? 1U : 0U,
      static_cast<unsigned long>(receiverFrame.channel1Us),
      static_cast<unsigned long>(receiverFrame.channel2Us),
      static_cast<unsigned long>(receiverFrame.channel3Us),
      static_cast<unsigned long>(receiverFrame.channel4Us),
      commands.roll, commands.pitch, commands.throttle, commands.yaw,
      static_cast<unsigned long>(receiverDiagnostics.rejectedChannel1),
      static_cast<unsigned long>(receiverDiagnostics.rejectedChannel2),
      static_cast<unsigned long>(receiverDiagnostics.rejectedChannel3),
      static_cast<unsigned long>(receiverDiagnostics.rejectedChannel4),
      static_cast<unsigned long>(receiverDiagnostics.failsafeEvents),
      static_cast<unsigned long>(receiverDiagnostics.recoveryEvents),
      static_cast<unsigned long>(receiverDiagnostics.validChannel1),
      static_cast<unsigned long>(receiverDiagnostics.validChannel2),
      static_cast<unsigned long>(receiverDiagnostics.validChannel3),
      static_cast<unsigned long>(receiverDiagnostics.validChannel4),
      static_cast<unsigned long>(receiverDiagnostics.shortChannel1),
      static_cast<unsigned long>(receiverDiagnostics.shortChannel2),
      static_cast<unsigned long>(receiverDiagnostics.shortChannel3),
      static_cast<unsigned long>(receiverDiagnostics.shortChannel4),
      static_cast<unsigned long>(receiverDiagnostics.longChannel1),
      static_cast<unsigned long>(receiverDiagnostics.longChannel2),
      static_cast<unsigned long>(receiverDiagnostics.longChannel3),
      static_cast<unsigned long>(receiverDiagnostics.longChannel4),
      static_cast<unsigned long>(receiverDiagnostics.orphanFallChannel1),
      static_cast<unsigned long>(receiverDiagnostics.orphanFallChannel2),
      static_cast<unsigned long>(receiverDiagnostics.orphanFallChannel3),
      static_cast<unsigned long>(receiverDiagnostics.orphanFallChannel4),
      sample.rollDeg, sample.pitchDeg, sample.accelX, sample.accelY,
      sample.accelZ, sample.gyroX, sample.gyroY, sample.gyroZ,
      sample.temperatureC);
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  if (!motor_output::begin()) {
    safety::stopWithError("ERRO: nao foi possivel iniciar o PWM do Motor 1.");
  }
  // Da tempo para o Monitor Serial reabrir depois do upload.
  delay(2500);

  Serial.println();
  Serial.println("=== Teste de bancada ESP32 + GY-521 + FS-iA6 ===");
  Serial.println("Ligacao: VCC=3V3, GND=GND, SDA=GPIO21, SCL=GPIO22");
  Serial.println(
      "Receptor: CH1=GPIO27, CH2=GPIO26, CH3=GPIO25, CH4=GPIO33");
  Serial.println(
      "Protecao RX: filtro de pulsos ativo e failsafe apos 100 ms sem sinal");
  Serial.println(
      "Motor 1: GPIO32, PWM 50 Hz, faixa de bancada limitada a 1000-1100 us");

  receiver::begin();

  if (!imu::begin()) {
    safety::stopWithError("ERRO: nao foi possivel configurar o MPU6050.");
  }
  if (!imu::calibrateGyroscope()) {
    safety::stopWithError("ERRO: falha na calibracao do MPU6050.");
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

  float deltaSeconds =
      static_cast<uint32_t>(nowUs - previousSampleUs) * 1.0e-6f;
  previousSampleUs = nowUs;

  receiver::update();
  const receiver::Frame receiverFrame = receiver::getFrame();
  const control::Commands commands = control::fromReceiver(receiverFrame);
  safety::update(receiverFrame.signalValid, commands, millis());
  motor_output::update(safety::outputsEnabled(), receiverFrame.signalValid,
                       commands.throttle, deltaSeconds);

  // Evita integrar um intervalo anormal depois de uma pausa longa.
  if (deltaSeconds > 0.05f) {
    deltaSeconds = kSamplePeriodUs * 1.0e-6f;
  }

  imu::Sample sample{};
  if (!imu::update(deltaSeconds, sample)) {
    ++consecutiveReadErrors;
    if (consecutiveReadErrors >= 10) {
      safety::stopWithError("ERRO: comunicacao com o MPU6050 foi perdida.");
    }
    return;
  }
  consecutiveReadErrors = 0;

  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - previousPrintMs) >= kPrintPeriodMs) {
    previousPrintMs = nowMs;
    printSample(sample);
  }
}

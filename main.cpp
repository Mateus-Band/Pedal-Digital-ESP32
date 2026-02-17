#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include <WiFi.h>
#include "esp_bt.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.hpp>

// ==========================================
// CONFIGURAÇÕES DE PINO
// ==========================================
#define PIN_IR 4
#define I2S_BCK 27
#define I2S_WS  26
#define I2S_DO  25
#define I2S_DI  35

// ==========================================
// VARIÁVEIS COMPARTILHADAS (VOLATILE)
// ==========================================
// Essas variáveis são controladas pelo Core 0 e lidas pelo Core 1
volatile float input_gain    = 0.4f;   
volatile float output_volume = 0.8f;   

volatile bool ativar_distorcao = false;  
volatile bool ativar_cab       = true;  
volatile bool ativar_delay     = false;
volatile bool ativar_reverb    = false; 

// ==========================================
// DISPLAY OLED
// ==========================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==========================================
// PARÂMETROS DE ÁUDIO (Do seu código V8)
// ==========================================
#define SAMPLE_RATE 48000 
#define BIT_BOOST 3
#define DAC_LIMIT 1600000000

// GATE (Mantive seu valor alto)
#define GATE_THRESHOLD 100000000 
#define GATE_HOLD_TIME 9000

// ==========================================
// FILTRO TIJOLO (Do seu código V8)
// ==========================================
float lpf_prev = 0.0f;

int32_t filtroTijolo(int32_t input) {
  // Mantive seu alpha de 0.1f. 
  // Se achar muito abafado, mude para 0.3f aqui.
  float alpha = 0.1f; 
  
  float filtered = lpf_prev + (alpha * ((float)input - lpf_prev));
  lpf_prev = filtered;
  return (int32_t)filtered;
}

// ==========================================
// EFEITOS (Do seu código V8)
// ==========================================
float dist_drive  = 7.0f; 
int32_t dist_clip = 1000000000;

#define DELAY_MAX 24000 
int32_t *delayBuffer = NULL;
int delayHead = 0;
int delay_time = 12000;
float delay_feedback = 0.4f;
float delay_mix = 0.3f;

#define REVERB_MAX 7000
int32_t *reverbBuffer = NULL;

// CAB IR
#define IR_LEN 128
float cabIR[IR_LEN] = {
   0.000000f,  0.000125f, -0.000412f,  0.000854f, -0.002156f,  0.005412f, -0.012541f,  0.035412f,
  -0.125412f,  0.354125f,  0.654125f,  0.254125f, -0.354125f, -0.451254f, -0.154125f,  0.125412f,
   0.254125f,  0.185412f, -0.054125f, -0.185412f, -0.154125f, -0.025412f,  0.125412f,  0.154125f,
   0.085412f, -0.025412f, -0.112541f, -0.095412f, -0.015412f,  0.065412f,  0.085412f,  0.035412f,
  -0.035412f, -0.065412f, -0.045412f,  0.005412f,  0.045412f,  0.055412f,  0.025412f, -0.015412f,
  -0.045412f, -0.035412f, -0.005412f,  0.025412f,  0.035412f,  0.015412f, -0.015412f, -0.025412f,
  -0.015412f,  0.005412f,  0.025412f,  0.025412f,  0.005412f, -0.015412f, -0.025412f, -0.015412f,
   0.005412f,  0.015412f,  0.015412f,  0.005412f, -0.005412f, -0.015412f, -0.015412f, -0.005412f,
   0.005412f,  0.012541f,  0.012541f,  0.005412f, -0.005412f, -0.012541f, -0.012541f, -0.005412f,
   0.005412f,  0.008541f,  0.008541f,  0.005412f, -0.005412f, -0.008541f, -0.008541f, -0.005412f,
   0.005412f,  0.006541f,  0.006541f,  0.005412f, -0.005412f, -0.006541f, -0.006541f, -0.005412f,
   0.002541f,  0.004541f,  0.004541f,  0.002541f, -0.002541f, -0.004541f, -0.004541f, -0.002541f,
   0.001541f,  0.002541f,  0.002541f,  0.001541f, -0.001541f, -0.002541f, -0.002541f, -0.001541f,
   0.000541f,  0.001541f,  0.001541f,  0.000541f, -0.000541f, -0.001541f, -0.001541f, -0.000541f,
   0.000254f,  0.000541f,  0.000541f,  0.000254f, -0.000254f, -0.000541f, -0.000541f, -0.000254f,
   0.000000f,  0.000125f,  0.000125f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f
};

static int32_t irBuffer[IR_LEN];
static int irIndex = 0;

int32_t applyCabIR(int32_t input) {
  irBuffer[irIndex] = input;
  float acc = 0.0f;
  int idx = irIndex;
  for (int i = 0; i < IR_LEN; i++) {
    acc += cabIR[i] * (float)irBuffer[idx];
    if (--idx < 0) idx = IR_LEN - 1;
  }
  irIndex = (irIndex + 1) % IR_LEN;
  return (int32_t)(acc * 1.5f);
}

float dc_estimate = 0.0f;

// ==========================================
// FUNÇÃO DESENHO (Rodando no Core 0)
// ==========================================
void desenharTela() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // VOL
  display.setTextSize(1); display.setCursor(0, 0); display.print("VOL:");
  display.setTextSize(2); display.setCursor(26, 0); display.print((int)(output_volume * 100));

  // GAIN
  display.setTextSize(1); display.setCursor(68, 0); display.print("GAIN:");
  display.setTextSize(2); display.setCursor(98, 0); display.print((int)(input_gain * 100));

  // BARRA
  display.drawRect(0, 18, 128, 6, SSD1306_WHITE);
  int w = map((int)(output_volume * 100), 0, 200, 0, 124);
  if(w>124) w=124;
  display.fillRect(2, 20, w, 2, SSD1306_WHITE);

  // EFEITOS
  display.setTextSize(1);
  int x1=0, x2=64, y1=32, y2=48;
  display.setCursor(x1, y1); if(ativar_cab) display.print("[CAB]"); else display.print(" ... ");
  display.setCursor(x1, y2); if(ativar_distorcao) display.print("[DRV]"); else display.print(" ... ");
  display.setCursor(x2, y1); if(ativar_reverb) display.print("[REV]"); else display.print(" ... ");
  display.setCursor(x2, y2); if(ativar_delay) display.print("[DLY]"); else display.print(" ... ");

  display.display();
}

// ==========================================
// TAREFA DE INTERFACE (CORE 0)
// ==========================================
void TaskInterface(void *pvParameters) {
  IrReceiver.begin(PIN_IR, ENABLE_LED_FEEDBACK);
  desenharTela();

  for (;;) {
    if (IrReceiver.decode()) {
      uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
      bool alterou = false;

      // MAPA DE BOTÕES
      if (codigo == 0xF30CFF00) { output_volume += 0.1f; if(output_volume > 3.0f) output_volume = 3.0f; alterou = true; } 
      else if (codigo == 0xE718FF00) { output_volume -= 0.1f; if(output_volume < 0.0f) output_volume = 0.0f; alterou = true; } 
      
      else if (codigo == 0xA15EFF00) { input_gain += 0.1f; if(input_gain > 5.0f) input_gain = 5.0f; alterou = true; } 
      else if (codigo == 0xF708FF00) { input_gain -= 0.1f; if(input_gain < 0.0f) input_gain = 0.0f; alterou = true; } 
      
      else if (codigo == 0xE31CFF00) { ativar_reverb = !ativar_reverb; alterou = true; } 
      else if (codigo == 0xA55AFF00) { ativar_distorcao = !ativar_distorcao; alterou = true; } 
      else if (codigo == 0xBD42FF00) { ativar_delay = !ativar_delay; alterou = true; } 
      else if (codigo == 0xAD52FF00) { ativar_cab = !ativar_cab; alterou = true; }

      if (alterou) {
        desenharTela();
      }
      IrReceiver.resume();
    }
    // Delay para não travar o watchdog do Core 0
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// SETUP (CORE 1)
// ==========================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  btStop();

  // 1. Memória
  delayBuffer  = (int32_t *)calloc(DELAY_MAX, sizeof(int32_t));
  reverbBuffer = (int32_t *)calloc(REVERB_MAX, sizeof(int32_t));
  if (!delayBuffer || !reverbBuffer) while (1);

  // 2. Display (Inicia Hardware)
  Wire.begin(21, 22); 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("Err Display");

  // 3. I2S Config
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = SAMPLE_RATE * 256
  };
  i2s_pin_config_t pins = { .bck_io_num = I2S_BCK, .ws_io_num = I2S_WS, .data_out_num = I2S_DO, .data_in_num = I2S_DI };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);

  // 4. ATIVA CLOCK ADC
  REG_WRITE(PIN_CTRL, 0xFF0); 
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

  // 5. INICIA CORE 0 (INTERFACE)
  xTaskCreatePinnedToCore(TaskInterface, "Interface", 10000, NULL, 1, NULL, 0);

  Serial.println("--- V15: BASE V8 + DUAL CORE ---");
}

// ==========================================
// LOOP DE ÁUDIO (CORE 1)
// ==========================================
void loop() {
  int32_t io[128];
  size_t br, bw;
  static int gate_timer = 0;

  i2s_read(I2S_NUM_0, io, sizeof(io), &br, portMAX_DELAY);
  if (br == 0) return;

  int frames = br / 8;

  for (int i = 0; i < frames; i++) {
    int L = i * 2;
    int R = L + 1;

    // 1. Entrada + Boost
    int32_t raw = io[L] << BIT_BOOST;

    // 2. DC Removal
    dc_estimate = 0.9995f * dc_estimate + 0.0005f * (float)raw;
    float sig = ((float)raw - dc_estimate) * input_gain;

    // 3. Gate
    if (fabs(sig) > GATE_THRESHOLD) gate_timer = GATE_HOLD_TIME;
    if (gate_timer > 0) gate_timer--;
    else sig = 0.0f;

    // 4. Distorção
    if (ativar_distorcao) {
      float x = sig / (float)dist_clip * dist_drive;
      sig = (x / (1.0f + fabs(x))) * (float)dist_clip;
    }

    int32_t s = (int32_t)sig;

    // 5. CAB IR
    if (ativar_cab) s = applyCabIR(s);

    // 6. Delay
    if (ativar_delay) {
      int rp = (delayHead - delay_time + DELAY_MAX) % DELAY_MAX;
      int32_t d = delayBuffer[rp];
      delayBuffer[delayHead] = s + (int32_t)(d * delay_feedback);
      s += (int32_t)(d * delay_mix);
      delayHead = (delayHead + 1) % DELAY_MAX;
    }

    // 7. Filtro Tijolo
    s = filtroTijolo(s);

    // 8. Limiter
    if (s > DAC_LIMIT) s = DAC_LIMIT;
    if (s < -DAC_LIMIT) s = -DAC_LIMIT;

    io[L] = (int32_t)(s * output_volume);
    io[R] = io[L]; 
  }

  i2s_write(I2S_NUM_0, io, br, &bw, portMAX_DELAY);
}
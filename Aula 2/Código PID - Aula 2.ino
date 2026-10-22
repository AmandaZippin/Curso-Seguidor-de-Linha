// Robô Seguidor de Linha - Arduino Nano
// Sensores: A1-A6 (6 sensores analógicos)
// Botão: D12 (pull-down)
// Motor Driver TB6612FNG: PWMA(D5), AI1(D9), AI2(D4), PWMB(D6), BI1(D7), BI2(D8)

#define NUM_SEN 6
#define BUTTON_PIN 12

// Pinos do motor driver TB6612FNG
#define PWMA 5   // PWM Motor A (esquerda)
#define AI1 9    // Direção Motor A
#define AI2 4    // Direção Motor A
#define PWMB 6   // PWM Motor B (direita)
#define BI1 7    // Direção Motor B
#define BI2 8    // Direção Motor B

// Pinos dos sensores (A1 a A6)
int pinoSensores[NUM_SEN] = {A1, A2, A3, A4, A5, A6};

// Variáveis para calibração
int minSensor[NUM_SEN];
int maxSensor[NUM_SEN];
int valorSensores[NUM_SEN];

// Variáveis para controle PID
float Kp = 0.0;    // Constante proporcional 15
float Kd = 0.0;    // Constante diferencial 30
float erro = 0;
float erroAnterior = 0;

// Velocidades dos motores
int velBase = 80;     // Velocidade base (0-255)
int velMax = 200;     // Velocidade máxima
int velMin = -100;    // Velocidade mínima

// Estados do programa
enum Estado {
  ESPERANDO,
  CALIBRANDO,
  CORRENDO
};

Estado estadoAtual = ESPERANDO;
bool botaoAtual = false;
bool botaoEstadoAnterior = false;

// Protótipos das funções
void lerEMaisMinMax();
void acelerar(int velEsquerda, int velDireita);
void pararMotores();
void calibrarSensores();
void seguirLinha();
void lerSensores();
float controleLinha();


void setup() {
  Serial.begin(9600);
  
  // Configuração dos pinos
  pinMode(BUTTON_PIN, INPUT);
  
  // Configuração dos pinos do motor
  pinMode(PWMA, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  
  pararMotores();
  
  // Inicializar valores de calibração
  for (int i = 0; i < NUM_SEN; i++) {
    minSensor[i] = 1023;
    maxSensor[i] = 0;
  }
  
  Serial.println("Sistema iniciado. Pressione o botão para calibrar.");
}

void loop() {
  // Leitura do botão
  bool estadoBotao = digitalRead(BUTTON_PIN);
  
  // Detecção de borda de subida do botão
  if (estadoBotao && !botaoEstadoAnterior) {
    botaoAtual = true;
    delay(50); // Debounce
  }
  botaoEstadoAnterior = estadoBotao;
  
  // Máquina de estados
  switch (estadoAtual) {
    case ESPERANDO:
      

      if (botaoAtual) {
        estadoAtual = CALIBRANDO;
        botaoAtual = false;
        Serial.println("\nIniciando calibração...");
        delay(500);
        calibrarSensores();
      }
      break;
      
    case CALIBRANDO:
      estadoAtual = CORRENDO;
      Serial.println("Calibração concluída. Iniciando corrida!");
      delay(3000); // Aguarda 3s antes da corrida
      break;
      
    case CORRENDO:
      if (botaoAtual) {
        estadoAtual = ESPERANDO;
        botaoAtual = false;
        pararMotores();
        Serial.println("Parando robô. Pressione o botão para recalibrar.");
      } else {
        seguirLinha();
  
      }
      break;
  }
}


void calibrarSensores() {
  Serial.println("Girando robô para calibração...");
  
  unsigned long tempoInicial = millis();
  
  // Girar para a direita por 3 segundos
  acelerar(60, -60);
  while (millis() - tempoInicial < 5000) {
    lerEMaisMinMax();
    delay(10);
  }
  
  // Girar para a esquerda por 3 segundos
  tempoInicial = millis();
  acelerar(-60, 60);
  while (millis() - tempoInicial < 3000) {
    lerEMaisMinMax();
    delay(10);
  }
  
  pararMotores();

  // Mostrar valores calculados na calibração
 /* Serial.println("\n--- VALORES DE CALIBRAÇÃO GUARDADOS ---");
  for (int i = 0; i < NUM_SEN; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": Min=");
    Serial.print(minSensor[i]);
    Serial.print(" | Max=");
    Serial.println(maxSensor[i]);
  }
  Serial.println("----------------------------------------\n");
  
  delay(1000);
  */
}

void lerEMaisMinMax() {
  for (int i = 0; i < NUM_SEN; i++) {
    int faixa = analogRead(pinoSensores[i]);
    if (faixa < minSensor[i]) {
      minSensor[i] = faixa;
    }
    if (faixa > maxSensor[i]) {
      maxSensor[i] = faixa;
    }
  }
}

void seguirLinha() {
  lerSensores();
  
  float posicao = controleLinha();
  
  erro = posicao - 2.5; // Centro dos 6 sensores (0 a 5)
  
  float derivativo = erro - erroAnterior;
  float pid = (Kp * erro) + (Kd * derivativo);
  erroAnterior = erro;
  
  int velEsquerda = velBase + pid;
  int velDireita = velBase - pid;
  
  velEsquerda = constrain(velEsquerda, velMin, velMax);
  velDireita = constrain(velDireita, velMin, velMax);
  
  acelerar(velEsquerda, velDireita);
}

void lerSensores() {
  for (int i = 0; i < NUM_SEN; i++) {
    int leitura = analogRead(pinoSensores[i]);
    // Normalização invertida: 1000 para linha branca e 0 para fundo preto
    valorSensores[i] = map(leitura, minSensor[i], maxSensor[i], 0, 1000);
    valorSensores[i] = constrain(valorSensores[i], 0, 1000);
  }
}

float controleLinha() {
  long num = 0;
  long den = 0;
  
  for (int i = 0; i < NUM_SEN; i++) {
    num += (long)valorSensores[i] * i * 1000;
    den += valorSensores[i];
  }
  
  if (den == 0) {
    return 2.5; 
  }
  
  return (float)num / den / 1000.0;
}

void acelerar(int velEsquerda, int velDireita) {
  // Motor esquerdo (canal A)
  if (velEsquerda >= 0) {
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
    analogWrite(PWMA, velEsquerda);
  } else {
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    analogWrite(PWMA, -velEsquerda);
  }
  
  // Motor direito (canal B)
  if (velDireita >= 0) {
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
    analogWrite(PWMB, velDireita);
  } else {
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    analogWrite(PWMB, -velDireita);
  }
}

void pararMotores() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, LOW);
  digitalWrite(BI1, LOW);
  digitalWrite(BI2, LOW);
}
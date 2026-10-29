// Robô Seguidor de Linha - Arduino Nano
// Sensores centrais: A1-A6 | Sensores laterais: A0 (direita) e A7 (esquerda)
// Botão: D12 (pull-down)
// Motor Driver TB6612FNG: PWMA(D5), AI1(D9), AI2(D4), PWMB(D6), BI1(D7), BI2(D8)


// --- Trechos da pista (sensor esquerdo) ---
// N_FAIXA = quantidade de marcas da esquerda + 1 (número de trechos).
// As 3 listas abaixo DEVEM ter exatamente N_FAIXA valores.
// Posição 0 = antes da 1ª marca, posição 1 = depois da 1ª marca, e assim por diante.

#define N_FAIXA 0
int   velTrajeto[N_FAIXA] = {};     // velocidade de cada trecho (0-255)
float kpTrajeto[N_FAIXA]  = {};     // Kp de cada trecho
float kdTrajeto[N_FAIXA]  = {};     // Kd de cada trecho

//  Valores padrão 
float Kp = 0.0;       // Proporcional
float Kd = 0.0;       // Derivativo

int velBase = 80;      // Velocidade padrão (0-255)

// --- Limites dos motores ---
int velMax = 200;      // Velocidade máxima (mantenha as velocidades dos trechos abaixo disso)
int velMin = -100;     // Velocidade mínima 

// --- Marca de chegada (sensor direito) ---
#define N_PARADA 4     // O robô para quando o sensor direito contar esta quantidade de marcas

// --- Detecção das marcas laterais ---
#define LIMIAR_MARCA 230                      // valor (0-1000) abaixo do qual o sensor lateral "vê" a marca
const unsigned long intervaloDir = 50;        // ms mínimos entre duas contagens (direita)
const unsigned long intervaloEsq = 100;       // ms mínimos entre duas contagens (esquerda)
 // Aumente se uma marca conta 2x; diminua se perde marcas.

// --- Saída da pista ---
const unsigned long tempoMaxFora = 1500;      // ms sem ver a linha até desligar (1,5 s)
const int limiarLinha = 200;                  // valor (0-1000) acima do qual um sensor central "vê" a linha
 

#define NUM_SEN 6
#define BUTTON_PIN 12

// Pinos do motor driver TB6612FNG
#define PWMA 5   // PWM Motor A (esquerda)
#define AI1 9    // Direção Motor A
#define AI2 4    // Direção Motor A
#define PWMB 6   // PWM Motor B (direita)
#define BI1 7    // Direção Motor B
#define BI2 8    // Direção Motor B

// Pinos dos sensores centrais (A1 a A6)
int pinoSensores[NUM_SEN] = {A1, A2, A3, A4, A5, A6};

// Pinos dos sensores laterais (A0 e A7)
int sensorDir = A0;
int sensorEsq = A7;

// Calibração e leituras dos sensores centrais
int minSensor[NUM_SEN];
int maxSensor[NUM_SEN];
int valorSensores[NUM_SEN];

// Sensor lateral direito
int minDir;
int maxDir;
int valorDir;
int N_Dir = 0;            // marcas contadas na direita

// Sensor lateral esquerdo
int minEsq;
int maxEsq;
int valorEsq;
int N_Esq = 0;            // marcas contadas na esquerda (= trecho atual)

// Controle PID
float erro = 0;
float erroAnterior = 0;

// Controle de tempo das contagens
unsigned long ultimoTempoDir = 0;
unsigned long ultimoTempoEsq = 0;

// Controle de saída da pista
unsigned long inicioForaPista = 0;
bool foraDaPista = false;

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
void lerSensorLateral(int sensorLateral);
bool saiuDaPista();
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
  minEsq = 1023;
  maxEsq = 0;
  minDir = 1023;
  maxDir = 0;

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
      foraDaPista = false;   // começa a corrida sem contagem de "fora da pista"
      Serial.println("Calibração concluída. Iniciando corrida!");
      delay(3000); // Aguarda 3 s antes da corrida
      break;

    case CORRENDO:
      if (botaoAtual) {
        estadoAtual = ESPERANDO;
        botaoAtual = false;
        N_Dir = 0;
        N_Esq = 0;
        foraDaPista = false;
        pararMotores();
        Serial.println("Parando robô. Pressione o botão para recalibrar.");
      } else {
        seguirLinha();
        
        Serial.println(N_Dir);
        Serial.println(N_Esq);
        if (N_Dir == N_PARADA) {
          botaoAtual = true;   
        }
      }
      break;
  }
}

void calibrarSensores() {
  Serial.println("Girando robô para calibração...");

  unsigned long tempoInicial = millis();

  // Girar para a direita por 5 segundos
  acelerar(60, -60);
  while (millis() - tempoInicial < 5000) {
    lerEMaisMinMax();

    int faixaEsq = analogRead(sensorEsq);
    if (faixaEsq < minEsq) minEsq = faixaEsq;
    if (faixaEsq > maxEsq) maxEsq = faixaEsq;

    int faixaDir = analogRead(sensorDir);
    if (faixaDir < minDir) minDir = faixaDir;
    if (faixaDir > maxDir) maxDir = faixaDir;

    delay(10);
  }

  // Girar para a esquerda por 3 segundos
  tempoInicial = millis();
  acelerar(-60, 60);
  while (millis() - tempoInicial < 3000) {
    lerEMaisMinMax();

    int faixaEsq = analogRead(sensorEsq);
    if (faixaEsq < minEsq) minEsq = faixaEsq;
    if (faixaEsq > maxEsq) maxEsq = faixaEsq;

    int faixaDir = analogRead(sensorDir);
    if (faixaDir < minDir) minDir = faixaDir;
    if (faixaDir > maxDir) maxDir = faixaDir;

    delay(10);
  }

  pararMotores();
}

// Atualiza o mínimo e o máximo dos 6 sensores centrais (usado na calibração)
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

// Função principal da corrida: lê sensores, conta marcas, calcula o PID e move os motores
void seguirLinha() {

  lerSensores();

  // Se ficou mais de tempoMaxFora sem enxergar a linha, desliga o robô
  if (saiuDaPista()) {
    pararMotores();
    botaoAtual = true;   // o loop() volta o estado para ESPERANDO
    Serial.println("Fora da pista! Parando.");
    return;
  }

  // Sensores laterais: contam as marcas da pista
  lerSensorLateral(sensorEsq);
  lerSensorLateral(sensorDir);

  unsigned long agora = millis();

  Serial.println(valorDir);
  if (valorDir < LIMIAR_MARCA && (agora - ultimoTempoDir >= intervaloDir)) {
    N_Dir++;
    ultimoTempoDir = agora;
  }

  Serial.println(valorEsq);
  if (valorEsq < LIMIAR_MARCA && (agora - ultimoTempoEsq >= intervaloEsq)) {
    N_Esq++;
    ultimoTempoEsq = agora;
  }

  // Erro: posição da linha (0 a 5) menos o centro (2,5)
  float posicao = controleLinha();
  erro = posicao - 2.5;
  float derivativo = erro - erroAnterior;
  erroAnterior = erro;

  // Escolhe velocidade, Kp e Kd do trecho atual (N_Esq = nº de marcas já vistas)
  int   vel = velBase;   // valores padrão, se N_Esq passar do tamanho das listas
  float kp  = Kp;
  float kd  = Kd;

  if (N_Esq < N_FAIXA) {
    vel = velTrajeto[N_Esq];
    kp  = kpTrajeto[N_Esq];
    kd  = kdTrajeto[N_Esq];
  }

  // PID: a correção é somada num motor e subtraída no outro
  float pid = (kp * erro) + (kd * derivativo);

  int velEsquerda = vel + pid;
  int velDireita  = vel - pid;

  velEsquerda = constrain(velEsquerda, velMin, velMax);
  velDireita  = constrain(velDireita, velMin, velMax);

  acelerar(velEsquerda, velDireita);
}

// Lê os 6 sensores centrais e normaliza para 0-1000
void lerSensores() {
  for (int i = 0; i < NUM_SEN; i++) {
    int leitura = analogRead(pinoSensores[i]);
    // Normalização: 1000 para linha branca e 0 para fundo preto
    valorSensores[i] = map(leitura, minSensor[i], maxSensor[i], 0, 1000);
    valorSensores[i] = constrain(valorSensores[i], 0, 1000);
  }
}

bool saiuDaPista() {
  // A linha está visível se pelo menos 1 sensor passa do limiar
  bool linhaVisivel = false;
  for (int i = 0; i < NUM_SEN; i++) {
    if (valorSensores[i] > limiarLinha) {   
      linhaVisivel = true;
      break;
    }
  }

  if (linhaVisivel) {
    foraDaPista = false;         
    return false;
  }

  if (!foraDaPista) {
    foraDaPista = true;
    inicioForaPista = millis();
  }

  return (millis() - inicioForaPista >= tempoMaxFora);
}

// Lê um sensor lateral e normaliza para 0-1000 usando a calibração dele
void lerSensorLateral(int sensorLateral) {
  int leitura = analogRead(sensorLateral);

  if (sensorLateral == sensorEsq) {
    valorEsq = map(leitura, minEsq, maxEsq, 0, 1000);
    valorEsq = constrain(valorEsq, 0, 1000);
  } else {
    valorDir = map(leitura, minDir, maxDir, 0, 1000);
    valorDir = constrain(valorDir, 0, 1000);
  }
}

// Posição da linha por média ponderada dos 6 sensores (0 a 5).
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

#include <Arduino.h> // Necessário para PlatformIO, mas não para Arduino IDE - biblioteca incluída automaticamente na IDE

/*
 * UC01786 - Automatização de uma Cascata com Aquecimento Solar
 * Entregável da escola: 100% Arduino IDE, sem rede (WiFi/MQTT fica na
 * versão de casa, em ../firmware_esp32_ha).
 *
 * Adaptado para Arduino Uno (20 pinos utilizáveis) em vez do Arduino Mega
 * do guia oficial de montagem - por isso os sinalizadores verde/amarelo/
 * vermelho e o alarme partilham o pino do vermelho (ver atualizarSaidas).
 */

// ===================== PINOS (Arduino Uno) =====================
// O Uno só tem 20 pinos utilizáveis (D2-D13 + A0-A5; D0/D1 reservados para
// o Serial). Usa-se aqui exatamente esses 18 pinos, um a um.

// Sensor de caudal (YF-S201 ou similar) - pulso, precisa de pino de interrupção
const int PIN_CAUDAL = 2; // INT0 no Uno (D2 ou D3 são os únicos com interrupção)

// Entradas de controlo
const int PIN_START = 3;
const int PIN_STOP = 4;
const int PIN_EMERGENCIA = 5;
const int PIN_SELETOR_AUTO = 6; // HIGH (aberto, pull-up) = automático, LOW = manual

// Sensores de nível - HC-SR04, pino dedicado por sensor
const int PIN_TRIG_NIVEL_SUP = 7;
const int PIN_ECHO_NIVEL_SUP = 8;
const int PIN_TRIG_NIVEL_INF = 9;
const int PIN_ECHO_NIVEL_INF = 10;

// Saídas - bombas
const int PIN_BOMBA_IMPULSAO = 11;
const int PIN_BOMBA_RETORNO = 12;
const int PIN_BOMBA_RECIRC = 13;

// Sensores analógicos
const int PIN_TEMP_ENTRADA = A0;
const int PIN_TEMP_SAIDA = A1;
const int PIN_CORRENTE = A2; // ACS712 ou similar

// Saídas - sinalização
const int PIN_SINAL_VERDE = A3;    // A3-A5 usados como digitais (não precisam ser analógicos)
const int PIN_SINAL_AMARELO = A4;
const int PIN_SINAL_VERMELHO = A5; // também aciona o alarme (ver atualizarSaidas) -
                                    // ligue o LED vermelho e o buzzer/relé de alarme
                                    // neste mesmo pino: o Uno não tem pino de sobra
                                    // para um alarme separado como no Mega do guia oficial

// ===================== SENSOR DE CAUDAL (por pulso/interrupção) =====================

volatile unsigned long contadorPulsos = 0;

void isrCaudal()
{
    contadorPulsos++;
}

// ===================== LIMIARES (ajustar após calibração) =====================

const int LIMIAR_NIVEL_MIN_CM = 40;     // acima disto (cm até a água) = falta de água
const int LIMIAR_NIVEL_MAX_CM = 5;      // abaixo disto (cm até a água) = transbordo
const float LIMIAR_CAUDAL_MIN_HZ = 2.0; // pulsos/seg abaixo disto = ausência de caudal
const int LIMIAR_CORRENTE_MAX = 800;    // acima disto = sobrecorrente
const int LIMIAR_CORRENTE_ARRANQUE = 80; // abaixo disto com a bomba ligada = bomba bloqueada
const unsigned long TIMEOUT_ARRANQUE_BOMBA_MS = 4000;
const float DELTA_TEMP_LIGA = 5.0;      // diferencial mínimo p/ recircular (°C)
const float DELTA_TEMP_DESLIGA = 1.0;
const int LEITURAS_INVALIDAS_PARA_AVARIA = 10; // ciclos consecutivos

// ===================== MÁQUINAS DE ESTADOS =====================
// Numeração conforme a Ficha Prática nº1.

enum EstadoCascata
{
    DESLIGADO = 0,
    VERIFICACAO = 1,
    IMPULSAO = 2,
    ENCHIMENTO = 3,
    RETORNO = 4,
    FUNCIONAMENTO = 5,
    PARAGEM = 6,
    ESVAZIAMENTO = 7,
    PARADO = 8,
    AVARIA_CASCATA = 9
};

enum EstadoSolar
{
    ESPERA = 0,
    VERIFICACAO_SOLAR = 1,
    RECIRCULACAO = 2,
    TEMPERATURA_MAXIMA = 3,
    AVARIA_SOLAR = 4
};

EstadoCascata estadoCascata = DESLIGADO;
EstadoSolar estadoSolar = ESPERA;

// Variáveis de sensores (atualizadas a cada volta do loop)
float nivelSup, nivelInf;             // distância sensor->água, em cm (menor = mais cheio)
int tempEntrada, tempSaida, corrente; // leitura analógica bruta
float caudal;                         // pulsos por segundo
bool falhaDetetada = false;
String descricaoFalha = "";
bool modoAutomatico = true;

// Contadores para deteção de "sensor avariado"
int leiturasInvalidasNivelSup = 0, leiturasInvalidasNivelInf = 0;
int leiturasInvalidasTemp = 0, leiturasInvalidasCorrente = 0;

// Deteção de "bomba bloqueada"
unsigned long tempoLigadaImpulsao = 0; // millis() de quando a bomba ligou (0 = desligada)

// Temporização não-bloqueante
unsigned long tempoAnterior = 0;
const unsigned long INTERVALO_LEITURA = 100; // ms

// ===================== PROTÓTIPOS =====================
// Necessário num ficheiro .cpp: ao contrário dos .ino, o PlatformIO
// não gera protótipos automaticamente, por isso loop() precisa deles
// declarados antes de serem definidos mais abaixo no ficheiro.

float lerDistanciaCm(int pinTrig, int pinEcho);
void lerSensores();
void verificarAvarias();
void verificarAvariaSolar();
void maquinaEstadosCascata();
void maquinaEstadosSolar();
void atualizarSaidas();
void registarDados();

// ===================== SETUP =====================

void setup()
{
    Serial.begin(9600);

    pinMode(PIN_START, INPUT_PULLUP);
    pinMode(PIN_STOP, INPUT_PULLUP);
    pinMode(PIN_EMERGENCIA, INPUT_PULLUP);
    pinMode(PIN_SELETOR_AUTO, INPUT_PULLUP);

    pinMode(PIN_TRIG_NIVEL_SUP, OUTPUT);
    pinMode(PIN_ECHO_NIVEL_SUP, INPUT);
    pinMode(PIN_TRIG_NIVEL_INF, OUTPUT);
    pinMode(PIN_ECHO_NIVEL_INF, INPUT);

    pinMode(PIN_CAUDAL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_CAUDAL), isrCaudal, FALLING);

    pinMode(PIN_BOMBA_IMPULSAO, OUTPUT);
    pinMode(PIN_BOMBA_RETORNO, OUTPUT);
    pinMode(PIN_BOMBA_RECIRC, OUTPUT);
    pinMode(PIN_SINAL_VERDE, OUTPUT);
    pinMode(PIN_SINAL_AMARELO, OUTPUT);
    pinMode(PIN_SINAL_VERMELHO, OUTPUT);

    Serial.println("Sistema Cascata UC01786 - inicializado");
}

// ===================== LOOP PRINCIPAL =====================

void loop()
{
    unsigned long agora = millis();
    if (agora - tempoAnterior >= INTERVALO_LEITURA)
    {
        tempoAnterior = agora;

        modoAutomatico = (digitalRead(PIN_SELETOR_AUTO) == HIGH);

        lerSensores();
        verificarAvarias();
        verificarAvariaSolar();
        maquinaEstadosCascata();
        maquinaEstadosSolar();
        atualizarSaidas();
        registarDados();
    }
}

// ===================== LEITURA DE SENSORES =====================

float lerDistanciaCm(int pinTrig, int pinEcho)
{
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    unsigned long duracao = pulseIn(pinEcho, HIGH, 30000); // timeout 30ms
    if (duracao == 0)
        return -1.0;                // sem eco = leitura inválida
    return duracao * 0.0343 / 2.0; // cm
}

void lerSensores()
{
    float distSup = lerDistanciaCm(PIN_TRIG_NIVEL_SUP, PIN_ECHO_NIVEL_SUP);
    float distInf = lerDistanciaCm(PIN_TRIG_NIVEL_INF, PIN_ECHO_NIVEL_INF);

    leiturasInvalidasNivelSup = (distSup < 0) ? (leiturasInvalidasNivelSup + 1) : 0;
    leiturasInvalidasNivelInf = (distInf < 0) ? (leiturasInvalidasNivelInf + 1) : 0;
    if (distSup >= 0)
        nivelSup = distSup;
    if (distInf >= 0)
        nivelInf = distInf;

    tempEntrada = analogRead(PIN_TEMP_ENTRADA);
    tempSaida = analogRead(PIN_TEMP_SAIDA);
    corrente = analogRead(PIN_CORRENTE);

    // Leitura fora da faixa fisicamente plausível (0 ou saturada) várias
    // vezes seguidas = sensor desligado/avariado.
    bool tempSuspeita = (tempEntrada <= 0 || tempEntrada >= 1023 || tempSaida <= 0 || tempSaida >= 1023);
    leiturasInvalidasTemp = tempSuspeita ? (leiturasInvalidasTemp + 1) : 0;

    bool correnteSuspeita = (corrente <= 0 || corrente >= 1023);
    leiturasInvalidasCorrente = correnteSuspeita ? (leiturasInvalidasCorrente + 1) : 0;

    // Frequência de pulso do sensor de caudal (Hz), com base no intervalo de leitura
    caudal = (contadorPulsos * 1000.0) / INTERVALO_LEITURA;
    contadorPulsos = 0;
}

// ===================== DETEÇÃO DE AVARIAS - CASCATA =====================
// Pode interromper qualquer estado e forçar AVARIA_CASCATA.
// Cobre as 7 avarias da ficha: falta de água, ausência de caudal, bomba
// bloqueada, sobrecorrente, sensor avariado, transbordo e falha do
// aquecimento solar.

void verificarAvarias()
{
    falhaDetetada = false;
    descricaoFalha = "";

    bool bombaImpulsaoLigada = (digitalRead(PIN_BOMBA_IMPULSAO) == HIGH);

    if (leiturasInvalidasNivelSup >= LEITURAS_INVALIDAS_PARA_AVARIA ||
        leiturasInvalidasNivelInf >= LEITURAS_INVALIDAS_PARA_AVARIA ||
        leiturasInvalidasTemp >= LEITURAS_INVALIDAS_PARA_AVARIA ||
        leiturasInvalidasCorrente >= LEITURAS_INVALIDAS_PARA_AVARIA)
    {
        falhaDetetada = true;
        descricaoFalha = "Sensor avariado";
    }
    else if (nivelSup > LIMIAR_NIVEL_MIN_CM)
    {
        falhaDetetada = true;
        descricaoFalha = "Falta de agua";
    }
    else if (nivelSup < LIMIAR_NIVEL_MAX_CM || nivelInf < LIMIAR_NIVEL_MAX_CM)
    {
        falhaDetetada = true;
        descricaoFalha = "Transbordo";
    }
    else if (caudal < LIMIAR_CAUDAL_MIN_HZ && estadoCascata == FUNCIONAMENTO)
    {
        falhaDetetada = true;
        descricaoFalha = "Ausencia de caudal";
    }
    else if (corrente > LIMIAR_CORRENTE_MAX)
    {
        falhaDetetada = true;
        descricaoFalha = "Sobrecorrente";
    }
    else if (bombaImpulsaoLigada &&
             tempoLigadaImpulsao > 0 &&
             (millis() - tempoLigadaImpulsao) > TIMEOUT_ARRANQUE_BOMBA_MS &&
             corrente < LIMIAR_CORRENTE_ARRANQUE)
    {
        falhaDetetada = true;
        descricaoFalha = "Bomba bloqueada";
    }

    if (estadoSolar == AVARIA_SOLAR)
    {
        // A ficha lista "falha do aquecimento solar" como avaria da cascata,
        // por isso propaga-se para a máquina de estados principal.
        falhaDetetada = true;
        descricaoFalha = "Falha do aquecimento solar";
    }

    if (falhaDetetada && estadoCascata != AVARIA_CASCATA)
    {
        estadoCascata = AVARIA_CASCATA;
    }
}

void verificarAvariaSolar()
{
    if (estadoSolar == RECIRCULACAO && tempSaida <= tempEntrada && (tempEntrada - tempSaida) < -50)
    {
        // Diferencial muito negativo e sustentado sugere sensor trocado ou
        // serpentina sem efeito nenhum - simplificado para o âmbito do Uno.
        estadoSolar = AVARIA_SOLAR;
    }
}

// ===================== MÁQUINA DE ESTADOS - CASCATA =====================

void maquinaEstadosCascata()
{
    bool start = (digitalRead(PIN_START) == LOW);
    bool stop = (digitalRead(PIN_STOP) == LOW);
    bool emergencia = (digitalRead(PIN_EMERGENCIA) == LOW);

    if (emergencia && estadoCascata != PARAGEM && estadoCascata != ESVAZIAMENTO && estadoCascata != PARADO)
    {
        estadoCascata = PARAGEM;
    }

    switch (estadoCascata)
    {
    case DESLIGADO:
        if (start)
        {
            // Em manual, liga-se logo o funcionamento, sem a sequência
            // temporizada de verificação/enchimento do modo automático.
            estadoCascata = modoAutomatico ? VERIFICACAO : FUNCIONAMENTO;
        }
        break;

    case VERIFICACAO:
        // Só avança com os sensores válidos (ver verificarAvarias).
        estadoCascata = ENCHIMENTO;
        break;

    case ENCHIMENTO:
        if (nivelSup <= LIMIAR_NIVEL_MAX_CM * 1.5)
            estadoCascata = IMPULSAO; // quase cheio
        break;

    case IMPULSAO:
        // bomba de impulsão liga (ver atualizarSaidas)
        estadoCascata = FUNCIONAMENTO;
        break;

    case FUNCIONAMENTO:
        if (stop)
        {
            estadoCascata = modoAutomatico ? PARAGEM : DESLIGADO;
        }
        else if (modoAutomatico && nivelSup < LIMIAR_NIVEL_MIN_CM * 0.6 && nivelInf > LIMIAR_NIVEL_MAX_CM * 2)
        {
            // Depósito superior a ficar baixo e reservatório inferior com
            // água disponível - coordena impulsão + retorno.
            estadoCascata = RETORNO;
        }
        break;

    case RETORNO:
        // bomba de impulsão continua ligada; bomba de retorno também liga
        // (ver atualizarSaidas) para repor o depósito superior.
        if (stop)
        {
            estadoCascata = PARAGEM;
        }
        else if (nivelSup >= LIMIAR_NIVEL_MAX_CM * 1.5)
        {
            estadoCascata = FUNCIONAMENTO;
        }
        break;

    case PARAGEM:
        estadoCascata = ESVAZIAMENTO;
        break;

    case ESVAZIAMENTO:
        estadoCascata = PARADO;
        break;

    case PARADO:
        if (start && !emergencia)
            estadoCascata = modoAutomatico ? VERIFICACAO : FUNCIONAMENTO;
        break;

    case AVARIA_CASCATA:
        // só sai daqui com reset manual (ex: novo START após resolver falha)
        if (start && !falhaDetetada)
            estadoCascata = DESLIGADO;
        break;
    }

    if (digitalRead(PIN_BOMBA_IMPULSAO) == LOW)
        tempoLigadaImpulsao = 0; // será marcado em atualizarSaidas() quando ligar
}

// ===================== MÁQUINA DE ESTADOS - SOLAR =====================

void maquinaEstadosSolar()
{
    float diferencial = (tempSaida - tempEntrada); // >0 = serpentina a aquecer a água

    switch (estadoSolar)
    {
    case ESPERA:
        estadoSolar = VERIFICACAO_SOLAR;
        break;

    case VERIFICACAO_SOLAR:
        if (diferencial >= DELTA_TEMP_LIGA)
        {
            estadoSolar = RECIRCULACAO;
        }
        else
        {
            estadoSolar = ESPERA;
        }
        break;

    case RECIRCULACAO:
        // bomba recirculadora liga (ver atualizarSaidas)
        if (diferencial < DELTA_TEMP_DESLIGA)
            estadoSolar = ESPERA;
        // TODO: definir limiar real de temperatura máxima da serpentina
        break;

    case TEMPERATURA_MAXIMA:
        estadoSolar = ESPERA;
        break;

    case AVARIA_SOLAR:
        // Sai da avaria solar quando a avaria geral da cascata for
        // reconhecida (START após resolver o problema).
        if (!falhaDetetada)
            estadoSolar = ESPERA;
        break;
    }
}

// ===================== ATUALIZAÇÃO DE SAÍDAS =====================

void atualizarSaidas()
{
    bool avaria = (estadoCascata == AVARIA_CASCATA);

    bool ligarImpulsao = !avaria && (estadoCascata == IMPULSAO || estadoCascata == FUNCIONAMENTO || estadoCascata == RETORNO);
    bool ligarRetorno = !avaria && (estadoCascata == RETORNO);
    bool ligarRecirc = !avaria && (estadoSolar == RECIRCULACAO);

    bool estavaLigadaImpulsao = (digitalRead(PIN_BOMBA_IMPULSAO) == HIGH);
    if (ligarImpulsao && !estavaLigadaImpulsao)
        tempoLigadaImpulsao = millis();
    else if (!ligarImpulsao)
        tempoLigadaImpulsao = 0;

    digitalWrite(PIN_BOMBA_IMPULSAO, ligarImpulsao);
    digitalWrite(PIN_BOMBA_RETORNO, ligarRetorno);
    digitalWrite(PIN_BOMBA_RECIRC, ligarRecirc);

    digitalWrite(PIN_SINAL_VERMELHO, avaria); // aciona LED vermelho + alarme no mesmo pino
    digitalWrite(PIN_SINAL_VERDE, (estadoCascata == FUNCIONAMENTO || estadoCascata == RETORNO) && !avaria);
    digitalWrite(PIN_SINAL_AMARELO, (estadoCascata == VERIFICACAO || estadoCascata == ENCHIMENTO) && !avaria);
}

// ===================== REGISTO DE DADOS =====================

void registarDados()
{
    Serial.print("EstadoCascata=");
    Serial.print(estadoCascata);
    Serial.print(" EstadoSolar=");
    Serial.print(estadoSolar);
    Serial.print(" Modo=");
    Serial.print(modoAutomatico ? "AUTO" : "MANUAL");
    Serial.print(" NivelSup=");
    Serial.print(nivelSup);
    Serial.print(" NivelInf=");
    Serial.print(nivelInf);
    Serial.print(" TempEnt=");
    Serial.print(tempEntrada);
    Serial.print(" TempSai=");
    Serial.print(tempSaida);
    Serial.print(" Caudal=");
    Serial.print(caudal);
    Serial.print(" Corrente=");
    Serial.print(corrente);
    if (falhaDetetada)
    {
        Serial.print(" FALHA=");
        Serial.print(descricaoFalha);
    }
    Serial.println();
}

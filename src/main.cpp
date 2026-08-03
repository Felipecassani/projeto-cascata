#include <Arduino.h> // Necessario para PlatformIO/CLion, nao para Wokwi/Arduino IDE

/*
 * UC01786 - Automatizacao de uma Cascata com Aquecimento Solar */

// ===================== PINOS (Arduino Mega 2560) =====================

// Sensor de caudal (YF-S201 ou similar) - pulso, precisa de pino de interrupcao
const uint8_t PIN_CAUDAL = 2; // INT0 no Mega (2, 3, 18, 19, 20, 21 tem interrupcao)

// Entradas de controlo
const uint8_t PIN_START = 3;
const uint8_t PIN_STOP = 4;
const uint8_t PIN_EMERGENCIA = 5;
const uint8_t PIN_SELETOR_AUTO = 6; // HIGH (aberto, pull-up) = automatico, LOW = manual

// Sensores de nivel - HC-SR04, pino dedicado por sensor
const uint8_t PIN_TRIG_NIVEL_SUP = 7;
const uint8_t PIN_ECHO_NIVEL_SUP = 8;
const uint8_t PIN_TRIG_NIVEL_INF = 9;
const uint8_t PIN_ECHO_NIVEL_INF = 10;

// Saidas - bombas (modulo rele: HIGH no pino IN liga o rele)
const uint8_t PIN_BOMBA_IMPULSAO = 11;
const uint8_t PIN_BOMBA_RETORNO = 12;
const uint8_t PIN_BOMBA_RECIRC = 13;

// Sensores analogicos
const uint8_t PIN_TEMP_ENTRADA = A0; // sensor de temperatura analogico (NTC), Vout
const uint8_t PIN_TEMP_SAIDA = A1;   // idem, saida da serpentina
const uint8_t PIN_CORRENTE = A2;     // potenciometro simulando ACS712 (sem parte real no Wokwi)

// Saidas - sinalizacao (LED RGB com os 3 canais, 1 cor por estado - ver atualizarSaidas)
const uint8_t PIN_SINAL_R = A3; // canal vermelho do LED RGB
const uint8_t PIN_SINAL_G = A4; // canal verde do LED RGB
const uint8_t PIN_SINAL_B = A7; // canal azul do LED RGB (NOVO 2026-08-01: mais cores por estado)
const uint8_t PIN_BUZZER = A5;  // buzzer com pino proprio (nao depende do LED)

// NOVO (2026-08-01): sensor de luz (LDR), desliga o sistema a noite
const uint8_t PIN_LDR = A6; // modulo fotoresistor, saida analogica (AO)

// ===================== SENSOR DE CAUDAL (por pulso, deteccao por polling) =====================
// Nota: no hardware real, com um sensor de fluxo de verdade (pulsos rapidos),
// usar attachInterrupt() seria o correto. Mas nesta simulacao o "sensor de
// caudal" e na verdade um botao manual (Wokwi/Tinkercad nao tem um sensor de
// fluxo real no catalogo) - so precisa detetar cliques ocasionais, entao
// polling a cada ciclo de 100ms e suficiente. Alem disso, attachInterrupt()
// nesse pino travou a simulacao no Wokwi (testado em 2026-08-01 - a
// simulacao parava em ~0.2s de tempo virtual e nunca avancava, mesmo depois
// de varios minutos reais) - polling evita esse problema por completo.

unsigned long contadorPulsos = 0;
bool estadoAnteriorCaudal = HIGH; // pino com pull-up: HIGH = solto/nao pressionado

// ===================== LIMIARES (ajustar apos calibracao) =====================

const int LIMIAR_NIVEL_MIN_CM = 40;     // acima disto (cm ate a agua) = falta de agua
const int LIMIAR_NIVEL_MAX_CM = 5;      // abaixo disto (cm ate a agua) = transbordo
const float LIMIAR_CAUDAL_MIN_HZ = 2.0; // pulsos/seg abaixo disto = ausencia de caudal
const int LIMIAR_CORRENTE_MAX = 800;    // acima disto = sobrecorrente
const int LIMIAR_CORRENTE_ARRANQUE = 80; // abaixo disto com a bomba ligada = bomba bloqueada
const unsigned long TIMEOUT_ARRANQUE_BOMBA_MS = 4000;
const float DELTA_TEMP_LIGA = 5.0;      // diferencial minimo p/ recircular (unidades ADC)
const float DELTA_TEMP_DESLIGA = 1.0;
const int LEITURAS_INVALIDAS_PARA_AVARIA = 10; // ciclos consecutivos
const int LIMIAR_LUZ_NOITE = 300; // ACIMA disto (0-1023) = escuro/noite.
                                   // wokwi-photoresistor-sensor tem resistor em serie com o LDR
                                   // formando divisor de tensao: no escuro a resistencia do LDR sobe,
                                   // entao o AO (ligado entre LDR e o resistor) SOBE tambem - confirmado
                                   // na doc oficial (docs.wokwi.com/parts/wokwi-photoresistor-sensor,
                                   // tabela de luminancia: 0.1 lux -> leitura 1016, 100000 lux -> leitura 8).

// Derivados dos limiares acima - dao nome aos "numeros magicos" que antes
// apareciam soltos (ex: "LIMIAR_NIVEL_MAX_CM * 1.5") nas transicoes de
// estado abaixo. Mesmo valor, mais facil de ler e de recalibrar depois.
const float NIVEL_QUASE_CHEIO_CM = LIMIAR_NIVEL_MAX_CM * 1.5;       // ENCHIMENTO -> IMPULSAO
const float NIVEL_MIN_PARA_RETORNO_CM = LIMIAR_NIVEL_MIN_CM * 0.6;  // gatilho do RETORNO: superior a ficar baixo
const float NIVEL_INF_DISPONIVEL_CM = LIMIAR_NIVEL_MAX_CM * 2;      // gatilho do RETORNO: inferior tem agua
const float CAIXA_SUP_LADO_CM = 70.0; // largura da caixa superior
const float CAIXA_SUP_ALTURA_CM = 60.0; // altura da caixa superior

const float CILINDRO_INF_ALTURA_CM = 35.0; // altura do cilindo inferior
const float CILINDRO_INF_DIAMETRO_CM = 45.0; // diametro do cilindro inferior
const float CILINDRO_INF_RAIO_CM = 22.5; // raio do cilindro

// ===================== MAQUINAS DE ESTADOS =====================
// Numeracao conforme a Ficha Pratica no1.

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

// Variaveis de sensores (atualizadas a cada volta do loop)
float litrosAguaSup, litrosAguainf;
float nivelSup, nivelInf;             // distancia sensor->agua, em cm (menor = mais cheio)
int tempEntrada, tempSaida, corrente; // leitura analogica bruta
int leituraLuz;                       // leitura analogica bruta do LDR (0-1023)
float caudal;                         // pulsos por segundo
bool falhaDetetada = false;
// const char* em vez de String: a classe String do Arduino aloca memoria
// dinamicamente a cada atribuicao, e reatribuir isto centenas de vezes por
// minuto (a cada volta do loop com falha ativa) fragmenta a RAM ate travar -
// risco real num programa que deve rodar continuamente. Como so atribuimos
// literais fixos, um ponteiro simples resolve sem esse custo.
const char *descricaoFalha = "";
bool modoAutomatico = true;
bool noite = false; // true quando leituraLuz indica escuro (ver LIMIAR_LUZ_NOITE)

// Contadores para deteccao de "sensor avariado"
int leiturasInvalidasNivelSup = 0, leiturasInvalidasNivelInf = 0;
int leiturasInvalidasTemp = 0, leiturasInvalidasCorrente = 0;

// Deteccao de "bomba bloqueada"
unsigned long tempoLigadaImpulsao = 0; // millis() de quando a bomba ligou (0 = desligada)

// Temporizacao nao-bloqueante
unsigned long tempoAnterior = 0;
const unsigned long INTERVALO_LEITURA = 100; // ms (alvo nominal - ver ultimoIntervaloMs)

// Duracao real do ultimo ciclo, em ms. Os dois lerDistanciaCm() em sequencia
// podem bloquear ate 30ms cada num timeout de sensor, entao o intervalo
// verdadeiro entre leituras as vezes passa bem de 100ms - usar esse valor
// real (em vez de assumir sempre INTERVALO_LEITURA) evita que o calculo do
// caudal fique impreciso quando isso acontece.
unsigned long ultimoIntervaloMs = INTERVALO_LEITURA;

// ===================== PROTOTIPOS =====================

float lerDistanciaCm(int pinTrig, int pinEcho);
void lerSensores();
void verificarAvarias();
void verificarAvariaSolar();
void maquinaEstadosCascata();
void maquinaEstadosSolar();
void atualizarSaidas();
void registarDados();
float litrosCaixaSup();
float litrosCilindroinf();
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

    pinMode(PIN_BOMBA_IMPULSAO, OUTPUT);
    pinMode(PIN_BOMBA_RETORNO, OUTPUT);
    pinMode(PIN_BOMBA_RECIRC, OUTPUT);
    pinMode(PIN_SINAL_R, OUTPUT);
    pinMode(PIN_SINAL_G, OUTPUT);
    pinMode(PIN_SINAL_B, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    Serial.println(F("Sistema Cascata UC01786 (Mega) - inicializado"));
}

// ===================== LOOP PRINCIPAL =====================

void loop()
{
    unsigned long agora = millis();
    if (agora - tempoAnterior >= INTERVALO_LEITURA)
    {
        ultimoIntervaloMs = agora - tempoAnterior;
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
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    unsigned long duracao = pulseIn(pinEcho, HIGH, 30000); // timeout 30ms (timeouts maiores travam a simulacao no Wokwi - testado 2026-08-01)
    if (duracao == 0)
        return -1.0;                // sem eco = leitura invalida
    return duracao / 58.0; // cm
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
    litrosAguaSup = litrosCaixaSup();
    litrosAguainf = litrosCilindroinf();
    tempEntrada = analogRead(PIN_TEMP_ENTRADA);
    tempSaida = analogRead(PIN_TEMP_SAIDA);
    corrente = analogRead(PIN_CORRENTE);
    leituraLuz = analogRead(PIN_LDR);
    noite = (leituraLuz > LIMIAR_LUZ_NOITE); // corrigido: era "<", invertido (ver comentario acima)

    // Leitura fora da faixa fisicamente plausivel (0 ou saturada) varias
    // vezes seguidas = sensor desligado/avariado.
    bool tempSuspeita = (tempEntrada <= 0 || tempEntrada >= 1023 || tempSaida <= 0 || tempSaida >= 1023);
    leiturasInvalidasTemp = tempSuspeita ? (leiturasInvalidasTemp + 1) : 0;

    bool correnteSuspeita = (corrente <= 0 || corrente >= 1023);
    leiturasInvalidasCorrente = correnteSuspeita ? (leiturasInvalidasCorrente + 1) : 0;

    // Deteta a borda de descida (solto -> pressionado) por polling - ver
    // comentario na declaracao de contadorPulsos, acima, sobre porque nao
    // usamos attachInterrupt() aqui.
    bool estadoAtualCaudal = digitalRead(PIN_CAUDAL);
    if (estadoAnteriorCaudal == HIGH && estadoAtualCaudal == LOW)
        contadorPulsos++;
    estadoAnteriorCaudal = estadoAtualCaudal;

    // Frequencia de pulso do sensor de caudal (Hz), com base na duracao real
    // do ciclo (ver comentario de ultimoIntervaloMs acima do loop()).
    caudal = (contadorPulsos * 1000.0) / ultimoIntervaloMs;
    contadorPulsos = 0;
}

float litrosCaixaSup()
{
    float alturaAguaSup = constrain(CAIXA_SUP_ALTURA_CM - nivelSup, 0, CAIXA_SUP_ALTURA_CM);
    float litrosAguaSup = (CAIXA_SUP_LADO_CM * CAIXA_SUP_LADO_CM) * alturaAguaSup / 1000.0;
    return litrosAguaSup;
} // números de litros de um cubo (l*p*A)/1000, subtrai a distancia do sensor ate o nivel d´agua e aplicando a formula terei a quantidade de litros.

float litrosCilindroinf()
{
    float alturaAguainf = constrain(CILINDRO_INF_ALTURA_CM - nivelInf, 0, CILINDRO_INF_ALTURA_CM);
    float litrosAguainf = M_PI * (CILINDRO_INF_RAIO_CM * CILINDRO_INF_RAIO_CM) * alturaAguainf / 1000.0;
    return litrosAguainf;
}  // números de litros de um cilindro (π*r²*a)/1000, subtrai a distancia do sensor ate o nivel d´agua e aplicando a formula terei a quantidade de litros.


// ===================== DETECAO DE AVARIAS - CASCATA =====================
// Pode interromper qualquer estado e forcar AVARIA_CASCATA.
// Cobre as 7 avarias da ficha: falta de agua, ausencia de caudal, bomba
// bloqueada, sobrecorrente, sensor avariado, transbordo e falha do
// aquecimento solar. (O desligamento noturno NAO e uma avaria - e tratado
// à parte em maquinaEstadosCascata(), como uma paragem programada normal.)

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
        // por isso propaga-se para a maquina de estados principal.
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
    // Deteta diferencial (saida - entrada) muito negativo e sustentado
    // durante recirculacao - sinal de sensores trocados ou serpentina sem
    // efeito nenhum.
    if (estadoSolar == RECIRCULACAO && (tempSaida - tempEntrada) < -50)
    {
        estadoSolar = AVARIA_SOLAR;
    }
}

// ===================== MAQUINA DE ESTADOS - CASCATA =====================

void maquinaEstadosCascata()
{
    bool start = (digitalRead(PIN_START) == LOW);
    bool stop = (digitalRead(PIN_STOP) == LOW);
    bool emergencia = (digitalRead(PIN_EMERGENCIA) == LOW);

    if (emergencia && estadoCascata != PARAGEM && estadoCascata != ESVAZIAMENTO && estadoCascata != PARADO)
    {
        estadoCascata = PARAGEM;
    }

    // Desligamento noturno (NOVO): mesma sequencia segura de paragem da
    // emergencia (PARAGEM -> ESVAZIAMENTO -> PARADO), mas nao e uma avaria -
    // e so nao reiniciar sozinho enquanto estiver escuro. Some ao amanhecer.
    if (noite && estadoCascata != PARAGEM && estadoCascata != ESVAZIAMENTO && estadoCascata != PARADO && estadoCascata != DESLIGADO)
    {
        estadoCascata = PARAGEM;
    }

    switch (estadoCascata)
    {
    case DESLIGADO:
        if (start && !noite)
        {
            // Em manual, liga-se logo o funcionamento, sem a sequencia
            // temporizada de verificacao/enchimento do modo automatico.
            estadoCascata = modoAutomatico ? VERIFICACAO : FUNCIONAMENTO;
        }
        break;

    case VERIFICACAO:
        // So avanca com os sensores validos (ver verificarAvarias).
        estadoCascata = ENCHIMENTO;
        break;

    case ENCHIMENTO:
        if (nivelSup <= NIVEL_QUASE_CHEIO_CM)
            estadoCascata = IMPULSAO; // quase cheio
        break;

    case IMPULSAO:
        // bomba de impulsao liga (ver atualizarSaidas)
        estadoCascata = FUNCIONAMENTO;
        break;

    case FUNCIONAMENTO:
        if (stop)
        {
            estadoCascata = modoAutomatico ? PARAGEM : DESLIGADO;
        }
        else if (modoAutomatico && nivelSup < NIVEL_MIN_PARA_RETORNO_CM && nivelInf > NIVEL_INF_DISPONIVEL_CM)
        {
            // Deposito superior a ficar baixo e reservatorio inferior com
            // agua disponivel - coordena impulsao + retorno.
            estadoCascata = RETORNO;
        }
        break;

    case RETORNO:
        // bomba de impulsao continua ligada; bomba de retorno tambem liga
        // (ver atualizarSaidas) para repor o deposito superior.
        if (stop)
        {
            estadoCascata = PARAGEM;
        }
        else if (nivelSup >= NIVEL_QUASE_CHEIO_CM)
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
        if (start && !emergencia && !noite)
            estadoCascata = modoAutomatico ? VERIFICACAO : FUNCIONAMENTO;
        break;

    case AVARIA_CASCATA:
        // so sai daqui com reset manual (ex: novo START apos resolver falha)
        if (start && !falhaDetetada)
            estadoCascata = DESLIGADO;
        break;
    }

    if (digitalRead(PIN_BOMBA_IMPULSAO) == LOW)
        tempoLigadaImpulsao = 0; // sera marcado em atualizarSaidas() quando ligar
}

// ===================== MAQUINA DE ESTADOS - SOLAR =====================

void maquinaEstadosSolar()
{
    float diferencial = (tempSaida - tempEntrada); // >0 = serpentina a aquecer a agua

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
        // TODO: definir limiar real de temperatura maxima da serpentina
        break;

    case TEMPERATURA_MAXIMA:
        estadoSolar = ESPERA;
        break;

    case AVARIA_SOLAR:
        // Sai da avaria solar quando a avaria geral da cascata for
        // reconhecida (START apos resolver o problema).
        if (!falhaDetetada)
            estadoSolar = ESPERA;
        break;
    }
}

// ===================== ATUALIZACAO DE SAIDAS =====================

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

    // LED RGB - 1 cor por grupo de estados da maquina de estados da cascata
    // 3 cores pra distinguir mais coisas, nao so
    // avaria/funcionamento/verificacao). Estados nunca se sobrepoem (so um
    // "estadoCascata" ativo por vez), entao os grupos abaixo sao mutuamente
    // exclusivos - cada estado sempre acende exatamente 1 cor:
    //   DESLIGADO(0)                    -> apagado
    //   VERIFICACAO(1)/ENCHIMENTO(3)    -> amarelo   (R+G) - preparando
    //   IMPULSAO(2)/FUNCIONAMENTO(5)    -> verde     (G)   - operando
    //   RETORNO(4)                      -> ciano     (G+B) - impulsao+retorno juntos
    //   PARAGEM(6)/ESVAZIAMENTO(7)      -> azul      (B)   - parando em seguranca
    //   PARADO(8)                       -> magenta   (R+B) - parado, pronto pra reiniciar
    //   AVARIA_CASCATA(9)               -> vermelho  (R)   - falha, precisa reset manual
    bool preparando = (estadoCascata == VERIFICACAO || estadoCascata == ENCHIMENTO);
    bool operando = (estadoCascata == IMPULSAO || estadoCascata == FUNCIONAMENTO);
    bool retornando = (estadoCascata == RETORNO);
    bool parandoSeguro = (estadoCascata == PARAGEM || estadoCascata == ESVAZIAMENTO);
    bool parado = (estadoCascata == PARADO);

    digitalWrite(PIN_SINAL_R, avaria || preparando || parado);
    digitalWrite(PIN_SINAL_G, operando || retornando || preparando);
    digitalWrite(PIN_SINAL_B, retornando || parandoSeguro || parado);
    digitalWrite(PIN_BUZZER, avaria);
}

// ===================== REGISTO DE DADOS =====================

void registarDados()
{
    // F() mantem estes literais na flash em vez de copiar para a RAM.
    Serial.print(F("EstadoCascata="));
    Serial.print(estadoCascata);
    Serial.print(F(" EstadoSolar="));
    Serial.print(estadoSolar);
    Serial.print(F(" Modo="));
    Serial.print(modoAutomatico ? F("AUTO") : F("MANUAL"));
    Serial.print(F(" NivelSup="));
    Serial.print(nivelSup);
    Serial.print(F(" NivelInf="));
    Serial.print(nivelInf);
    Serial.print(F(" TempEnt="));
    Serial.print(tempEntrada);
    Serial.print(F(" TempSai="));
    Serial.print(tempSaida);
    Serial.print(F(" Caudal="));
    Serial.print(caudal);
    Serial.print(F(" Corrente="));
    Serial.print(corrente);
    Serial.print(F(" Luz="));
    Serial.print(leituraLuz);
    Serial.print(F(" Noite="));
    Serial.print(noite ? F("SIM") : F("NAO"));
    if (falhaDetetada)
    {
        Serial.print(F(" FALHA="));
        Serial.print(descricaoFalha);
    }
    Serial.println();
}




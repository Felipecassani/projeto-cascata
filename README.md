# Automação de Cascata com Aquecimento Solar — Arduino Uno

Entregável escolar (UC01786): automação de uma cascata de água com
recirculação de nível e um circuito solar de aquecimento em paralelo —
duas máquinas de estado a correr no mesmo Arduino Uno, sem rede
(a versão com WiFi/MQTT + Home Assistant está em
[firmware_esp32_ha](https://github.com/Felipecassani/firmware_esp32_ha)).

## O que o sistema faz
- **Máquina de estados da cascata**: liga/desliga bombas de impulsão e
  retorno conforme o nível de água em dois depósitos (sensores
  ultrassónicos HC-SR04), com modos automático e manual.
- **Máquina de estados solar**: liga a bomba de recirculação quando a
  água aquecida na serpentina está suficientemente mais quente que a
  água de entrada (sensores de temperatura analógicos).
- **Deteção de 7 tipos de avaria**: falta de água, transbordo, ausência
  de caudal, sobrecorrente, bomba bloqueada, sensor avariado e falha do
  aquecimento solar — qualquer uma força o sistema para o estado de
  avaria com sinalização (LED vermelho + alarme).

## Hardware
- Arduino Uno (adaptado do guia oficial, pensado para Arduino Mega —
  ver comentário no início do código sobre a partilha de pino do alarme)
- Sensor de caudal por pulso (interrupção)
- 2x sensor ultrassónico HC-SR04 (nível superior/inferior)
- Sensor de corrente (tipo ACS712)
- 2x sensor de temperatura analógico
- 3 bombas, sinalização por LED (verde/amarelo/vermelho) + alarme

## Leituras não-bloqueantes
Toda a lógica corre em ciclos de 100ms usando `millis()`, sem `delay()`
bloqueante — leitura de sensores, deteção de avarias e as duas máquinas
de estado correm no mesmo `loop()`.

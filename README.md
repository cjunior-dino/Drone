# Drone com ESP32

Projeto experimental para desenvolver, por etapas, um quadricóptero controlado por um ESP32-WROOM-32D. A fase atual é exclusivamente de bancada: leitura da unidade inercial, validação das conexões e preparação da base do firmware. O sistema ainda não está pronto para voar e nenhum ESC ou motor é acionado pelo código atual.

## Objetivo inicial

Construir uma controladora de voo didática baseada em ESP32, evoluindo de testes isolados até um sistema com:

- leitura e calibração da IMU;
- estimativa de inclinação;
- recepção dos comandos do rádio;
- controle independente de quatro ESCs;
- estabilização por PID;
- armamento seguro e failsafe;
- testes controlados antes de qualquer tentativa de voo.

## Estado atual

O firmware disponível realiza um teste de bancada do GY-521/MPU6050:

- procura o sensor nos endereços I²C `0x68` e `0x69`;
- valida o registrador `WHO_AM_I`;
- configura o acelerômetro em ±4 g;
- configura o giroscópio em ±500 °/s;
- calibra o desvio do giroscópio usando 1.000 amostras;
- lê a IMU a 200 Hz;
- estima `roll` e `pitch` com filtro complementar;
- envia os dados pelo monitor serial a 10 Hz;
- interrompe o teste após falhas consecutivas de comunicação.

Nesta etapa não há saída para ESCs ou motores.

## Materiais identificados

- placa de desenvolvimento ESP32-WROOM-32D;
- módulo GY-521 com MPU6050;
- rádio FlySky FS-i6;
- receptor FlySky FS-iA6 de seis canais;
- quatro motores A2212/6T 2200 KV;
- quatro ESCs do kit, com modelo e especificações ainda a confirmar;
- estrutura de quadricóptero em configuração X;
- hélices de duas pás;
- bateria LiPo 3S 2200 mAh, 11,1 V e 30C;
- placa de distribuição de energia CRIUS V1.0;
- conversor DC-DC ajustável LM2596;
- módulo NRF24L01+ com antena externa;
- conversor de nível lógico de quatro canais;
- carregador balanceador, multímetro, ferro de solda, estanho, termo-retrátil e conectores.

O NRF24L01+ não participa do teste atual. O controle principal previsto é o conjunto FlySky; o uso do NRF24L01+ poderá ser avaliado posteriormente para telemetria ou experimentos separados.

## Ligação atual do MPU6050

| GY-521 | ESP32 |
| --- | --- |
| `VCC` | `3V3` |
| `GND` | `GND` |
| `SDA` | `GPIO21` |
| `SCL` | `GPIO22` |

O firmware aceita o sensor em `0x68` ou `0x69`. O conversor de nível lógico não é necessário nessa ligação porque o ESP32 e a comunicação I²C do teste trabalham em 3,3 V.

## Estrutura do projeto

```text
Drone/
├── assets/                    # Fotos e documentos de referência
├── firmware/
│   └── fm/
│       └── src/
│           └── main.cpp      # Teste de bancada do MPU6050
├── platformio.ini            # Configuração do ESP32 e do PlatformIO
└── README.md
```

O `platformio.ini` permanece na raiz para que toda a pasta `Drone` possa ser aberta no VS Code. O código-fonte usado na compilação é definido por:

```ini
[platformio]
src_dir = firmware/fm/src
```

## Preparação do ambiente

1. Instale o Visual Studio Code.
2. Instale a extensão **PlatformIO IDE**.
3. Abra a pasta raiz `C:\Users\JUNIOR\Documents\Drone` no VS Code.
4. Aguarde o PlatformIO terminar a indexação do projeto.

O ambiente configurado é:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

## Compilar

No VS Code, abra o painel do PlatformIO e execute:

```text
PROJECT TASKS → esp32dev → General → Build
```

Alternativamente, no terminal do PlatformIO:

```powershell
pio run
```

A compilação validada utiliza aproximadamente 6,7% da RAM e 22,2% da área de programa disponível para o ambiente configurado.

## Teste de bancada

1. Mantenha motores, ESCs e bateria de potência desconectados.
2. Conecte somente o ESP32 e o GY-521 conforme a tabela de ligação.
3. Grave o firmware no ESP32.
4. Abra o monitor serial em `115200` baud.
5. Pressione `EN/RESET` se as mensagens iniciais não aparecerem.
6. Durante a calibração, mantenha o sensor parado e nivelado por aproximadamente dois segundos.
7. Movimente o sensor lentamente e observe `roll`, `pitch`, aceleração, velocidade angular e temperatura.

## Segurança

- Todos os testes iniciais devem ser feitos sem hélices.
- Não alimente o ESP32 diretamente com a bateria LiPo 3S.
- Ajuste e confirme a saída do LM2596 com um multímetro antes de conectá-la a qualquer componente.
- Verifique polaridade e continuidade antes de energizar a placa de distribuição.
- Não conecte os ESCs enquanto o firmware não possuir armamento, limite de aceleração e failsafe testados.
- Carregue a LiPo somente com carregador balanceador compatível e em local seguro.

## Próximas etapas

1. Validar continuamente a leitura e a orientação física do MPU6050.
2. Separar o firmware em módulos de IMU, receptor, motores, PID e segurança.
3. Identificar e testar o sinal produzido pelo receptor FlySky FS-iA6.
4. Implementar leitura dos canais do rádio sem ligar os ESCs.
5. Gerar e medir os sinais destinados aos ESCs, ainda sem motores.
6. Testar um único ESC e motor sem hélice, com limitação e parada de emergência.
7. Implementar armamento, failsafe e controle PID.
8. Validar o conjunto em uma bancada de testes antes de instalar hélices.

## Referências de arquitetura

Os projetos [Betaflight](https://github.com/betaflight/betaflight) e [Cleanflight](https://github.com/cleanflight/cleanflight) são referências para organização de controladoras de voo, processamento de sensores, controle PID e mecanismos de segurança. Este firmware é uma implementação experimental própria para ESP32 e não é compatível com esses projetos neste estágio.


# Drone experimental com ESP32

Projeto de estudo para desenvolver, por etapas, um controlador de voo baseado no ESP32-WROOM-32D. O estágio atual é exclusivamente de bancada: leitura da unidade inercial, recepção dos comandos do rádio e validação das rotinas de segurança.

O firmware possui uma máquina de estados de armamento apenas simulada e não produz sinais para ESCs ou motores. Mesmo no estado `SIM_ARMED`, `outputs_enabled` permanece em `0`.

## Segurança

- Mantenha bateria LiPo, ESCs, motores e hélices desconectados durante os testes atuais.
- Nunca conecte a bateria LiPo diretamente ao ESP32, ao receptor ou ao GY-521.
- Use o LM2596 somente depois de ajustar e confirmar sua saída com um multímetro.
- O armamento atual é somente uma simulação lógica; nenhuma saída física é habilitada.
- Este protótipo ainda não está pronto para voo.

## Objetivo inicial

Construir uma base confiável para:

1. ler e calibrar o MPU6050;
2. receber os quatro canais principais do FlySky FS-i6;
3. detectar perda ou corrupção do sinal do receptor;
4. converter os comandos do rádio para valores normalizados;
5. preparar, em etapas futuras, mixer, controle PID e saídas para os ESCs.

## Etapa validada em bancada

O firmware atual já executa:

- detecção do MPU6050 nos endereços I2C `0x68` ou `0x69`;
- validação do registrador `WHO_AM_I`;
- acelerômetro configurado para ±4 g;
- giroscópio configurado para ±500 graus/s;
- calibração inicial do giroscópio com 1.000 amostras;
- leitura da IMU a 200 Hz;
- estimativa de roll e pitch por filtro complementar;
- leitura simultânea dos canais CH1, CH2, CH3 e CH4 por interrupções;
- rejeição de pulsos fora da faixa de 900 a 2.100 microssegundos;
- confirmação de mudanças grandes em dois quadros consecutivos;
- failsafe após 100 ms sem pulso válido;
- comandos normalizados de roll, pitch, throttle e yaw;
- telemetria no monitor serial a 10 Hz;
- leitura da IMU, do receptor e normalização dos comandos separadas em módulos;
- módulo de segurança com armamento e desarmamento simulados, temporizadores e bloqueio após failsafe;
- contadores cumulativos de pulsos rejeitados, failsafe e recuperação do receptor;
- `outputs_enabled` permanentemente em `0`, sem saída para ESC ou motor.

## Materiais usados nesta etapa

- ESP32-WROOM-32D em placa de desenvolvimento;
- módulo GY-521 com MPU6050;
- rádio FlySky FS-i6;
- receptor FlySky FS-iA6;
- conversor de nível lógico BSS138 de quatro canais;
- protoboard e cabos de conexão;
- cabo USB para alimentação e programação.

Também estão disponíveis para etapas futuras quatro motores A2212/6T 2200KV, ESCs, placa de distribuição, bateria LiPo 3S 2200 mAh e conversor LM2596. Esses componentes não fazem parte do teste atual.

## Ligações do GY-521

| GY-521 | ESP32 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

O GY-521 está montado verticalmente nesta fase. Por isso, os ângulos exibidos servem para confirmar resposta e direção do sensor, mas ainda não representam a orientação de voo. Antes de implementar ou ajustar PID, o módulo deverá ser fixado horizontalmente e os eixos deverão ser validados novamente.

## Ligações do receptor e conversor de nível

O lado `HV` do conversor usa 5 V e recebe os sinais do receptor. O lado `LV` usa 3,3 V e envia os sinais ao ESP32. Todos os módulos precisam compartilhar o mesmo GND.

| Receptor FS-iA6 | Conversor | ESP32 |
| --- | --- | --- |
| CH1, pino S | HV1 → LV1 | GPIO27 |
| CH2, pino S | HV2 → LV2 | GPIO26 |
| CH3, pino S | HV3 → LV3 | GPIO25 |
| CH4, pino S | HV4 → LV4 | GPIO33 |
| 5V/VCC | HV | — |
| GND | GND comum | GND |
| — | LV | 3V3 |

O BSS138 funcionou nos testes de bancada. Para a instalação final, seu uso deverá ser reavaliado considerando integridade dos pulsos, comprimento dos fios e imunidade a ruído.

## Mapeamento do rádio

O FlySky FS-i6 está configurado em Mode 1. O mapeamento observado é:

| Canal | Comando | Controle físico |
| --- | --- | --- |
| CH1 | Roll | manche direito, horizontal |
| CH2 | Pitch | manche esquerdo, vertical |
| CH3 | Throttle | manche direito, vertical |
| CH4 | Yaw | manche esquerdo, horizontal |

Esse mapeamento é consequência do modo configurado no transmissor. Não se deve trocar CH2 e CH3 apenas para imitar Mode 2; qualquer mudança de modo deve ser feita conscientemente no rádio e depois validada novamente no firmware.

## Tratamento dos sinais do receptor

Cada canal é medido em microssegundos. O firmware:

- aceita somente pulsos entre 900 e 2.100 µs;
- ignora pulsos isolados fora dessa faixa;
- exige duas leituras consecutivas para confirmar uma variação grande;
- considera o receptor indisponível depois de 100 ms sem sinal válido;
- zera todos os comandos normalizados durante o failsafe.

Os campos normalizados são:

| Campo | Faixa |
| --- | --- |
| `roll_cmd` | -1,0 a +1,0 |
| `pitch_cmd` | -1,0 a +1,0 |
| `throttle_cmd` | 0,0 a 1,0 |
| `yaw_cmd` | -1,0 a +1,0 |

Roll, pitch e yaw possuem uma pequena zona morta ao redor do centro.

## Máquina de estados simulada

A lógica de armamento existe somente para testar as condições de segurança. Os estados são:

- `DISARMED`: estado inicial e seguro;
- `ARMING`: gesto de armamento sendo mantido;
- `SIM_ARMED`: armamento lógico confirmado, ainda sem saídas físicas;
- `DISARMING`: gesto de desarmamento sendo mantido;
- `FAILSAFE_LOCKOUT`: bloqueio após perda do sinal do receptor.

Para entrar em `SIM_ARMED`, o receptor deve estar válido, o throttle deve estar abaixo de 5%, roll e pitch devem estar centralizados e o yaw deve permanecer totalmente à direita por 1,5 segundo. Para desarmar, o throttle deve estar baixo e o yaw totalmente à esquerda por 1 segundo.

Uma perda de sinal depois da aquisição válida leva imediatamente a `FAILSAFE_LOCKOUT`. Depois que o sinal retorna, throttle, roll, pitch e yaw devem permanecer em posição segura por 1 segundo para o sistema voltar a `DISARMED`. Não existe rearmamento automático.

O campo `state_ms` da telemetria informa há quanto tempo o estado atual está ativo. O campo `outputs_enabled` permanece sempre em `0` nesta fase.

### Diagnósticos do receptor

A telemetria também informa:

- `rx_reject_ch1` a `rx_reject_ch4`: pulsos descartados por estarem fora da faixa de 900 a 2.100 µs;
- `rx_valid_ch1` a `rx_valid_ch4`: pulsos completos e válidos recebidos em cada canal;
- `rx_short_ch1` a `rx_short_ch4`: pulsos completos abaixo de 900 µs;
- `rx_long_ch1` a `rx_long_ch4`: pulsos completos acima de 2.100 µs;
- `rx_orphan_ch1` a `rx_orphan_ch4`: bordas de descida detectadas sem uma borda de subida pendente;
- `rx_failsafe_events`: perdas de sinal ocorridas depois que o receptor já havia ficado válido;
- `rx_recovery_events`: recuperações posteriores a uma perda de sinal.

Iniciar o ESP32 com o transmissor desligado não aumenta `rx_failsafe_events`. O contador cresce somente na transição de sinal válido para failsafe.

Uma borda órfã não é interpretada como pulso nem altera os comandos. Esse diagnóstico permite separar transições incompletas de pulsos realmente fora da faixa aceita.

## Resultados dos testes

- O failsafe foi acionado corretamente ao desligar o transmissor: `rx_ok:0` e comandos zerados.
- A recepção voltou normalmente depois que o transmissor foi religado.
- Os quatro comandos alcançaram aproximadamente suas faixas normalizadas completas.
- Depois do ajuste que ignora pulsos fora da faixa já na interrupção, um teste com cinco movimentos completos de yaw registrou 323 amostras válidas e nenhuma amostra inválida.
- Depois da modularização do receptor, um novo teste de roll registrou 223 amostras válidas, CH1 de 988 a 1.997 µs, `roll_cmd` de -1,000 a +0,994 e retorno ao centro em 1.494 µs.
- O módulo de segurança foi validado com `state:DISARMED`, `outputs_enabled:0`, failsafe ativo e todos os canais e comandos zerados enquanto o transmissor estava desligado.
- A sequência completa de aquisição, perda e recuperação do rádio foi validada: a primeira aquisição manteve `rx_failsafe_events:0` e `rx_recovery_events:0`; ao desligar o transmissor, os valores passaram para `rx_ok:0`, `rx_failsafe_events:1` e comandos zerados; depois de religá-lo, foram obtidos `rx_ok:1`, `rx_failsafe_events:1` e `rx_recovery_events:1`.
- Durante essa sequência, o estado permaneceu `DISARMED` e `outputs_enabled:0` em todas as fases.
- O diagnóstico detalhado registrou 13.008 pulsos válidos por canal com os manches parados, nenhum pulso curto ou longo e algumas bordas de descida incompletas, que foram descartadas sem afetar os comandos.
- Depois de movimentar todos os manches até os extremos, os quatro canais continuaram sem pulsos curtos ou longos, sem failsafe e com retorno correto à posição segura.
- Ao desligar o transmissor no teste detalhado, o firmware zerou novamente os canais, registrou exatamente um evento de failsafe e permaneceu sem pulsos curtos ou longos.
- As bordas incompletas aparecem mesmo com os controles parados. Antes de habilitar saídas reais, o caminho de sinal com o BSS138 deverá ser validado com instrumentação adequada ou substituído por uma interface unidirecional apropriada.
- Ligar o transmissor com os controles em posição segura manteve o estado `DISARMED`.
- O gesto completo de yaw à direita com throttle baixo levou corretamente a `SIM_ARMED`, sempre com `outputs_enabled:0`.
- Um gesto de armamento com duração inferior a 1,5 segundo foi cancelado e manteve `DISARMED`.
- O armamento foi bloqueado com throttle em 15,3%, acima do limite configurado de 5%.
- O gesto de yaw à esquerda com throttle baixo retornou corretamente de `SIM_ARMED` para `DISARMED`.
- Ao desligar o transmissor durante `SIM_ARMED`, o estado mudou para `FAILSAFE_LOCKOUT`, os comandos foram zerados e as saídas permaneceram bloqueadas.
- Após recuperar o sinal com os controles em posição segura, o estado voltou somente para `DISARMED`, sem rearmamento automático.

Exemplo da saída serial:

```text
state:DISARMED,state_ms:27443,outputs_enabled:0,rx_ok:1,ch1_us:1493,ch2_us:1495,ch3_us:1000,ch4_us:1496,roll_cmd:0.000,pitch_cmd:0.000,throttle_cmd:0.000,yaw_cmd:0.000,rx_reject_ch1:0,rx_reject_ch2:0,rx_reject_ch3:0,rx_reject_ch4:0,rx_failsafe_events:1,rx_recovery_events:1
```

## Estrutura do projeto

```text
Drone/
├── firmware/
│   └── fm/
│       └── src/
│           ├── control_commands.cpp
│           ├── control_commands.h
│           ├── imu.cpp
│           ├── imu.h
│           ├── main.cpp
│           ├── receiver.cpp
│           ├── receiver.h
│           ├── safety.cpp
│           └── safety.h
├── assets/
├── platformio.ini
├── .gitignore
└── README.md
```

A pasta `sw/` contém projetos externos usados apenas como referência e não é versionada neste repositório.

## Ambiente de desenvolvimento

O projeto usa VS Code com PlatformIO. O arquivo `platformio.ini` na raiz define `firmware/fm/src` como diretório do código-fonte e utiliza a placa `esp32dev` com o framework Arduino.

Se o comando `pio` estiver disponível no terminal:

```powershell
pio run
pio run --target upload --upload-port COM4
pio device monitor --port COM4 --baud 115200
```

Se o PowerShell não encontrar `pio`, use o executável instalado no perfil do usuário:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM4
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM4 --baud 115200
```

Somente um programa pode usar a porta COM4 por vez. Feche o monitor serial com `Ctrl+C` antes de gravar um novo firmware.

## Procedimento atual de bancada

1. Confirme que bateria, ESCs, motores e hélices estão desconectados.
2. Confira todas as ligações com o USB desconectado.
3. Ligue o transmissor e deixe o throttle na posição mínima.
4. Conecte o ESP32 ao USB.
5. Abra o monitor serial em 115200 baud.
6. Confirme `state:DISARMED` e `rx_ok:1`.
7. Movimente um controle por vez e confira o campo correspondente.
8. Ao terminar, feche o monitor e desconecte o USB.

## Limitações atuais

- não há definição de pinos de ESC;
- não há geração de PWM ou DShot;
- não há armamento real; a máquina de estados atual é apenas uma simulação sem saídas;
- não há mixer de quadricóptero;
- não há controle PID;
- o GY-521 ainda está montado verticalmente;
- o modelo e as especificações exatas dos ESCs ainda precisam ser confirmados;
- a alimentação pela LiPo e pelo LM2596 ainda não foi integrada ao teste.

## Próximas etapas planejadas

1. validar com instrumentação adequada ou substituir a interface BSS138 antes de habilitar saídas reais;
2. corrigir a montagem do GY-521 e validar a orientação dos eixos;
3. implementar mixer e PID inicialmente apenas como valores de diagnóstico;
4. identificar as especificações dos ESCs;
5. somente depois das validações anteriores, testar um ESC e um motor sem hélice, com desligamento de emergência preparado.

## Projetos de referência

Betaflight e Cleanflight são usados como referências conceituais para organização de sensores, recepção de rádio, filtros, failsafe, mixer e controle. Este firmware é uma implementação experimental própria e não declara compatibilidade com esses projetos.

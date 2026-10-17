# Drone experimental com ESP32

Projeto de estudo para desenvolver, por etapas, um controlador de voo baseado no ESP32-WROOM-32D. O estágio atual é exclusivamente de bancada: leitura da unidade inercial, recepção dos comandos do rádio e validação das rotinas de segurança.

O firmware permanece sempre no estado `DISARMED` e não produz sinais para ESCs ou motores.

## Segurança

- Mantenha bateria LiPo, ESCs, motores e hélices desconectados durante os testes atuais.
- Nunca conecte a bateria LiPo diretamente ao ESP32, ao receptor ou ao GY-521.
- Use o LM2596 somente depois de ajustar e confirmar sua saída com um multímetro.
- O estado `DISARMED` atual é fixo; ainda não existe rotina de armamento.
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
- estado permanente `DISARMED`, sem saída para ESC ou motor.

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

## Resultados dos testes

- O failsafe foi acionado corretamente ao desligar o transmissor: `rx_ok:0` e comandos zerados.
- A recepção voltou normalmente depois que o transmissor foi religado.
- Os quatro comandos alcançaram aproximadamente suas faixas normalizadas completas.
- Depois do ajuste que ignora pulsos fora da faixa já na interrupção, um teste com cinco movimentos completos de yaw registrou 323 amostras válidas e nenhuma amostra inválida.

Exemplo da saída serial:

```text
state:DISARMED,rx_ok:1,ch1_us:1493,ch2_us:1495,ch3_us:1000,ch4_us:1496,roll_cmd:0.000,pitch_cmd:0.000,throttle_cmd:0.000,yaw_cmd:0.000
```

## Estrutura do projeto

```text
Drone/
├── firmware/
│   └── fm/
│       └── src/
│           └── main.cpp
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
- não há rotina de armamento;
- não há mixer de quadricóptero;
- não há controle PID;
- o GY-521 ainda está montado verticalmente;
- o modelo e as especificações exatas dos ESCs ainda precisam ser confirmados;
- a alimentação pela LiPo e pelo LM2596 ainda não foi integrada ao teste.

## Próximas etapas planejadas

1. separar o firmware em módulos de IMU, receptor, comandos e segurança;
2. adicionar diagnósticos cumulativos de pulsos rejeitados e eventos de failsafe;
3. criar uma máquina de estados de armamento apenas simulada, sem saídas físicas;
4. corrigir a montagem do GY-521 e validar a orientação dos eixos;
5. implementar mixer e PID inicialmente apenas como valores de diagnóstico;
6. identificar as especificações dos ESCs;
7. somente depois das validações anteriores, testar um ESC e um motor sem hélice, com desligamento de emergência preparado.

## Projetos de referência

Betaflight e Cleanflight são usados como referências conceituais para organização de sensores, recepção de rádio, filtros, failsafe, mixer e controle. Este firmware é uma implementação experimental própria e não declara compatibilidade com esses projetos.

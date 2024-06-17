# Sistema de monitoramento de um biodigestor com ESP32 e Sensores:

## Proposta:

O projeto visa facilitar a operação e o monitoramento de sistemas de biodigestão, fornecendo dados precisos em tempo real sobre as condições do ambiente. Isso é crucial para o desempenho eficiente do processo de biodigestão, permitindo ajustes proativos e garantindo um funcionamento seguro e otimizado do sistema. O monitoramento contínuo das condições ambientais possibilitado pelos sensores contribui para a maximização da produção de biogás e para a sustentabilidade de sistemas de tratamento de resíduos orgânicos.

## Arquitetura do Projeto:

O projeto segue uma arquitetura onde o ESP32 coleta dados dos sensores e os envia para a plataforma Adafruit via protocolo MQTT. A plataforma Adafruit recebe os dados e os exibe em um dashboard online, permitindo o monitoramento remoto do sistema de biodigestão.

![Arquitetura do Projeto](./arch.jpg)

## Hardware:

### Visão Geral

O projeto do sistema de monitoramento de um biodigestor com ESP32 e sensores é uma solução para monitorar e otimizar o processo de biodigestão, fornecendo dados precisos em tempo real sobre as condições ambientais. Isso é fundamental para garantir um funcionamento eficiente e seguro do sistema, maximizando a produção de biogás e contribuindo para a sustentabilidade de sistemas de tratamento de resíduos orgânicos.

### Estruturação Modular do Projeto:

O projeto do biodigestor foi estruturado de forma modular, segmentando o hardware em diferentes funções para facilitar o desenvolvimento, manutenção e possíveis iterações futuras. Cada componente desempenha uma função específica no monitoramento das condições do ambiente durante o processo de biodigestão.

### Componentes Utilizados:

- ESP32 (microcontrolador)
- [DHT11 (sensor de temperatura e umidade)](https://github.com/JulioAmaral007/Biodigestor/tree/main/Sensores/Sensor-DHT11)
- [BMP280 (sensor de pressão atmosférica)](https://github.com/JulioAmaral007/Biodigestor/tree/main/Sensores/Sensor-BMP280)
- [MQ-2 (sensor de gás)](https://github.com/JulioAmaral007/Biodigestor/tree/main/Sensores/Sensor-MQ2)

No projeto de biodigestor, diferentes sensores desempenham funções específicas para monitorar as condições ambientais e garantir a eficiência e segurança do sistema. Aqui está como cada sensor funcionará:

### Integração com a Plataforma Adafruit IO:

Todos os dados coletados pelos sensores (MQ-2, DHT11 e BMP280) serão enviados para a plataforma Adafruit IO por meio de um ESP32. Isso permite que os dados sejam visualizados e analisados em tempo real pelos operadores, proporcionando um monitoramento abrangente do sistema de biodigestão.

No link a seguir estará o passo-a-passo da integração com o [adafruit](https://github.com/JulioAmaral007/Biodigestor/tree/main/Adafruit).

## Configuração de Software:

### Pinagem:

| Pino (Nome no Código) | Número do Pino | Observações                                                                     |
| --------------------- | -------------- | ------------------------------------------------------------------------------- |
| `DHTPIN`              | 4              | Utilizado para leitura de dados do sensor DHT11                                 |
| `MQ2PIN`              | 35             | Utilizado para leitura de dados do sensor MQ2                                   |
| `RELAY`               | 14             | Utlizado para controle do relé                                                  |
| `SDA`                 | 21             | Utilizado como barramento de dados da comunicação I2C dos sensores BMP280       |
| `SCL`                 | 22             | Utilizado como sinal de clock da comunicação I2C dos sensores BMP280            |

Certifique-se de ajustar as configurações de acordo com as especificações do seu projeto.

### Ambiente de Desenvolvimento:

O código-fonte foi desenvolvido na Arduino IDE, uma plataforma de desenvolvimento que permite a escrita, compilação e upload de programas para placas compatíveis com Arduino, como o ESP32 utilizado neste projeto.

### Programa Final:

A pasta Programas [codigo_final/final_code.ino](https://github.com/JulioAmaral007/Biodigestor/blob/main/codigo_final/final_code.ino) contém o código-fonte consolidado que será executado no ESP32. Inclui a integração dos códigos de todos os sensores utilizados.

## Tabela de Conteúdos:

- [Proposta](#proposta)
- [Componentes Utilizados](#componentes-utilizados)
- [Arquitetura do Projeto](#arquitetura-do-projeto)
- [Integração com a Plataforma Adafruit IO](#integração-com-a-plataforma-adafruit-io)
- [Instruções de Execução](#instruções-de-execução)

## Instruções de Execução:

Para executar o projeto localmente, siga estas etapas:

1. **Pré-requisitos**:
   - Instale a última versão da Arduino IDE a partir do [site oficial](https://www.arduino.cc/en/software).
   - Configure a Arduino IDE para suportar o ESP32 seguindo as instruções disponíveis na [documentação do ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
   - Instale todas as bibliotecas necessárias mencionadas no arquivo [bibliotecas](./bibliotecas.txt).

2. **Execução da Aplicação**:
   - **Carregar o Código no ESP32**:
     1. Conecte o ESP32 ao seu computador.
     2. Abra o Arduino IDE.
     3. Carregue o código no ESP32.
        - [Código Final](https://github.com/JulioAmaral007/Biodigestor/blob/main/codigo_final/final_code.ino)
   - **Monitoramento via Adafruit IO**:
     1. Configure suas credenciais no código para conexão com a Adafruit IO.
     2. Verifique os dados em tempo real no dashboard da Adafruit IO.
     3. Caso precise no link a seguir estará o passo-a-passo da integração com o [adafruit](https://github.com/JulioAmaral007/Biodigestor/tree/main/Adafruit).

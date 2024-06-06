# Biodigestor com ESP32 e Sensores

## Problema Resolvido

O projeto visa facilitar a operação e o monitoramento de sistemas de biodigestão, fornecendo dados precisos em tempo real sobre as condições do ambiente. Isso é crucial para o desempenho eficiente do processo de biodigestão, permitindo ajustes proativos e garantindo um funcionamento seguro e otimizado do sistema. O monitoramento contínuo das condições ambientais possibilitado pelos sensores contribui para a maximização da produção de biogás e para a sustentabilidade de sistemas de tratamento de resíduos orgânicos.

## Componentes Utilizados

- ESP32 (microcontrolador)
- DHT11 (sensor de temperatura e umidade)
- BMP280 (sensor de pressão atmosférica)
- MQ-2 (sensor de gás)

## Arquitetura do Projeto

O projeto segue uma arquitetura onde o ESP32 coleta dados dos sensores e os envia para a plataforma Adafruit via protocolo MQTT. A plataforma Adafruit recebe os dados e os exibe em um dashboard online, permitindo o monitoramento remoto do sistema de biodigestão.

![Arquitetura do Projeto](./arch.jpg)

## Funcionamento dos Sensores no Projeto de Biodigestor

No projeto de biodigestor, diferentes sensores desempenham funções específicas para monitorar as condições ambientais e garantir a eficiência e segurança do sistema. Aqui está como cada sensor funcionará:

### [Sensor MQ-2 (Detecção de Gases Combustíveis)](https://github.com/JulioAmaral007/Biodigestor/blob/main/C%C3%B3digos/codigo_mq2.txt)

O sensor MQ-2 é essencial para a segurança, detectando a presença de gases combustíveis para identificar vazamentos potenciais. Deve ser posicionado fora do biodigestor, próximo a áreas propensas a vazamentos de gases como metano, produzido durante a digestão anaeróbica de resíduos orgânicos. O sensor será conectado ao ESP32, que lerá os valores de concentração de gás do pino analógico do MQ-2. Os dados coletados serão enviados para a plataforma Adafruit IO para monitoramento em tempo real, permitindo alertas em caso de detecção de níveis perigosos de gás.

### [Sensor DHT11 (Medição de Temperatura Ambiente)](https://github.com/JulioAmaral007/Biodigestor/blob/main/C%C3%B3digos/codigo_dht.txt)

O sensor DHT11 será usado para medir a temperatura do ambiente onde o biodigestor está localizado. Deve ser posicionado próximo ao biodigestor para capturar com precisão a temperatura ambiente, importante para o monitoramento do processo de biodigestão. O ESP32 lerá os valores de temperatura do pino de dados do DHT11 e enviará essas informações para a Adafruit IO. Alertas podem ser configurados na plataforma em caso de variações significativas de temperatura.

### [Sensor BMP280 (Medição de Pressão e Temperatura Interna)](https://github.com/JulioAmaral007/Biodigestor/blob/main/C%C3%B3digos/codigo_bmp.txt)

O sensor BMP280 será colocado dentro do biodigestor para monitorar a pressão atmosférica e a temperatura no ambiente interno. Conectado ao ESP32 através dos pinos de comunicação I2C (SCL e SDA), o BMP280 enviará os dados de pressão e temperatura para a plataforma Adafruit IO. Isso permitirá que os operadores monitorem as condições internas do biodigestor em tempo real e tomem medidas preventivas se necessário.

### Integração com a Plataforma Adafruit IO

Todos os dados coletados pelos sensores (MQ-2, DHT11 e BMP280) serão enviados para a plataforma Adafruit IO por meio de um ESP32. Isso permite que os dados sejam visualizados e analisados em tempo real pelos operadores, proporcionando um monitoramento abrangente do sistema de biodigestão.

## Tabela de Conteúdos

- [Problema Resolvido](#problema-resolvido)
- [Componentes Utilizados](#componentes-utilizados)
- [Arquitetura do Projeto](#arquitetura-do-projeto)
- [Funcionamento dos Sensores no Projeto de Biodigestor](#funcionamento-dos-sensores-no-projeto-de-biodigestor)
- [Integração com a Plataforma Adafruit IO](#integração-com-a-plataforma-adafruit-io)
- [Instruções de Execução](#instruções-de-execução)
- [Contribua com o Projeto](#contribua-com-o-projeto)
- [Extra - Adicione Badges](#extra---adicione-badges)
- [Changelog](#changelog)

## Instruções de Execução

Para executar o projeto localmente, siga estas etapas:

1. **Pré-requisitos**:
   - Node.js
   - Python
   - Bibliotecas do Arduino IDE (para ESP32 e sensores)

2. **Instalação das Dependências**:
   ```bash
   # Instalar bibliotecas do Arduino IDE
   Instale as bibliotecas DHT, Adafruit BMP280 e Adafruit MQTT.
   ```

3. **Execução da Aplicação**:
   - **Carregar o Código no ESP32**:
     1. Conecte o ESP32 ao seu computador.
     2. Abra o Arduino IDE.
     3. Carregue os códigos fornecidos para cada sensor no ESP32.
        - [Código MQ-2](https://github.com/JulioAmaral007/Biodigestor/blob/main/C%C3%B3digos/codigo_mq2.txt)
        - [Código DHT11](https://github.com/JulioAmaral007/Biodigestor/blob/main/C%C3%B3digos/codigo_dht.txt)
        - [Código BMP280](https://github.com/JulioAmaral007/Biodigestor/blob/main/C%C3%B3digos/codigo_bmp.txt)
   - **Monitoramento via Adafruit IO**:
     1. Configure suas credenciais no código para conexão com a Adafruit IO.
     2. Verifique os dados em tempo real no dashboard da Adafruit IO.

## Contribua com o Projeto

Para contribuir com o projeto, siga as instruções detalhadas no arquivo [CONTRIBUTING](./CONTRIBUTING.md).

## Extra - Adicione Badges

Adicione badges ao seu README para melhorar a apresentação e fornecer informações rápidas. Exemplos de badges estão disponíveis no projeto [markdown-badges](https://github.com/Ileriayo/markdown-badges).

## Changelog

Mantenha um histórico das versões e mudanças no projeto utilizando um [CHANGELOG](./CHANGELOG.md). Isso ajudará a acompanhar o progresso e as melhorias feitas ao longo do tempo.

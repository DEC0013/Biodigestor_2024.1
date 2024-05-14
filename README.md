
# Biodigestor com ESP32 e Sensores

Este projeto implementa um sistema de biodigestão utilizando um microcontrolador ESP32 e diversos sensores para monitoramento ambiental. O biodigestor converte resíduos orgânicos em biogás, e os sensores ajudam a monitorar parâmetros como temperatura, umidade, pressão atmosférica e qualidade do ar no ambiente do biodigestor.


## Problema Resolvido

O projeto visa facilitar o monitoramento de sistemas de biodigestão, fornecendo dados precisos em tempo real sobre as condições do ambiente, o que pode ser crucial para o desempenho eficiente do processo de biodigestão.

## Componentes Utilizados

 - ESP32 (microcontrolador)
 - DHT11 (sensor de temperatura e umidade)
 - BMP280 (sensor de pressão atmosférica)
 - MQ-2 (sensor de gás)
 - Relé (simular a válvula)

## Arquitetura do Projeto

O projeto segue uma arquitetura onde o ESP32 coleta dados dos sensores e os envia para a plataforma Adafruit via protocolo MQTT. A plataforma Adafruit recebe os dados e os exibe em um dashboard online, permitindo o monitoramento remoto do sistema de biodigestão.

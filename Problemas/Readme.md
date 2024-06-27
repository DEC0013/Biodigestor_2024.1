# Soluções para Problemas com Sensores e ESP32

### Problema com a comunicação entre o ESP32 e o computador

**Descrição do Problema:** Durante a utilização do ESP32, é necessário verificar se o cabo USB trasnfere dados ou é somente usado
para alimentação, o que nesse caso não seria viável para a utlização do hardware. É necessário também fazer o download dos drivers
para a utilização das portas do comunicação do computador, que serão usadas para se comunicar com o ESP32 através do cabo USB.

**Solução:** Troca de cabo USB que seja capaz de transferir dados e download dos drivers das portas de comunicação, a seguir: [Transferir dados e download dos drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads)

### Problema na compilação e execução do código do ESP32 na IDE Arduino

**Descrição do Problema:** Para que o código seja compilado corretamente, é preciso que na IDE Arduino seja selecionado o modelo correto
da placa do ESP32 e selecionado a porta de comunicação correta. Também na IDE, é possível que ocorra o erro de que ao final da compilação
do código, esse não seja transferido para o ESP32 pois não está no modo download, o que varia em certos modelos e placas desse hardware.

**Solução:** Selecionar o modelo do ESP32 e porta de comunicação corretos, e caso o erro de compilação ocorra, segurar pressionado o botão
de boot no momento em que a IDE alertar que está fazendo o download do código na placa, e quando finalizar, apertar o botão de enable.

### Problema no esquema de pinagem do sensor DHT11

**Descrição do Problema:** O sensor de temperatura DHT11 possui além do sensor por si, outros módulos com diferentes pinagens, o que pode
influenciar no funcionamento correto do sensor.

**Solução:** Verificar pinagem do sensor ou do módulo em que está soldado antes de sua utilização.

### Problema com a comunicação I2C no módulo do sensor BMP280

**Descrição do Problema:** O sensor BMP280 pode realizar a comunicação I2C através de dois endereços de memória, 0x76 e 0x77,
alimentando em 0V ou 3.3V o pino SDO/SPI. Isso pode alterar o teste correto do sensor, alertando no terminal que não foi 
possível encontrar um sensor BMP280, visto que em algumas bibliotecas, o seu endereço é definido por padrão um dos dois endereços.

**Solução:** Verificar na biblioteca usada para o sensor BMP280 se há alguma função que define seu endereço, e qual é usado por padrão, podendo confirmar qual endereço o sensor está usando por meio de um código I2C Scan, como esse: [Código I2C Scan](https://learn.adafruit.com/scanning-i2c-addresses/arduino)

### Problema com a leitura de dados do sensor MQ-2

**Descrição do Problema:** O sensor MQ-2 requer uma alimentação de 5V para melhor precisão da leitura dos dados. Ainda com o propósito
de uma leitura de dados precisa, as bibliotecas que utilizam o MQ-2, possuem certos valores que precisam ser definidos para
a percepção do tipo de gás correto pelo sensor.

**Solução:** Alimentar o MQ-2 corretamente com 5V e definir os valores certos para a leitura do gás desejado, como no código utilizado nesse
projeto, foi necessário ultilizar as funções MQ.RSRoMQAir(9.83), MQ.setRL(10), MQ.valuea(20.7074), MQ.valueb(-0.36), MQ.dangerousPer(17.86)
com esse valores para que o gás CH4 fosse lido.

### Problema com a integração dos sensores

**Descrição do Problema:** Na integração dos sensores, para que captassem os dados corretamente, foi necessário um delay entre suas leituras
no código, visto que é necessário um certo intervalo de tempo para que essa informações fossem lidas e processadas corretamente pelo ESP32.
Também foi essencial que fosse alimentado o pino SDO/SPI de um dos sensores BMP280 com 3.3V, sendo possível a leitura dos dois sensores por
meio da comunicação I2C, como visto em um problema anterior.

**Solução:** Ajustar delay entre a leitura dos sensores no código e alimentar o pino SDO/SPI de um dos sensores BMP280 com 3.3V.
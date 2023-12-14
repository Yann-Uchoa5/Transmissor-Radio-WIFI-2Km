#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RF24.h>
#include <SPI.h>

// Define os pinos para o módulo NRF24L01
#define CE_PIN   4
#define CSN_PIN  5

// Define os pinos para o barramento I2C
#define I2C_SDA  32
#define I2C_SCL  33

// Endereço de transmissão do módulo NRF24L01
const byte address[6] = "00002";

// Estrutura para armazenar os dados do sensor
struct SensorData {
  float temperature; // Temperatura em graus Celsius
  float humidity;    // Umidade relativa em percentagem
  float pressure;    // Pressão atmosférica em hectopascais (hPa)
  float weight;      // Peso simulado em quilogramas (substitua pela leitura real do sensor de peso)
};

// Inicializa o módulo NRF24L01
RF24 radio(CE_PIN, CSN_PIN);

// Inicializa o barramento I2C para o sensor BME280
TwoWire I2CBME = TwoWire(0);

// Inicializa o sensor BME280
Adafruit_BME280 bme;

// Função para inicializar a comunicação com o módulo NRF24L01
void initNRF24Communication() {
  radio.begin(); // Inicia o módulo NRF24L01
  radio.openWritingPipe(address); // Define o endereço de transmissão
  radio.setPALevel(RF24_PA_HIGH); // Configura a potência de transmissão para alta
  Serial.println("[RF24 Transmitter] Inicialização concluída");
}

// Função para ler os dados do sensor BME280
void readSensorData(SensorData &data) {
  // Lê os dados do sensor BME280
  data.temperature = bme.readTemperature();
  data.humidity = bme.readHumidity();
  data.pressure = bme.readPressure() / 100.0F;  // Converte para hPa
  data.weight = random(1, 5);  // Simula um valor de peso (substitua pela leitura real do sensor de peso)
}

// Função para enviar os dados do sensor via NRF24L01
void sendSensorData(SensorData data) {
  radio.stopListening(); // Para de escutar para poder transmitir
  radio.write(&data, sizeof(SensorData)); // Envia os dados
  radio.startListening(); // Volta a escutar para possíveis respostas
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Inicializa o barramento I2C para o sensor BME280
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  // Inicializa o sensor BME280
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("Erro ao iniciar o sensor BME280. Verifique as conexões!");
    while (1);
  }

  // Inicializa a comunicação com o módulo NRF24L01
  initNRF24Communication();

  // Mensagem indicando que o setup foi concluído
  Serial.println("[Setup] Setup concluído");
}

void loop() {
  // Estrutura para armazenar os dados do sensor
  SensorData sensorData;

  // Leitura dos dados do sensor BME280
  readSensorData(sensorData);

  // Exibe os dados no console serial
  Serial.print("Enviando dados - ");
  Serial.print("Temperature: ");
  Serial.print(sensorData.temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(sensorData.humidity);
  Serial.print(" %, Pressure: ");
  Serial.print(sensorData.pressure);
  Serial.print(" hPa, Weight: ");
  Serial.print(sensorData.weight);
  Serial.println(" kg");

  // Envia os dados para o receptor via NRF24L01
  sendSensorData(sensorData);

  // Aguarda 5 segundos antes de enviar novamente (ajuste conforme necessário)
  delay(5000);
}

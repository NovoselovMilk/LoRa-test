#include <Arduino.h>
#include <LoRa.h>
LoRa lora; // объект класса

#if defined( WIFI_LoRa_32 ) 
// HELTEC_LORA_V1
#define PIN_LED   GPIO_NUM_25
#define PIN_RESET GPIO_NUM_14
#define PIN_DIO0  GPIO_NUM_26
#define PIN_DIO1  GPIO_NUM_33
#define SPI_NSS   GPIO_NUM_18
#define SPI_BUS   HELTEC_LORA
#elif defined( WIFI_LoRa_32_V2 ) // WIFI_LoRa_32
// HELTEC_LORA_V2 (работают криво, т.е. не работают)
#define PIN_LED   GPIO_NUM_25
#define PIN_RESET GPIO_NUM_14
#define PIN_DIO0  GPIO_NUM_34
#define PIN_DIO1  GPIO_NUM_35
#define PIN_IRQ   GPIO_NUM_26
#define SPI_NSS   GPIO_NUM_18
#define SPI_BUS   HELTEC_LORA
#elif ~(defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 )) // WIFI_LoRa_32_V2
// OTHER ESP
// #define PIN_LED   GPIO_NUM_2
#define PIN_RESET GPIO_NUM_33
#define PIN_DIO0  GPIO_NUM_27
#define PIN_DIO1  GPIO_NUM_26
// #define PIN_DIO3  GPIO_NUM_25
#define SPI_NSS   GPIO_NUM_32
#define SPI_BUS   VSPI
#endif // ~(defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ))



// --- --- --- Настройки передатчика LoRa --- --- ---
//
// - [Frf] Радиочастота
// Разрешённый диапазон 433.25 МГц - 434.75 МГц с шагом не меньше 25КГц
// 43325E4 и 43455E4 используются для теплиц в ФИЦе, будут и другие каналы
// На 43375E4 электронщики тестировали LoRa модули
#define BAND             433375E3
// - Усиление сигнала
// true|false (при false сигнал очень плохой)
#define PABOOST          true
// - [SP] Сила сигнала
// 4-20 дБм
#define SIGNAL_POWER     14
// - [SF] Коэффициент распространения (число N чирпов для передачи символа: N = 2^SF)
// 7-12 (6 только при особых условиях)
#define SPREADING_FACTOR 8
// - [SBW] Пропускная способность сигнала
// 7.8, 10.4, 15.6, 20.8, 31.2, 41.7, 92.5, 125, 250, 500 КГц
#define SIGNAL_BANDWIDTH 150E3
// - Слово синхронизации
// 0-255 (0x12 базовая, 0x34 зарезервировано для сетей LoRaWAN, на 0x4A работают теплицы)
#define SYNC_WORD 0xA5
//
// В связи с устройством LoRa сигнала он занимает не только свою радиочастоту,
// а BAND±(SBW/2) , также при приёме сигнала, на случай неточностей в работе кварца,
// имеется защита, поэтому LoRa принимает смещённый по частоте сигнал, но не более
// чем на 25% от SBW: BAND±(1.25*SBW/2).
// То есть минимальный промежуток, между частотами работы 2-х LoRa модулей, чтобы
// они друг с другом не взаимодействовали и не мешали:
// ΔFrf = (SBW1 + SBW2)/2 + max(SBW1, SBW2)/4 , округлённое до
//     ближайшего большего числа кратного 25КГц.
//
// --- --- ---  --- --- ---  --- --- ---  --- --- ---

// #define SENDING_MODE
  #define RECEPTION_MODE
// #define DEBUG

void interrupt_init(uint8_t pin_dio0, uint8_t pin_dio1);
void interrupt_lora_dio0();
void interrupt_lora_dio1();

uint8_t LoRa_begin_result = 0xFF;
volatile uint16_t itr_dio0 = 0, itr_dio1 = 0;
#define MAX_SIZE 99
uint8_t buffer[MAX_SIZE] = "Hello world!";
uint8_t len = 13;
// lora_receive_mode - чтобы после проверки прерывания (при
//     работе с обоими режимами) не запустилась функция приёма
//     В main поддержки 2-х режимов нет.
bool lora_receive_mode = false;

bool end_receive = false;
bool end_send = false;
bool start_send = false;

uint32_t tick_send_packet; // время прошедшее с момента отправки пакета



void setup() {
    Serial.begin(115200);
    while(!Serial){}

    // --- Работа с LoRa модулем
    // Инициализация пинов и SPI-шины
    lora.init(PIN_RESET, SPI_BUS, SPI_NSS, PIN_DIO0, PIN_DIO1);
    // Запуск с указанными параметрами
    if(lora.begin(BAND, PABOOST, SIGNAL_POWER, SPREADING_FACTOR, SIGNAL_BANDWIDTH, SYNC_WORD))
        Serial.println("Begin error!");
        
    // Включение прерываний
    interrupt_init(PIN_IRQ, PIN_DIO1);

    delay(2000); 
    Serial.println("Start!");

    tick_send_packet = millis();

#ifdef RECEPTION_MODE 
    // Включение приёма пакета
    lora.receiver_packet(1, 0);
    Serial.println("Start receive.");
#endif
}

void loop() {
#ifdef SENDING_MODE
    // Отправка пакета каждый 5 секунд
    if((millis() - tick_send_packet > 5000) && (!start_send)) {
        start_send = true;
        if(lora.sender_packet(buffer, len, 0))
            Serial.println("Send error!");
        tick_send_packet = millis();
        Serial.println("Start send.");
    }
    if(end_send) {
        start_send = false;
        end_send = false;
        Serial.println("End send.");
    }
#endif

#ifdef RECEPTION_MODE
    if(end_receive) {
        end_receive = false;

        Serial.print("End receive: {");
        for(int i = 0; i < len; ++i) {
            Serial.print(buffer[i], HEX);
            if(i < len - 1)
                Serial.print(", ");
        }
        Serial.println("}");

        lora_receive_mode = true;
        lora.receiver_packet(1, 0);
        Serial.println("Start receive.");
    }
#endif

#ifdef DEBUG

#endif

    // Вывод прерываний
    static uint16_t last_itr_dio0 = 0;
    if(itr_dio0 != last_itr_dio0) {
        last_itr_dio0 = itr_dio0;
        Serial.print("'");
    }
    static uint16_t last_itr_dio1 = 0;
    if(itr_dio1 != last_itr_dio1) {
        last_itr_dio1 = itr_dio1;
        Serial.print(",");
    }
}


void interrupt_init(uint8_t pin_dio0, uint8_t pin_dio1) {
    if(pin_dio0 != 0) {
        pinMode(pin_dio0, INPUT);
        attachInterrupt(digitalPinToInterrupt(pin_dio0), interrupt_lora_dio0, RISING);
    }
    if(pin_dio1 != 0) {
        pinMode(pin_dio1, INPUT);
        attachInterrupt(digitalPinToInterrupt(pin_dio1), interrupt_lora_dio1, RISING);
    }
}

uint8_t receive_packet(uint8_t *buf, uint8_t max_size) {
	LoRa_packet packet = lora.receiver_packet(0, 0);
	uint8_t i = 0;
	for(uint8_t len = packet.get_len(); i < len && i < max_size; ++i) {
		buf[i] = packet[i];
	}
	return i;
}

void interrupt_lora_dio0() {
    ++itr_dio0;
#ifdef SENDING_MODE
    Serial.println("interrupt");
    if(!lora_receive_mode) {
        end_send = true;
        lora.mode_sleep();
    }
#endif

#ifdef RECEPTION_MODE
    if(lora_receive_mode) {
        len = receive_packet(buffer, MAX_SIZE);
        lora_receive_mode = false;
        end_receive = true;
        lora.mode_sleep();
    }
#endif

}

void interrupt_lora_dio1() {
    ++itr_dio1;
#ifdef RECEPTION_MODE
    lora.receiver_packet(1, 0);
    lora_receive_mode = true;
#endif
}

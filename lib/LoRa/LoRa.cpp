#include "LoRa.h"

//   ----- ----- ----- - - - - - - - - - ----- ----- -----
// ----- ----- ----- Список регистров LoRa ----- ----- -----
//   ----- ----- ----- - - - - - - - - - ----- ----- -----
const uint8_t
REG_FIFO = 0x00,
// --- Общие регистры настроек ---
REG_OP_MODE = 0x01,
REG_FRF_MSB = 0x06,
REG_FRF_MID = 0x07,
REG_FRF_LSB = 0x08,
// --- Регистры для блоков RF ---
REG_PA_CONFIG = 0x09,
REG_LNA = 0x0C,
// --- Страница регистров LoRa ---
REG_FIFO_ADDR_PTR = 0x0D,
REG_FIFO_TX_BASE_ADDR = 0x0E,
REG_FIFO_RX_BASE_ADDR = 0x0F,
REG_FIFO_RX_CURRENT_ADDR = 0x10,
REG_IRQ_FLAGS = 0x12,
REG_RX_NB_BYTES = 0x13,
REG_PKT_SNR_VALUE = 0x19,
REG_PKT_RSSI_VALUE = 0x1A,
REG_MODEM_CONFIG_1 = 0x1D,
REG_MODEM_CONFIG_2 = 0x1E,
REG_PREAMBLE_MSB = 0x20,
REG_PREAMBLE_LSB = 0x21,
REG_PAYLOAD_LENGTH = 0x22,
REG_MODEM_CONFIG_3 = 0x26,
REG_DETECTION_OPTIMIZE = 0x31,
REG_DETECTION_THRESHOLD = 0x37,
REG_SYNC_WORD = 0x39,
// --- Регистры управления IO ---
REG_DIO_MAPPING_1 = 0x40,
REG_DIO_MAPPING_2 = 0x41,
// --- Регистр версий ---
REG_VERSION = 0x42,
// --- Дополнительные регистры ---
REG_PA_DAC = 0x4D//,

;

// modes
#define MODE_SLEEP 0x00 // Спящий режим
#define MODE_STDBY 0x01 // Режим ожидания
#define MODE_FSTX 0x02  // Синтез частот TX
#define MODE_TX 0x03    // Передача пакета
#define MODE_FSRX 0x04  // Синтез частот RX
#define MODE_RX_CONTINUOUS 0x05 // Непрерывное получение
#define MODE_RX_SINGLE 0x06 // Единичное получение
#define MODE_CAD 0x07 // Обнаружение активности канала

// 
#define RX_BASE_ADDR 0x00
#define TX_BASE_ADDR 0x01

// флаги IRQ

#define RX_TIME_OUT 0x80
#define RX_DONE 0x40
#define PAY_LOAD_CRC_ERROR 0x20
#define VALID_HEADER 0x10
#define TX_DONE 0x08
// PaDac
#define RF_PADAC_20DBM_ON  0x07
#define RF_PADAC_20DBM_OFF 0x04

// DetectionThreshold
#define DT_SF6 0x0C
#define DT_SF7_12 0x0A
// DetectionOptimize
#define DO_SF6 0x05
#define DO_SF7_12 0x03
// Frequency
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08


LoRa::LoRa(/* args */) {
    _spi = nullptr;
    _setting = nullptr;
}
LoRa::LoRa(uint8_t pin_reset, uint8_t spi_bus, uint8_t spi_nss, uint8_t pin_dio0, uint8_t pin_dio1, uint8_t pin_dio3) {
    init(pin_reset, spi_bus, spi_nss, pin_dio0, pin_dio1, pin_dio3);
}

LoRa::~LoRa() {
    if(_spi != nullptr)
        delete _spi;
    if(_setting != nullptr)
        delete _setting;
}

bool LoRa::init(uint8_t pin_reset, uint8_t spi_bus, uint8_t spi_nss, uint8_t pin_dio0, uint8_t pin_dio1, uint8_t pin_dio3) {
    if((spi_bus != HSPI) && (spi_bus != VSPI) && (spi_bus != HELTEC_LORA)) {
        _init = false;
        return true;
    }
    if(spi_bus == HELTEC_LORA)
        _spi = new SPIClass(VSPI);
    else
        _spi = new SPIClass(spi_bus);
    _setting = new SPISettings(2E6, MSBFIRST, SPI_MODE0);
    // _setting = new SPISettings(8E6, MSBFIRST, SPI_MODE0);
    uint8_t sclk, miso, mosi;
    if(spi_bus == VSPI) {
        // sclk = 5; miso = 19; mosi = 27; // SS = 18;
        sclk = 18; miso = 19; mosi = 23; // SS = 5;
        if(spi_nss == 0)
            spi_nss = 5;
    }
    else if(spi_bus == HSPI) {
        sclk = 14; miso = 12; mosi = 13; // SS = 15;
        if(spi_nss == 0)
            spi_nss = 15;
    }
    else {
        sclk = 5; miso = 19; mosi = 27; spi_nss = 18;
    }
    _spi->begin(sclk, miso, mosi, spi_nss);
     init(_spi, _setting, spi_nss);
    _nss = spi_nss;
    _reset = pin_reset;
    _dio0 = pin_dio0;
    _dio1 = pin_dio1;
    _dio3 = pin_dio3;
	_FifoTxBaseAddr = 0;

    if (_dio0 != 0)
        pinMode(_dio0, INPUT);
    if (_dio1 != 0)
        pinMode(_dio1, INPUT);
    if (_dio3 != 0)
        pinMode(_dio3, INPUT);
        
    _init = true;
    return false;
}

void LoRa::init(SPIClass* set_spi, SPISettings* set_setting, uint8_t nss) {
    _spi = set_spi;
    _setting = set_setting;
    _nss = nss;
    pinMode(_nss, OUTPUT);
    digitalWrite(_nss, HIGH);
}
// uint8_t LoRa::field_set(Address_field field, uint32_t value, bool write) {
//     uint8_t result;
//     result =  set_field_value(field, value);
//     if(write) {
//         result =  register_write(field, true, true);
//     }
//     return result;
// }
// uint8_t LoRa::field_set(Address_field* fields, uint32_t* values, uint8_t amt, bool write) {
//     uint8_t result;
//     result =  set_field_value(fields, values, amt);
//     if(write) {
//         result =  register_write(fields, amt, true, true);
//     }
//     return result;
// }

// uint8_t LoRa::field_get(Address_field field, uint32_t* value, bool read) {
//     return  get_field_value(field, value, read);
// }
// uint8_t LoRa::field_get(Address_field fields[], uint32_t* values, uint8_t amt, bool read) {
//     return  get_field_value(fields, values, amt, read);
// }


uint8_t LoRa::begin(ulong frequency, bool paboost, uint8_t signal_power, uint8_t SF, ulong SBW, uint8_t sync_word) {
    if(!_init) return 1;
    uint32_t result = 0;                           
    //uint8_t amt;
    // настройка выходов
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, HIGH);

    // запуск модуля
    digitalWrite(_reset, LOW);
    delay(20);
    digitalWrite(_reset, HIGH);
    delay(50);
    
    // проверка версии LoRa-модуля
    // amt = field_get(Version, &result);

    result = _read_register(REG_VERSION);  
    if (result != 0x12) return 2;
    // переход в режим сна/настройки
    mode_sleep();
    // установка частоты работы модуля
    set_frequency(frequency);
    // установка адресов памяти RX и TX
    // if(field_set(FifoRxBaseAddr, 0x00) != 1) return 5;
    set_base_addr(RX_BASE_ADDR, 0x00);
    // if(field_set(FifoTxBaseAddr, _FifoTxBaseAddr) != 1) return 6;
    set_base_addr(TX_BASE_ADDR, _FifoTxBaseAddr);
    // настройка LNA
    //if(field_set(LnaBoostHf, 0x03) != 1) return 7;
    set_LNA(true);
    // установка low data rate optimize
    //if(field_set(LowDataRateOptimize, 0) != 1) return 8;
    set_data_optimize(false);
    // установка автоматического AGC
    //if(field_set(AgcAutoOn, 1) != 1) return 9;
    set_AGC(true);
    // установка силы сигнала на 14 дБ
    set_TX_power(signal_power, paboost);
    // установка силы коэффициента распространения SF
    set_spreading_factor(SF);
    // установка пропускной способности
    set_signal_bandwidth(SBW);
    // установка кодового слова 0x4A,  0x34 - LoRaWan
    set_sync_word(sync_word);
    // включение проверки ошибки пакета
    crc_enable();
    // переход в режим ожидания
    mode_STDBY();
    return 0;
}

void LoRa::end() {
    // переход в режим сна
    mode_sleep();
    // остановка SPI, если создавалась классом
    if (_spi != nullptr)
        _spi->end();
}


void LoRa::set_mode(uint8_t mode) {
    //  Address_field fields[3] = {LongRangeMode, LowFrequencyModeOn, Mode};
    //  uint32_t values[3] = {1, 0, mode}; 
    // return field_set(fields, values, 3);

    uint8_t reg = _read_register(REG_OP_MODE);
    reg = ((reg | 0x80) & 0xF7); 
    reg = ((reg & 0xF8) | (mode & 0x07)); 
    _write_register(REG_OP_MODE, reg);
}
// Режим сна/настройки
void LoRa::mode_sleep() {
    set_mode(MODE_SLEEP);
}
// Режим ожидания
void LoRa::mode_STDBY() {
    set_mode(MODE_STDBY);
}
// Режим отправки
void LoRa::mode_TX(bool set_dio) {    
    if (set_dio & (_dio0 != 0)){
       // field_set(Dio0Mapping, 1);
        uint8_t RegDioMapping1 = _read_register(REG_DIO_MAPPING_1);
        RegDioMapping1 = (RegDioMapping1 & 0x3F) | 0x40;
        _write_register(REG_DIO_MAPPING_1, RegDioMapping1);
    }
        
    set_mode(MODE_TX);
}
// Подготовка к отправке (настройка частот)
void LoRa::mode_FSTX() {
    set_mode(MODE_FSTX);
}
// Режим непрерывного приёма
void LoRa::mode_RX_continuous(bool set_dio) {
    // if (_dio0 != 0)
    //     field_set(Dio0Mapping, 0);
    // if (_dio1 != 0)
    //     field_set(Dio1Mapping, 0);
    if (set_dio & ((_dio0 != 0) || (_dio1 != 0))) {
        // field_set(Dio0Mapping, 0, false);
        // field_set(Dio1Mapping, 0);
        uint8_t RegDioMapping1 = _read_register(REG_DIO_MAPPING_1);
        RegDioMapping1 &= 0x0F;
        _write_register(REG_DIO_MAPPING_1, RegDioMapping1);
    }
    set_mode(MODE_RX_CONTINUOUS);
}
// Режим единичного приёма
void LoRa::mode_RX_single(bool set_dio) {
    if (set_dio & ((_dio0 != 0) || (_dio1 != 0))) {
        // field_set(Dio0Mapping, 0, false);
        // field_set(Dio1Mapping, 0);
        uint8_t RegDioMapping1 = _read_register(REG_DIO_MAPPING_1);
        RegDioMapping1 &= 0x0F;
        _write_register(REG_DIO_MAPPING_1, RegDioMapping1);
    }
    set_mode(MODE_RX_SINGLE);
}
// Подготовка к приёму (настройка частот)
void LoRa::mode_FSRX() {
    set_mode(MODE_FSRX);
}
// Режим проверки сети
void LoRa::mode_CAD(bool set_dio) {
    // if (_dio0 != 0)
    //     field_set(Dio0Mapping, 2);
    // if (_dio1 != 0)
    //     field_set(Dio1Mapping, 2);
    if (set_dio & ((_dio0 != 0) || (_dio1 != 0))) {
        // field_set(Dio0Mapping, 2, false);               // ??
        // field_set(Dio1Mapping, 2);
        uint8_t RegDioMapping1 = _read_register(REG_DIO_MAPPING_1);
        RegDioMapping1 = (RegDioMapping1 & 0x0F) | 0xA0;
        _write_register(REG_DIO_MAPPING_1, RegDioMapping1);
    }
    set_mode(MODE_CAD);
}


// Установка силы отправляемого пакета
uint8_t LoRa::set_TX_power(uint8_t power, bool paboost, uint8_t max_power) {
    //  Address_field fields[4] = {PaDac, PaSelect, MaxPower, OutputPower};
    //  //uint32_t pa_dac, pa_select;

      int16_t power_adjustment, min_power_value, max_power_value;

      uint8_t RegPaConfig = _read_register(REG_PA_CONFIG);
      uint8_t RegPaDac = _read_register(REG_PA_DAC);

    if (max_power < 0x01)
        max_power = 0x01;
    else if (max_power > 0x07)
        max_power = 0x07;
    //  register_read(fields, 4);
    // Изменение бита PABOOST
    if (paboost) {
        //pa_select = 1;
        RegPaConfig |= 0x80;
        min_power_value = 2;
        max_power_value = 20;
        if (power > 17)
            power_adjustment = -5;
        else
            power_adjustment = -2;
    }
    else {
        //pa_select = 0;
        RegPaConfig &= 0x7F;
        min_power_value = -1;
        max_power_value = 14;
        power_adjustment = 1;
    }
    // Проверка выхода силы сигнала за диапазон
    if (power < min_power_value)
        power = min_power_value;
    if (power > max_power_value)
        power = max_power_value;
    // Корректировка параметра
    power += power_adjustment;
    // Настройка флага высокого сигнала
    if (power_adjustment == -5)
       // pa_dac = RF_PADAC_20DBM_ON;
        RegPaDac |= RF_PADAC_20DBM_ON;
    else
        //pa_dac = RF_PADAC_20DBM_OFF;
        RegPaDac = (RegPaDac & 0xF8) | RF_PADAC_20DBM_OFF;
    // Передача настроек

    //  uint32_t values[4] = {pa_dac, pa_select, max_power, power};
    //  return field_set(fields, values, 4);

    _write_register(REG_PA_CONFIG, RegPaConfig);
    _write_register(REG_PA_DAC, RegPaDac);
    return 1;


}

// Установка частоты радиосигнала
uint8_t LoRa::set_frequency(ulong frequency) {

    _frequency = frequency;
    uint32_t frf = ((uint64_t)frequency << 19) / 32000000;
    uint8_t RegFrLsb = frf & 0xFF;
    uint8_t RegFrMid = frf >> 8 & 0xFF;
    uint8_t RegFrMsb = frf >> 16 & 0xFF;
    _write_register(REG_FRF_LSB, RegFrLsb);
    _write_register(REG_FRF_MID, RegFrMid);
    _write_register(REG_FRF_MSB, RegFrMsb);
    return 1;
}

// Установка силы коэффициента распространения SF
uint8_t LoRa::set_spreading_factor(uint8_t SF) {

    // uint8_t RegDetectThreshold;
    uint8_t RegModemConfig2 = _read_register(REG_MODEM_CONFIG_2);
    uint8_t RegDetectOptimize = _read_register(REG_DETECTION_OPTIMIZE);              
    uint8_t detection_optimize, detection_threshold;                                           // ВОПРОС
    if (SF < 6)
        SF = 6;
    else if (SF > 12)
        SF = 12;
    if (SF == 6) {
        detection_optimize = DO_SF6;
        detection_threshold = DT_SF6;
    }
    else {
        detection_optimize = DO_SF7_12;
        detection_threshold = DT_SF7_12;
    }

    SF <<= 4;
    RegModemConfig2 = (RegModemConfig2 & 0x0F) | (SF & 0xF0);
    RegDetectOptimize = (RegDetectOptimize & 0xF8) | (detection_optimize & 0x07);
    // RegDetectThreshold = detection_threshold;
    // Address_field fields[3] = {DetectionOptimize, DetectionThreshold, SpreadingFactor};
    // uint32_t values[3] = {detection_optimize, detection_threshold, SF};
    // return field_set(fields, values, 3);

    _write_register(REG_MODEM_CONFIG_2, RegModemConfig2);
    _write_register(REG_DETECTION_OPTIMIZE, RegDetectOptimize);
    _write_register(REG_DETECTION_THRESHOLD, detection_threshold);
    return 1;
}

// Установка пропускной способности
uint8_t LoRa::set_signal_bandwidth(ulong sbw) {
    uint8_t bw;
    if (sbw <= 7.8E3)        bw = 0;
    else if (sbw <= 10.4E3)  bw = 1;
    else if (sbw <= 15.6E3)  bw = 2;
    else if (sbw <= 20.8E3)  bw = 3;
    else if (sbw <= 31.25E3) bw = 4;
    else if (sbw <= 41.7E3)  bw = 5;
    else if (sbw <= 62.5E3)  bw = 6;
    else if (sbw <= 125E3)   bw = 7;
    else if (sbw <= 250E3)   bw = 8;
    else                     bw = 9;

    uint8_t reg = _read_register(REG_MODEM_CONFIG_1);
    reg = (reg & 0x0F) | (bw << 4 & 0xF0);
    _write_register(REG_MODEM_CONFIG_1, reg);
    return 1;

}

bool LoRa::set_sync_word(uint8_t SW){
    _write_register(REG_SYNC_WORD, SW);
    return 1;
}

// Установка длины преамбулы
bool LoRa::set_preamble_length(uint length) {
    // return field_set(PreambleLength, length);
    uint8_t RegPreambleLsb;
    uint8_t RegPreambleMsb;
    
    RegPreambleLsb = length & 0xFF;
    RegPreambleMsb = length >> 8 & 0xFF;
    _write_register(REG_PREAMBLE_LSB, RegPreambleLsb);
    _write_register(REG_PREAMBLE_MSB, RegPreambleMsb);  
    return 1;  
}


// Установка базового адреса буфера FIFO
bool LoRa::set_base_addr(bool mode, uint8_t base_addr){
if (mode) _write_register(REG_FIFO_TX_BASE_ADDR, base_addr); 
else _write_register(REG_FIFO_RX_BASE_ADDR, base_addr);
return 1;
}

// Установка LNA
bool LoRa::set_LNA(bool set){
    uint8_t RegLna = _read_register(REG_LNA);
    if(set) RegLna |= 0x03;
    else RegLna &= 0xFC;
    _write_register(REG_LNA, RegLna);
    return 1;
}

bool LoRa::set_data_optimize(bool set){
    uint8_t RegModemConfig3 = _read_register(REG_MODEM_CONFIG_3);
    if(set) RegModemConfig3 |= 0x08;
    else RegModemConfig3 &= 0xF7;
    _write_register(REG_MODEM_CONFIG_3, RegModemConfig3);
    return 1;
}

bool LoRa::set_AGC(bool set){
    uint8_t RegModemConfig3 = _read_register(REG_MODEM_CONFIG_3);
    if(set) RegModemConfig3 |= 0x04;
    else RegModemConfig3 &= 0xFB;
    _write_register(REG_MODEM_CONFIG_3, RegModemConfig3);
    return 1;
}

// Включение CRC на LoRa-модуле
bool LoRa::crc_enable() {
    //return field_set(RxPayloadCrcOn, 1);

    uint8_t RegModemConfig2 = _read_register(REG_MODEM_CONFIG_2);
    if (RegModemConfig2 >> 0x02 & 0x01) return 1;
    RegModemConfig2 |= 0x04; 
    _write_register(REG_MODEM_CONFIG_2, RegModemConfig2);
    return 1;
}
// Выключение CRC на LoRa-модуле
bool LoRa::crc_disable() {
    // return field_set(RxPayloadCrcOn, 0);

    uint8_t reg = _read_register(REG_MODEM_CONFIG_2);
    if (!(reg >> 0x02 & 0x01)) return 1;
    reg &= 0xFB; 
    _write_register(REG_MODEM_CONFIG_2, reg);
    return 1;
}


// Приём пакета
class LoRa_packet LoRa::receiver_packet(uint8_t count, ulong wait, bool rssi, bool snr) {
    class LoRa_packet send_packet;
    // Address_field fields[3] = {RxTimeout, RxDone, PayloadCrcError};
    // Address_field flags[3] = {RxDone, ValidHeader, PayloadCrcError};
    if(count <= 1) {
        if(count == 1)
            mode_RX_single();
        uint8_t rx_done, rx_timeout, crc_err;
        //uint8_t amt;
        bool signal = false;
        ulong time, start_time, read_time;
        int pin_done, pin_timeout, pin_crc_err;
        // uint32_t values[3] = {0, 0, 0};
        rx_done = rx_timeout = crc_err = 0;
        if(wait == 0)
            time = 0;
        else
            time = millis();
        for(start_time = time, read_time = time; (wait == 0) || (millis() - time < wait);) {
            // Считывание каждые 10 мс.
            if((wait == 0) || (millis() - read_time > 10U)) {
                // Если работают DIO выходы, то при HIGH хотя бы на одном из них, пускаем signal
                if ((_dio0 != 0) && (_dio1 != 0)) {
                    pin_done = digitalRead(_dio0);
                    pin_timeout = digitalRead(_dio1);
                    if (_dio3 != 0) {
                        pin_crc_err = digitalRead(_dio3);
                    }
                    else {
                        pin_crc_err = LOW;
                    }
                    if ((pin_done == HIGH) || (pin_timeout == HIGH) || (pin_crc_err == HIGH)) {
                        signal = true;
                    }
                }
                // Если неработают DIO выходы, был signal или превышено время ожидания
                if (((count == 0) && (wait == 0)) || (_dio0 == 0) || (_dio1 == 0) || (millis() - start_time > 2000) || signal) {
                    // amt = field_get(fields, values, 3, true);
                    // if(amt == 3) {
                    //     rx_timeout = values[0];
                    //     rx_done = values[1];
                    //     crc_err = values[2];
                    // }

                    uint8_t RegIrqFlags = _read_register(REG_IRQ_FLAGS);
                    rx_timeout = RegIrqFlags & RX_TIME_OUT;
                    rx_done = RegIrqFlags & RX_DONE;
                    crc_err = RegIrqFlags & PAY_LOAD_CRC_ERROR;
                }
                if(rx_done != 0) {
                    break;
                }
                if(rx_timeout != 0) {
                    //  clear_flags(RxTimeout);
                    _write_register(REG_IRQ_FLAGS, RX_TIME_OUT);
                    mode_RX_single(false);
                    rx_done = rx_timeout = crc_err = 0;
                    signal = false;
                    if(wait != 0)
                        start_time = millis();
                }
                if(wait == 0) {
                    break;
                }
            }
        }
        if((rx_done > 0) /*&& (crc_err == 0)*/) {
            // очистка флагов
            _write_register(REG_IRQ_FLAGS, RX_DONE | RX_TIME_OUT);
            //  clear_flags(RxDone);
            //  clear_flags(RxTimeout);
            send_packet = read_packet_data(crc_err, rssi, snr);
        }
        else {
            // field_get(fields, values, 3, true);
            _write_register(REG_IRQ_FLAGS, RX_DONE | VALID_HEADER | PAY_LOAD_CRC_ERROR);
            // clear_flags(flags, 3);
            // send_packet = LoRa_packet();
        }
        if(wait != 0)
            mode_sleep();
    }
    else {
        for(int i = 0; i < count; ++i) {
            
        }
    }
    return send_packet;
}

// Содержание последнего принятого пакета
class LoRa_packet LoRa::read_packet_data(bool crc_err, bool f_rssi, bool f_snr) {
    uint32_t length, adr;
    uint8_t rssi;//, *data;
    if (f_rssi)
        rssi = packet_rssi();
    else
        rssi = 0;

    // field_get(FifoRxBytesNb, &length, true);
    // field_get(FifoRxCurrentAddr, &adr, true);
    // field_set(FifoAddrPtr, adr);
    length = _read_register(REG_RX_NB_BYTES);
    adr = _read_register(REG_FIFO_RX_CURRENT_ADDR);
    _write_register(REG_FIFO_ADDR_PTR, adr);

    class LoRa_packet send_packet(nullptr, 0, crc_err, rssi);
    uint32_t data32 = 0;
    for(int i = 0; i < length; ++i) {
        // field_get(Fifo, &data32, true);
        data32 = _read_register(REG_FIFO);
        send_packet.add_packet_data(data32);
        // send_packet.add_packet_data((uint8_t)(data32 & 0xFF));
        // data[i] = data32 & 0xFF;
    }
    // data = new uint8_t[length];
    // uint32_t data32 = 0;
    // for(int i = 0; i < length; ++i) {
    //     field_get(Fifo, &data32, true);
    //     data[i] = data32 & 0xFF;
    // }
    // class LoRa_packet send_packet(data, length, crc_err, rssi);
    // delete[] data;
    return send_packet;
}

// RSSI последнего принятого пакета
uint8_t LoRa::packet_rssi() {
    uint8_t rssi = 0;
    //field_get(PacketRssi, &rssi, true);
    rssi = _read_register(REG_PKT_RSSI_VALUE);
    if (_frequency < 868E6)
        rssi -= 164;
    else
        rssi -= 157;
    return rssi;
}
// SNR последнего принятого пакета
float LoRa::packet_snr() {
    float snr = 0;
    // field_get(PacketSnr, (uint32_t*)&snr, true);
    uint8_t RegPktSnrValue = _read_register(REG_PKT_SNR_VALUE);
    snr = RegPktSnrValue;
    return (snr * 0.25);
}
// Отправка пакета
bool LoRa::sender_packet(const uint8_t* packet, uint8_t len, bool wait) {
    packet_begin();
    if (packet_write(packet, len))
        return true;
    if (packet_end(wait))
        return true;
    return false;
}
bool LoRa::sender_packet(const std::vector<uint8_t>& add_packet, bool wait) {
    return sender_packet(&add_packet[0], add_packet.size(), wait);
}
bool LoRa::sender_packet(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& add_packet, uint8_t len, bool wait) {
    return sender_packet(&add_packet[0], len, wait);
}
// Объявление пакета
bool LoRa::packet_begin() {
    mode_FSTX();
    //  field_set(FifoAddrPtr, _FifoTxBaseAddr);
    _write_register(REG_FIFO_ADDR_PTR, _FifoTxBaseAddr);
    _packet_length = 0;
    return true;
}
// Отправка данных в пакет buffer, size=None? (len)
bool LoRa::packet_write(const uint8_t* packet, uint8_t len) {
    if (len + _packet_length > 255)
        return true;
    _packet_length += len;
    for(int i = 0; i < len; ++i) 
        // field_set(Fifo, packet[i]);
        _write_register(REG_FIFO, packet[i]);
    // field_set(PayloadLength, _packet_length);
    _write_register(REG_PAYLOAD_LENGTH, _packet_length);
    return false;
}
bool LoRa::packet_write(const std::vector<uint8_t>& packet) {
    return packet_write(&packet[0], packet.size());
    // if (packet.size() + _packet_length > 255)
    //     return true;
    // _packet_length += packet.size();
    // for(int i = 0; i < packet.size(); ++i) 
    //     field_set(Fifo, packet[i]);
    // field_set(PayloadLength, _packet_length);
    // return false;
}

// Отправка пакета
bool LoRa::packet_end(ulong wait, bool sleep) {
    //  clear_flags(TxDone);
    mode_TX();
    bool result = false;
    if(wait > 0) {
        uint32_t tx_done = 0;
        bool signal = false;
        ulong time, start_time, read_time;
        int pin_done;
        tx_done = 0;
        for(time = millis(), start_time = time, read_time = time; millis() - time < wait;) {
            // Считывание каждые 10 мс.
            if(millis() - read_time > 10) {
                // Если работают DIO выходы, то при HIGH хотя бы на одном из них, пускаем signal
                if (_dio0 != 0) {
                    pin_done = digitalRead(_dio0);
                    if (pin_done == HIGH) {
                        signal = true;
                    }
                }
                // Если неработают DIO выходы, был signal или превышено время ожидания
                if ((_dio0 == 0) || (millis() - start_time > 2000) || signal) {
                    uint8_t RegIrqFlags = _read_register(REG_IRQ_FLAGS);
                    RegIrqFlags &= TX_DONE;
                    if (RegIrqFlags != 0){
                        tx_done = true;
                    }
                   // field_get(TxDone, &tx_done, true);
                }
                if(tx_done != 0) {
                    // очистка флаг 
                    _write_register(REG_IRQ_FLAGS, TX_DONE);
                    // fiekd_set(1->TxDone)
                    //break;
                }
            }
        }
        if(sleep && (tx_done != 0)) {
            mode_sleep();
        }
    }
    return result;
}

uint8_t LoRa::_read_register(uint8_t address) {
    return _single_transfer(address & 0x7f, 0x00); // 01111111
}
void LoRa::_write_register(uint8_t address, uint8_t value) {
    _single_transfer(address | 0x80, value); // 10000000
}
uint8_t LoRa::_single_transfer(uint8_t address, uint8_t value) {
    uint8_t response;
#if defined( ESP32 )
    // Подача NSS сигнала
    digitalWrite(_nss, LOW);
    // Отправка бита действия и 7 бит адреса
    _spi->beginTransaction(*_setting);
    _spi->transfer(address);
    // Отправка/приём байта значения
    response = _spi->transfer(value);
    _spi->endTransaction();
    // Прекращение NSS сигнала
    digitalWrite(_nss, HIGH);
#else
    // Подача NSS сигнала
    HAL_GPIO_WritePin(_nss_port, _nss_pin, GPIO_PIN_RESET);
    // Отправка бита действия и 7 бит адреса
    HAL_SPI_TransmitReceive(_spi, &address, &response, 1, 1000);
    for(int i = 0; i < 50; i++) __NOP();
    // Отправка/приём байта значения
    HAL_SPI_TransmitReceive(_spi, &value, &response, 1, 1000);
    for(int i = 0; i < 20; i++) __NOP();
    // Прекращение NSS сигнала
    HAL_GPIO_WritePin(_nss_port, _nss_pin, GPIO_PIN_SET);
#endif
    return response;
}


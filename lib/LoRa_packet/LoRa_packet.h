#ifndef __LORA_PACKET_H__
#define __LORA_PACKET_H__

#include "LoRa_packet_config.h"
#include "LoRa_packet_data.h"

class LoRa_packet {
private:
    LoRa_packet_data* packet_data{}; // Указатель на данные пакета
    uint8_t rssi_ = 0; // RSSI соединения
    bool crc_error_ = false; // Ошибка контрольной суммы

    // Функция поиска свободного места для пакета
    bool search_data();

    // Прямой допуск к полям для класса LoRa_packet_data
    friend class LoRa_packet_data;
public:
    LoRa_packet();
    LoRa_packet(const uint8_t* data, uint8_t len, bool crc_err=false, uint8_t rssi=0);
#if defined( USE_VECTOR )
    LoRa_packet(const std::vector<uint8_t>& data, bool crc_err=false, uint8_t rssi=0);
#endif
#if defined( USE_STANDARD_ARRAY )
    LoRa_packet(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len, bool crc_err=false, uint8_t rssi=0);
#endif
    LoRa_packet(const LoRa_packet& right);
    LoRa_packet(LoRa_packet&& right) noexcept;
    ~LoRa_packet();

    // Установить параметры и данные в пакете
    bool set_packet(const uint8_t* data, uint8_t len, bool crc_error=false, uint8_t rssi=0);
    bool set_packet(const LoRa_packet_data& data, bool crc_error=false, uint8_t rssi=0);
#if defined( USE_VECTOR )
    bool set_packet(const std::vector<uint8_t>& data, bool crc_error=false, uint8_t rssi=0);
#endif
#if defined( USE_STANDARD_ARRAY )
    bool set_packet(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len, bool crc_error=false, uint8_t rssi=0);
#endif

    // Дополнить данные в конец пакета
    bool add_packet_data(uint8_t data);
    bool add_packet_data(const uint8_t* data, uint8_t len);
#if defined( USE_VECTOR )
    bool add_packet_data(const std::vector<uint8_t>& data);
#endif
#if defined( USE_STANDARD_ARRAY )
    bool add_packet_data(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len);

    // Получить данные из пакета
    std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> get_data();
    std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> const & get_data() const;
#elif defined( USE_VECTOR )
    std::vector<uint8_t> get_data() const;
#endif
    LoRa_packet_data* get_packet();
    LoRa_packet_data const * get_packet() const;
    bool    get_crc_error() const; // получение ошибки передачи пакета
    uint8_t get_rssi() const; // получение RSSI пакета

    // получение одного байта
    uint8_t get_len() const;
    uint8_t get_data(int num) const;
    uint8_t& operator[](int index);
    const uint8_t& operator[](int index) const;

    // Очистка пакета
    void clear_packet();

    // Присваивание
    LoRa_packet& operator=(const LoRa_packet& right);
    LoRa_packet& operator=(LoRa_packet&& right) noexcept;

} typedef LoRa_packet;






#endif // __LORA_PACKET_H__

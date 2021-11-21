#ifndef __LORA_PACKET_DATA_H__
#define __LORA_PACKET_DATA_H__

#include "LoRa_packet_config.h"

class LoRa_packet;

class LoRa_packet_data {
private:
    bool free_object_ = true; // Свободный объект
#if defined( USE_STANDARD_ARRAY )
    uint8_t len_ = 0; // Количество байт
    std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> data_{};
#else // USE_VECTOR
    std::vector<uint8_t> data_{};
#endif
    // Прямой допуск к полям для класса LoRa_packet
    friend class LoRa_packet;
public:
    LoRa_packet_data() = default;
    ~LoRa_packet_data() = default;

    // Установить данные в пакете
    bool set_data(const uint8_t* data, uint8_t len);
    void set_data(const class LoRa_packet& lora_packet);
    void set_data(const class LoRa_packet_data& lora_packet);
#if defined( USE_VECTOR )
    bool set_data(const std::vector<uint8_t>& data);
#endif
#if defined( USE_STANDARD_ARRAY )
    bool set_data(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len);
#endif

    // Дополнить данные в конец пакета
    bool add_data(uint8_t data_byte);
    bool add_data(const uint8_t* data_byte, uint8_t amt_byte);
#if defined( USE_VECTOR )
    bool add_data(const std::vector<uint8_t>& data);
#endif
#if defined( USE_STANDARD_ARRAY )
    bool add_data(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data_byte, uint8_t amt_byte);

    // Получить данные из пакета
    std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> get_data();
    std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> const & get_data() const;
#elif defined( USE_VECTOR )
    std::vector<uint8_t> get_data() const;
#endif

    // получение одного байта
    uint8_t get_len() const;
    uint8_t get_data(int index) const;
    uint8_t& operator[](int index);
    const uint8_t& operator[](int index) const;

    // Проверка на то, является ли объект свободным
    bool free() const;

    // Присваивание
    class LoRa_packet_data& operator=(const class LoRa_packet& right);
    class LoRa_packet_data& operator=(const class LoRa_packet_data& right);
};

#include "LoRa_packet.h"

#endif // __LORA_PACKET_DATA_H__

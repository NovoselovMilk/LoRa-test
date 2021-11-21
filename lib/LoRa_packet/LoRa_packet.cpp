//
// Created by garvet on 05.02.2021.
//

#include "LoRa_packet.h"

#if defined( USE_STANDARD_ARRAY )
extern std::array<LoRa_packet_data, SIZE_LORA_PACKET_BUFFER> lora_packet_data;
#else
extern LoRa_packet_data lora_packet_data[SIZE_LORA_PACKET_BUFFER];
#endif

/// Private
bool LoRa_packet::search_data() {
#if defined( USE_STANDARD_ARRAY )
    packet_data = std::find_if(lora_packet_data.begin(), lora_packet_data.end(),
                               [](const LoRa_packet_data &data){return data.free();} );
    if(packet_data == lora_packet_data.end()) {
        packet_data = nullptr;
#if defined( ESP32 )
        Serial.println("!lora_packet_data memory error!");
#endif
        return true;
    }
    packet_data->free_object_ = false;
    packet_data->len_ = 0;
    return false;
#else
    for(int i = 0; i < SIZE_LORA_PACKET_BUFFER; ++i) {
        if(lora_packet_data[i].free_object_) {
            lora_packet_data[i].free_object_ = false;
            lora_packet_data[i].len_ = 0;
            packet_data = &lora_packet_data[i];
            return false;
        }
    }
    packet_data = nullptr;
    return true;
#endif
}

/// Public
LoRa_packet::LoRa_packet(): packet_data(nullptr) {
    search_data();
}
LoRa_packet::LoRa_packet(const uint8_t* data, uint8_t len, bool crc_error, uint8_t rssi): packet_data(nullptr) {
    search_data();
    set_packet(data, len, crc_error, rssi);
}
#if defined( USE_VECTOR )
LoRa_packet::LoRa_packet(const std::vector<uint8_t>& data, bool crc_error, uint8_t rssi): packet_data(nullptr) {
    search_data();
    set_packet(data, crc_error, rssi);
}
#endif
#if defined( USE_STANDARD_ARRAY )
LoRa_packet::LoRa_packet(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len,
                         bool crc_error, uint8_t rssi): packet_data(nullptr) {
    search_data();
    set_packet(data, len, crc_error, rssi);
}
#endif
LoRa_packet::LoRa_packet(const LoRa_packet& right): packet_data(nullptr) {
    search_data();
    *this = right;
}
LoRa_packet::LoRa_packet(LoRa_packet&& right) noexcept : packet_data(nullptr) {
    *this = std::move(right);
}
LoRa_packet::~LoRa_packet() {
    if(packet_data != nullptr)
        packet_data->free_object_ = true;
}


// Установить параметры и данные в пакете
bool LoRa_packet::set_packet(const uint8_t* data, uint8_t len, bool crc_error, uint8_t rssi) {
    if(packet_data->set_data(data, len))
        return true;
    crc_error_ = crc_error;
    rssi_ = rssi;
    return false;
}
bool LoRa_packet::set_packet(const LoRa_packet_data& data, bool crc_error, uint8_t rssi) {
    *packet_data = data;
    crc_error_ = crc_error;
    rssi_ = rssi;
    return false;
}
#if defined( USE_VECTOR )
bool LoRa_packet::set_packet(const std::vector<uint8_t>& data, bool crc_err, uint8_t rssi) {
    return set_packet(&data[0], data.size(), crc_err, rssi);
}
#endif
#if defined( USE_STANDARD_ARRAY )
bool LoRa_packet::set_packet(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len, bool crc_err, uint8_t rssi) {
    return set_packet(&data[0], len, crc_err, rssi);
}
#endif


// Дополнить данные в конец пакета
bool LoRa_packet::add_packet_data(uint8_t data) {
    return packet_data->add_data(data);
}
bool LoRa_packet::add_packet_data(const  uint8_t* data, uint8_t len) {
    return packet_data->add_data(data, len);
}
#if defined( USE_VECTOR )
bool LoRa_packet::add_packet_data(const std::vector<uint8_t>& data) {
    return packet_data->add_data(data);
}
#endif
#if defined( USE_STANDARD_ARRAY )
bool LoRa_packet::add_packet_data(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len) {
    return packet_data->add_data(data, len);
}


// Получить данные из пакета
std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> LoRa_packet::get_data() {
    return packet_data->get_data();
}
std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> const & LoRa_packet::get_data() const {
    return ((const LoRa_packet_data*) packet_data)->get_data();
}
#elif defined( USE_VECTOR )
std::vector<uint8_t> LoRa_packet::get_data() const {
    return packet_data->get_data();
}
#endif
LoRa_packet_data* LoRa_packet::get_packet() {
    return packet_data;
}
LoRa_packet_data const * LoRa_packet::get_packet() const {
    return packet_data;
}
bool LoRa_packet::get_crc_error() const {
    return crc_error_;
}
uint8_t LoRa_packet::get_rssi() const {
    return rssi_;
}


// получение одного байта
uint8_t LoRa_packet::get_data(int num) const {
    return packet_data->data_[num];
}
uint8_t LoRa_packet::get_len() const {
    if(packet_data == nullptr)
        return 0;
    return packet_data->len_;
}
uint8_t& LoRa_packet::operator[] (int index) {
    return packet_data->data_[index];
}
const uint8_t& LoRa_packet::operator[](int index) const {
    return packet_data->data_[index];
}


// Очистка пакета
void LoRa_packet::clear_packet() {
    if(packet_data == nullptr)
        search_data();
    else
        packet_data->len_ = 0;
    rssi_ = 0;
    crc_error_  = false;
}


// Присваивание
class LoRa_packet& LoRa_packet::operator=(const class LoRa_packet& right) {
    // Проверка на самоприсваивание
    if (this == &right)
        return *this;
    // Проверка на пустой объект
    if(packet_data == nullptr)
        search_data();
    // Перенос значений
    *packet_data = *right.packet_data; // * для копирования содержимого, а не указателя
    crc_error_ = right.crc_error_;
    rssi_ = right.rssi_;
    return *this;
}
class LoRa_packet& LoRa_packet::operator=(class LoRa_packet&& right) noexcept {
    // Проверка на самоприсваивание
    if (this == &right)
        return *this;
    // Перенос значений
    if(packet_data != nullptr) {
        packet_data->free_object_=true;
    }
    packet_data = right.packet_data;
    crc_error_ = right.crc_error_;
    rssi_ = right.rssi_;
    right.packet_data = nullptr;
    return *this;
}

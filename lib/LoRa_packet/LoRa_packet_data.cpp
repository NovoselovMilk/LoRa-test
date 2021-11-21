#include "LoRa_packet_data.h"

#if defined( USE_STANDARD_ARRAY )
std::array<LoRa_packet_data, SIZE_LORA_PACKET_BUFFER> lora_packet_data;
#else
LoRa_packet_data lora_packet_data[SIZE_LORA_PACKET_BUFFER];
#endif

// Установить данные в пакете
bool LoRa_packet_data::set_data(const uint8_t* data, uint8_t len) {
    if(len > SIZE_LORA_PACKET_MAX_LEN)
        return true;
    if((data != nullptr) && (len != 0)) {
        len_ = len;
        for(int i = 0; i < len_; ++i)
            data_[i] = data[i];
    }
    return false;
}
void LoRa_packet_data::set_data(const class LoRa_packet& lora_packet) {
    *this = lora_packet;
}
void LoRa_packet_data::set_data(const class LoRa_packet_data& lora_packet) {
    *this = lora_packet;
}
#if defined( USE_VECTOR )
bool LoRa_packet_data::set_data(const std::vector<uint8_t>& data){
    return set_data(&data[0], data.size());
//    if(data.size() > SIZE_LORA_PACKET_MAX_LEN)
//        return true;
//    if(!data.empty()) {
//        len_ = data.size();
//        for(size_t i = 0; i < len_; ++i)
//            data_[i] = data[i];
//    }
//    return false;
}
#endif
#if defined( USE_STANDARD_ARRAY )
bool LoRa_packet_data::set_data(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data, uint8_t len) {
    return set_data(&data[0], len);
//    if(len > SIZE_LORA_PACKET_MAX_LEN)
//        return true;
//    if(len != 0) {
//        len_ = len;
//        for(int i = 0; i < len_; ++i)
//            data_[i] = data[i];
//    }
//    return false;
}
#endif


// Дополнить данные в конец пакета
bool LoRa_packet_data::add_data(uint8_t data_byte) {
    if(len_ >= SIZE_LORA_PACKET_MAX_LEN)
        return true;
    data_[len_++] = data_byte;
    return false;
}
bool LoRa_packet_data::add_data(const uint8_t* data_byte, uint8_t amt_byte) {
    if((len_ + amt_byte) > SIZE_LORA_PACKET_MAX_LEN)
        return true;
    for(size_t i = 0; i < amt_byte; ++i)
        data_[i + len_] = data_byte[i];
    len_ += amt_byte;
    return false;
}
#if defined( USE_VECTOR )
bool LoRa_packet_data::add_data(const std::vector<uint8_t>& data_byte) {
    return add_data(&data_byte[0], data_byte.size());
//    if((len_ + data_byte.size()) > SIZE_LORA_PACKET_MAX_LEN)
//        return true;
//    for(size_t i = 0; i < data_byte.size(); ++i)
//        data_[i + len_] = data_byte[i];
//    len_ += data_byte.size();
//    return false;
}
#endif
#if defined( USE_STANDARD_ARRAY )
bool LoRa_packet_data::add_data(const std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN>& data_byte, uint8_t amt_byte) {
    return add_data(&data_byte[0], amt_byte);
//    if((len_ + amt_byte) > SIZE_LORA_PACKET_MAX_LEN)
//        return true;
//    for(size_t i = 0; i < amt_byte; ++i)
//        data_[i + len_] = data_byte[i];
//    len_ += amt_byte;
//    return false;
}


// Получить данные из пакета
std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> LoRa_packet_data::get_data() {
    return data_;
}
std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> const & LoRa_packet_data::get_data() const {
    return data_;
}
#elif defined( USE_VECTOR )
std::vector<uint8_t> LoRa_packet_data::get_data() const{
    std::vector<uint8_t> data;
    for(size_t i = 0; i < len_; ++i)
        data.push_back(data_[i]);
    return data;
}
#endif


// получение одного байта
uint8_t LoRa_packet_data::get_len() const {
    return len_;
}
uint8_t LoRa_packet_data::get_data(int index) const {
    return data_[index];
}
uint8_t& LoRa_packet_data::operator[] (int index) {
    return data_[index];
}
const uint8_t& LoRa_packet_data::operator[](int index) const {
    return data_[index];
}


// Проверка на то, является ли объект свободным
bool LoRa_packet_data::free() const {
    return free_object_;
}


// Присваивание
class LoRa_packet_data& LoRa_packet_data::operator=(const class LoRa_packet& right) {
    if(right.packet_data != nullptr) {
        *this = *right.packet_data;
    }
    else
        len_ = 0;
    return *this;
}
class LoRa_packet_data& LoRa_packet_data::operator=(const class LoRa_packet_data& right) {
    if((!(right.free())) && (right.get_len() != 0)){
        len_ = right.get_len();
        for(int i = 0; i < right.get_len(); ++i)
            data_[i] = right[i];
    }
    else
        len_ = 0;
    return *this;
}

#include "SHTC3.h"


// Constants
#define I2C_ADDR            ( (uint8_t) 0x70 )
#define RESET_DELAY_US      245
// Scalation constants Humidity K h = 100 * val / 2^16
// //(100.0f/65536.0f) * Val
#define H_K                 (0.001525878906f)  
// t = -45 + 175 * (val / 2^16 )
// 175.0f / /65536.0f) * val - 45.0f
#define T_K                 (0.002670288086f)
#define T_MIN               (45.0f)
#define T_MIN_INT           (-4500)
#define T_MAX_INT           (17500)


// Copied form senrion library.
// Credits: https://github.com/Sensirion/arduino-sht/
uint8_t SHTC3::crc8(const uint8_t *data, uint8_t len) {
  // adapted from SHT21 sample code from
  // http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = 0xff;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= data[byteCtr];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

bool SHTC3::twiCommand(uint16_t cmd,uint8_t stop) {
    uint8_t tx=0;
    _wire.beginTransmission(I2C_ADDR);
    tx+= _wire.write(cmd >> 8 );             // MSB first
    tx+= _wire.write(cmd & 0xFF);
    return _wire.endTransmission(stop) ==0 && (tx==2) ;
}

bool SHTC3::twiTransfer(uint16_t cmd,uint8_t *data,uint8_t len,uint8_t pause) {
    bool r=false;
    if(twiCommand(SHTC3_ID,false)) {
        if(pause>0) delay(pause);
        _wire.requestFrom(I2C_ADDR, len, (uint8_t)true);
        r= ( _wire.available() == len );
        if(r) {
            while(len--) *data++=_wire.read();
        }
    }
    return r;
}


bool SHTC3::begin(bool do_sample) {
    
    uint8_t id[3]; // ID + CRC
    bool r=false;
    delayMicroseconds(RESET_DELAY_US);
    if(twiTransfer(SHTC3_ID,id,3) && checkCRC(id)) {
        // xxxx' 1 xxx’  |  xx 00’0111  is chip signature  Check it
        r= (id[0] & 0b00001000) && ( (id[1] & 0b00111111) == 0b000001111 );
        // Soft reset of the device && put in sleep mode
        r = r && reset();
        delayMicroseconds(RESET_DELAY_US);
        r =  r  && ( (do_sample) ? sample() : sleep() );
    }
    return r;
}
bool SHTC3::sample(uint16_t readcmd,uint8_t pause) {
    bool r=false;
    uint8_t buff[6];    // tmp buffer
    if(wakeup()) {
        r = twiTransfer(readcmd,buff,6,pause);
        r = r && checkCRC(buff) && checkCRC(&buff[3]);
        if (r) { // translate measurement to values
            _t = (buff[0] << 8 ) + buff[1];
            _h = (buff[3] << 8 ) + buff[4];
        }
        sleep(); // Sleep at the end
    }
    return r;
}

#if defined(ARDUINO_SHTC3_NOFLOAT)

int16_t SHTC3::readTempC() {
    uint16_t t= 26 *_t;
    t+= (_t/1000)*(int16_t)7;
    return  t - (int16_t)4500;
}

uint16_t SHTC3::readHumidity() {
    uint16_t h= 15 * _h;
    h+= (_t/1000) * 2;
    return h;
}

#else

float SHTC3::readTempC(){
    return   ( _t * T_K ) - T_MIN; 
}

float SHTC3::readHumidity() {
    return (float)_h * H_K;
}

#endif
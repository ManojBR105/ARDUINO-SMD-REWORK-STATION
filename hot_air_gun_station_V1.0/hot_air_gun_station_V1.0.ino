/*
 * Hot air gun controller based on atmega328 IC
 * Released November 5, 2018
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <CommonControls.h>
#include <EEPROM.h>
#include <SPI.h>

const uint16_t temp_minC 	= 150;
const uint16_t temp_maxC	= 500;
const uint16_t temp_ambC    = 25;
const uint16_t temp_tip[3] = {200, 300, 400};                               // Temperature reference points for calibration

const uint8_t AC_SYNC_PIN   = 2;                                            // Outlet 220 v synchronization pin. Do not change!
const uint8_t HOT_GUN_PIN   = 7;                                            // Hot gun heater management pin
const uint8_t FAN_GUN_PIN   = 9;                                            // Hot gun fan management pin. Do not change! 
const uint8_t TEMP_GUN_PIN	= A0;                                           // Hot gun temperature checking pin

const uint8_t R_MAIN_PIN	= 3;                                            // Rotary encoder main pin. Do not change!
const uint8_t R_SECD_PIN	= 4;                                            // Rotary encoder secondary pin
const uint8_t R_BUTN_PIN	= 5;                                            // Rotary encoder button pin

const uint8_t REED_SW_PIN   = 11;                                            // Reed switch pin
const uint8_t BUZZER_PIN	= 8;                                            // Buzzer pin

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
 * uint32_t ID                           each time increment by 1
 * struct cfg                            config data, 8 bytes
 * byte CRC                              the checksum
*/
struct cfg {
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout
};

class CONFIG {
    public:
        CONFIG() {
            can_write     = false;
            buffRecords   = 0;
            rAddr = wAddr = 0;
            eLength       = 0;
            nextRecID     = 0;
            uint8_t rs = sizeof(struct cfg) + 5;                             // The total config record size
            // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
            for (record_size = 8; record_size < rs; record_size <<= 1);
        }
        void init();
        bool load(void);
        void getConfig(struct cfg &Cfg);                                    // Copy config structure from this class
        void updateConfig(struct cfg &Cfg);                                 // Copy updated config into this class
        bool save(void);                                                    // Save current config copy to the EEPROM
        bool saveConfig(struct cfg &Cfg);                                   // write updated config into the EEPROM

    protected:
        struct   cfg Config;

    private:
        bool     readRecord(uint16_t addr, uint32_t &recID);
        bool     can_write;                                                 // The flag indicates that data can be saved
        uint8_t  buffRecords;                                               // Number of the records in the outpt buffer
        uint16_t rAddr;                                                     // Address of thecorrect record in EEPROM to be read
        uint16_t wAddr;                                                     // Address in the EEPROM to start write new record
        uint16_t eLength;                                                   // Length of the EEPROM, depends on arduino model
        uint32_t nextRecID;                                                 // next record ID
        uint8_t  record_size;                                               // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
    eLength = EEPROM.length();
    uint32_t recID;
    uint32_t minRecID = 0xffffffff;
    uint16_t minRecAddr = 0;
    uint32_t maxRecID = 0;
    uint16_t maxRecAddr = 0;
    uint8_t  records = 0;

    nextRecID = 0;

    // read all the records in the EEPROM find min and max record ID
    for (uint16_t addr = 0; addr < eLength; addr += record_size) {
        if (readRecord(addr, recID)) {
            ++records;
            if (minRecID > recID) {
                minRecID = recID;
                minRecAddr = addr;
            }
            if (maxRecID < recID) {
                maxRecID = recID;
                maxRecAddr = addr;
            }
        } else {
            break;
        }
    }

    if (records == 0) {
        wAddr = rAddr = 0;
        can_write = true;
        return;
    }

    rAddr = maxRecAddr;
    if (records < (eLength / record_size)) {                                // The EEPROM is not full
        wAddr = rAddr + record_size;
        if (wAddr > eLength) wAddr = 0;
    } else {
        wAddr = minRecAddr;
    }
    can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
    memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
    memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
    updateConfig(Cfg);
    return save();                                                          // Save new data into the EEPROM
}

bool CONFIG::save(void) {
    if (!can_write) return can_write;
    if (nextRecID == 0) nextRecID = 1;

    uint16_t startWrite = wAddr;
    uint32_t nxt = nextRecID;
    uint8_t summ = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(startWrite++, nxt & 0xff);
        summ <<=2; summ += nxt;
        nxt >>= 8;
    }
    uint8_t* p = (byte *)&Config;
    for (uint8_t i = 0; i < sizeof(struct cfg); ++i) {
        summ <<= 2; summ += p[i];
        EEPROM.write(startWrite++, p[i]);
    }
    summ ++;                                                                // To avoid empty records
    EEPROM.write(wAddr+record_size-1, summ);

    rAddr = wAddr;
    wAddr += record_size;
    if (wAddr > EEPROM.length()) wAddr = 0;
    nextRecID ++;                                                           // Get ready to write next record
    return true;
}

bool CONFIG::load(void) {
    bool is_valid = readRecord(rAddr, nextRecID);
    nextRecID ++;
    return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
    uint8_t Buff[record_size];

    for (uint8_t i = 0; i < record_size; ++i) 
        Buff[i] = EEPROM.read(addr+i);
  
    uint8_t summ = 0;
    for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {
        summ <<= 2; summ += Buff[i];
    }
    summ ++;                                                                // To avoid empty fields
    if (summ == Buff[record_size-1]) {                                      // Checksumm is correct
        uint32_t ts = 0;
        for (char i = 3; i >= 0; --i) {
            ts <<= 8;
            ts |= Buff[byte(i)];
        }
        recID = ts;
        memcpy(&Config, &Buff[4], sizeof(struct cfg));
        return true;
    }
    return false;
}

//------------------------------------------ class HOT GUN CONFIG ----------------------------------------------
class HOTGUN_CFG : public CONFIG {
    public:
        HOTGUN_CFG()                                                        { }
        void     init(void);
        bool     isCold(uint16_t temp);                                     // Whether the HOT GUN temperature is low
        uint16_t tempPreset(void);                                          // The preset temperature in internal units
		uint8_t	 fanPreset(void);                                           // The preset fan speed 0 - 255 
        uint16_t tempInternal(uint16_t temp);                               // Translate the human readable temperature into internal value
        uint16_t tempHuman(uint16_t temp);                                  // Translate temperature from internal units to the Celsius
        void     save(uint16_t temp, uint8_t fanSpeed);                     // Save preset temperature in the internal units and fan speed
        void     applyCalibrationData(uint16_t tip[3]);
        void     getCalibrationData(uint16_t tip[3]);
        void     saveCalibrationData(uint16_t tip[3]);
        void     setDefaults(bool Write);                                   // Set default parameter values if failed to load data from EEPROM
    private:
        uint16_t t_tip[3];
        const   uint16_t def_tip[3] = {587, 751, 850};                      // Default values of internal sensor readings at reference temperatures
        const   uint16_t min_temp  = 50;
        const   uint16_t max_temp  = 900;
        const   uint16_t def_temp  = 600;                                   // Default preset temperature
        const   uint8_t  def_fan   = 64;                                  	// Default preset fan speed 0 - 255
        const   uint16_t ambient_temp = 0;
        const   uint16_t ambient_tempC= 25;
};

void HOTGUN_CFG::init(void) {
    CONFIG::init();
    if (!CONFIG::load()) setDefaults(false);                                // If failed to load the data from EEPROM, initialize the config data with the default values
    uint32_t   cd = Config.calibration;
    t_tip[0] = cd & 0x3FF; cd >>= 10;                                       // 10 bits per calibration parameter, because the ADC readings are 10 bits
    t_tip[1] = cd & 0x3FF; cd >>= 10;
    t_tip[2] = cd & 0x3FF;
    // Check the tip calibration is correct
    if ((t_tip[0] >= t_tip[1]) || (t_tip[1] >= t_tip[2])) {
        setDefaults(false);
        for (uint8_t i = 0; i < 3; ++i)
            t_tip[i] = def_tip[i];
    }
    return;
}
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout

bool HOTGUN_CFG::isCold(uint16_t temp) {
    return (temp < t_tip[0]) && (map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]) < 50);
}

uint16_t HOTGUN_CFG::tempPreset(void) {
    return Config.temp;
}

uint8_t HOTGUN_CFG::fanPreset(void) {
    return Config.fan;
}

uint16_t HOTGUN_CFG::tempInternal(uint16_t t) {                             // Translate the human readable temperature into internal value
    uint16_t temp = t;
    t = constrain(t, temp_minC, temp_maxC);
    if (t >= temp_tip[1])
        temp = map(t+1, temp_tip[1], temp_tip[2], t_tip[1], t_tip[2]);
    else
        temp = map(t+1, temp_tip[0], temp_tip[1], t_tip[0], t_tip[1]);
 
    for (uint8_t i = 0; i < 10; ++i) {
        uint16_t tH = tempHuman(temp);
        if (tH <= t) break;
        --temp;
    }
    return temp;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Fahrenheit)
uint16_t HOTGUN_CFG::tempHuman(uint16_t temp) {
    uint16_t tempH = 0;
    if (temp <= ambient_temp) {
        tempH = ambient_tempC;
    } else if (temp < t_tip[0]) {
        tempH = map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]);
    } else if (temp >= t_tip[1]) {
        tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1], temp_tip[2]);
    } else {
        tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0], temp_tip[1]);
    }
    return tempH;
}

void HOTGUN_CFG::save(uint16_t temp, uint8_t fanSpeed) {
    Config.temp        = constrain(temp, min_temp, max_temp);
    Config.fan         = fanSpeed;
    CONFIG::save();                                                         // Save new data into the EEPROM
}

void HOTGUN_CFG::applyCalibrationData(uint16_t tip[3]) {
    if (tip[0] < ambient_temp) {
        uint16_t t = ambient_temp + tip[1];
        tip[0] = t >> 1;
    }
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    if (tip[2] > max_temp) tip[2] = max_temp; 
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::getCalibrationData(uint16_t tip[3]) {
    tip[0] = t_tip[0];
    tip[1] = t_tip[1];
    tip[2] = t_tip[2];
}

void HOTGUN_CFG::saveCalibrationData(uint16_t tip[3]) {
    if (tip[2] > max_temp) tip[2] = max_temp;
    uint32_t cd = tip[2] & 0x3FF; cd <<= 10;                                // Pack tip calibration data in one 32-bit word: 10-bits per value
    cd |= tip[1] & 0x3FF; cd <<= 10;
    cd |= tip[0];
    Config.calibration = cd;
    t_tip[0] = tip[0];
    t_tip[2] = tip[1];
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::setDefaults(bool Write) {
    uint32_t c = def_tip[2] & 0x3FF; c <<= 10;
    c |= def_tip[1] & 0x3FF;         c <<= 10;
    c |= def_tip[0] & 0x3FF;
    Config.calibration = c;
    Config.temp        = def_temp;
    Config.fan         = def_fan;
    if (Write) {
        CONFIG::save();
    }
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
    BUZZER(byte buzzerP)  { buzzer_pin = buzzerP; }
    void init(void);
    void shortBeep(void)  { digitalWrite(buzzer_pin, HIGH); delay(80);  digitalWrite(buzzer_pin, LOW); }
    void lowBeep(void)    { digitalWrite(buzzer_pin, HIGH); delay(160); digitalWrite(buzzer_pin, LOW); }
    void doubleBeep(void) { digitalWrite(buzzer_pin, HIGH); delay(160); digitalWrite(buzzer_pin, LOW); delay(150);
                            digitalWrite(buzzer_pin, HIGH); delay(160); digitalWrite(buzzer_pin, LOW);
                          }
    void failedBeep(void) { digitalWrite(buzzer_pin, HIGH); delay(170); digitalWrite(buzzer_pin, LOW); delay(10);
                            digitalWrite(buzzer_pin, HIGH); delay(80);  digitalWrite(buzzer_pin, LOW); delay(100);
                            digitalWrite(buzzer_pin, HIGH); delay(80);  digitalWrite(buzzer_pin, LOW);
                          }
  private:
    byte buzzer_pin;
};

void BUZZER::init(void) {
    pinMode(buzzer_pin, OUTPUT);
    noTone(buzzer_pin);
}

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
class DSPL : protected LiquidCrystal_I2C {
    public:
        DSPL(void) : LiquidCrystal_I2C(0x27, 16, 2) { }
        void    init(void);
        void    clear(void)                                                 { LiquidCrystal_I2C::clear(); }
        void    tSet(uint16_t t, bool Celsius = true);                      // Show the preset temperature
        void    tCurr(uint16_t t);                                          // Show the current temperature
        void    tInternal(uint16_t t);                                      // Show the current temperature in internal units
        void    tReal(uint16_t t);                                          // Show the real temperature in Celsius in calibrate mode
        void    fanSpeed(uint8_t s);                                        // Show the fan speed
		void	appliedPower(uint8_t p, bool show_zero = true);			    // Show applied power (%)
        void    setupMode(uint8_t mode);
        void    msgON(void);                                                // Show message: "ON"
        void    msgOFF(void);
        void    msgReady(void);
        void    msgCold(void);
        void    msgFail(void);                                              // Show 'Fail' message
        void    msgTune(void);                                              // Show 'Tune' message
    private:
        bool 	full_second_line;                                           // Whether the second line is full with the message
		char 	temp_units;
        const   uint8_t custom_symbols[3][8] = {
                          { 0b00110,                                        // Degree
                            0b01001,
                            0b01001,
                            0b00110,
                            0b00000,
                            0b00000,
                            0b00000,
                            0b00000
                          },
                          { 0b00100,                                        // Fan sign
                            0b01100,
                            0b01100,
                            0b00110,
                            0b01011,
                            0b11001,
                            0b10000,
                            0b00000
                          },
                          { 0b00011,                                        // Power sign
                            0b00110,
                            0b01100,
                            0b11111,
                            0b00110,
                            0b01100,
                            0b01000,
                            0b10000
                          }
                        };
};

void DSPL::init(void) {
    LiquidCrystal_I2C::begin();
    LiquidCrystal_I2C::clear();
    for (uint8_t i = 0; i < 3; ++i)
        LiquidCrystal_I2C::createChar(i+1, (uint8_t *)custom_symbols[i]);
    full_second_line = false;
	temp_units = 'C';
}

void DSPL::tSet(uint16_t t, bool Celsius) {
    char buff[10];
	if (Celsius) {
		temp_units = 'C';
	} else {
		temp_units = 'F';
	}
    LiquidCrystal_I2C::setCursor(0, 0);
    sprintf(buff, "Set:%3d%c%c", t, (char)1, temp_units);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::tCurr(uint16_t t) {
    char buff[6];
    LiquidCrystal_I2C::setCursor(0, 1);
    if (t < 1000) {
        sprintf(buff, "%3d%c ", t, (char)1);
    } else {
        LiquidCrystal_I2C::print(F("xxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
    if (full_second_line) {
        LiquidCrystal_I2C::print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tInternal(uint16_t t) {
    char buff[6];
    LiquidCrystal_I2C::setCursor(0, 1);
    if (t < 1023) {
        sprintf(buff, "%4d ", t);
    } else {
        LiquidCrystal_I2C::print(F("xxxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
    if (full_second_line) {
        LiquidCrystal_I2C::print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tReal(uint16_t t) {
    char buff[6];
    LiquidCrystal_I2C::setCursor(11, 1);
    if (t < 1000) {
        sprintf(buff, ">%3d%c", t, (char)1);
    } else {
        LiquidCrystal_I2C::print(F("xxx"));
        return;
    }
    LiquidCrystal_I2C::print(buff);
}

void DSPL::fanSpeed(uint8_t s) {
    char buff[6];
    s = map(s, 0, 255, 0, 99);
    sprintf(buff, " %c%2d%c", (char)2, s, '%');
    LiquidCrystal_I2C::setCursor(11, 1);
    LiquidCrystal_I2C::print(buff);
}

void DSPL::appliedPower(uint8_t p, bool show_zero) {
	char buff[6];
	if (p > 99) p = 99;
    LiquidCrystal_I2C::setCursor(5, 1);
    if (p == 0 && !show_zero) {
        LiquidCrystal_I2C::print(F("     "));
    } else {
	    sprintf(buff, " %c%2d%c", (char)3, p, '%');
        LiquidCrystal_I2C::print(buff);
    }
}

void DSPL::setupMode(byte mode) {
    LiquidCrystal_I2C::clear();
    LiquidCrystal_I2C::print(F("setup"));
    LiquidCrystal_I2C::setCursor(1,1);
    switch (mode) {
        case 0:                                                             // tip calibrate
            LiquidCrystal_I2C::print(F("calibrate"));
            break;
        case 1:                                                             // tune
            LiquidCrystal_I2C::print(F("tune"));
            break;
        case 2:                                                             // save
            LiquidCrystal_I2C::print(F("save"));
            break;
        case 3:                                                             // cancel
            LiquidCrystal_I2C::print(F("cancel"));
            break;
        case 4:                                                             // set defaults
            LiquidCrystal_I2C::print(F("reset config"));
            break;
        default:
            break;
    }
}

void DSPL::msgON(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F("    ON"));
}

void DSPL::msgOFF(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F("   OFF"));
}


void DSPL::msgReady(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F(" Ready"));
}

void DSPL::msgCold(void) {
    LiquidCrystal_I2C::setCursor(10, 0);
    LiquidCrystal_I2C::print(F("  Cold"));
}

void DSPL::msgFail(void) {
    LiquidCrystal_I2C::setCursor(0, 1);
    LiquidCrystal_I2C::print(F(" -== Failed ==- "));
}

void DSPL::msgTune(void) {
    LiquidCrystal_I2C::setCursor(0, 0);
    LiquidCrystal_I2C::print(F("Tune"));
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
	public:
		HISTORY(void)                               						{ len = 0; }
		void     init(void)                         						{ len = 0; }
		uint16_t last(void);
		uint16_t top(void)                          						{ return queue[0]; }
		void     put(uint16_t item);                						// Put new entry to the history
		uint16_t average(void);                     						// calculate the average value
        float    dispersion(void);                                          // calculate the math dispersion
	private:
		volatile uint16_t queue[H_LENGTH];
		volatile byte len;                          						// The number of elements in the queue
		volatile byte index;                        						// The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
	if (len < H_LENGTH) {
		queue[len++] = item;
	} else {
		queue[index ] = item;
		if (++index >= H_LENGTH) index = 0;         						// Use ring buffer
	}
}

uint16_t HISTORY::last(void) {
    if (len == 0) return 0;
	uint8_t i = len - 1;
	if (index)
		i = index - 1;
	return queue[i];
}

uint16_t HISTORY::average(void) {
	uint32_t sum = 0;
    if (len == 0) return 0;
	if (len == 1) return queue[0];
	for (uint8_t i = 0; i < len; ++i) sum += queue[i];
	sum += len >> 1;                              							// round the average
	sum /= len;
	return uint16_t(sum);
}

float HISTORY::dispersion(void) {
    if (len < 3) return 1000;
    uint32_t sum = 0;
    uint32_t avg = average();
    for (uint8_t i = 0; i < len; ++i) {
        long q = queue[i];
        q -= avg;
        q *= q;
        sum += q;
    }
    sum += len << 1;
    float d = (float)sum / (float)len;
    return d;
}

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algorithm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formula is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 *  
 *  PID coefficients history:
 *  10/14/2017  [768, 32, 328]
 */
class PID {
    public:
        PID(void) {
            Kp = 638;
            Ki = 196;
            Kd =   1;
        }
        void resetPID(int temp = -1);                                       // reset PID algorithm history parameters
        // Calculate the power to be applied
        long reqPower(int temp_set, int temp_curr);
        int  changePID(uint8_t p, int k);                                   // set or get (if parameter < 0) PID parameter
    private:
        void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
        int   temp_h0, temp_h1;                                             // previously measured temperature
        bool  pid_iterate;                                                  // Whether the iterative process is used
        long  i_summ;                                                       // Ki summary multiplied by denominator
        long  power;                                                        // The power iterative multiplied by denominator
        long  Kp, Ki, Kd;                                                   // The PID algorithm coefficients multiplied by denominator
        const byte denominator_p = 11;                                      // The common coefficient denominator power of 2 (11 means divide by 2048)
};

void PID::resetPID(int temp) {
    temp_h0 = 0;
    power  = 0;
    i_summ = 0;
    pid_iterate = false;
    if ((temp > 0) && (temp < 1000))
        temp_h1 = temp;
    else
        temp_h1 = 0;
}

int PID::changePID(uint8_t p, int k) {
    switch(p) {
        case 1:
            if (k >= 0) Kp = k;
            return Kp;
        case 2:
            if (k >= 0) Ki = k;
            return Ki;
        case 3:
            if (k >= 0) Kd = k;
            return Kd;
        default:
        break;
    }
    return 0;
}

long PID::reqPower(int temp_set, int temp_curr) {
    if (temp_h0 == 0) {
        // When the temperature is near the preset one, reset the PID and prepare iterative formula                        
        if ((temp_set - temp_curr) < 30) {
            if (!pid_iterate) {
                pid_iterate = true;
                power = 0;
                i_summ = 0;
            }
        }
        i_summ += temp_set - temp_curr;                                     // first, use the direct formula, not the iterate process
        power = Kp*(temp_set - temp_curr) + Ki*i_summ;
    // If the temperature is near, prepare the PID iteration process
    } else {
        long kp = Kp * (temp_h1 - temp_curr);
        long ki = Ki * (temp_set - temp_curr);
        long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
        long delta_p = kp + ki + kd;
        power += delta_p;                                                   // power kept multiplied by denominator!
    }
    if (pid_iterate) temp_h0 = temp_h1;
    temp_h1 = temp_curr;
    long pwr = power + (1 << (denominator_p-1));                            // prepare the power to delete by denominator, round the result
    pwr >>= denominator_p;                                                  // delete by the denominator
    return pwr;
}

//--------------------- High frequency PWM signal calss on D9 pin ------------------------- ---------------
class FastPWM_D9 {
    public:
        FastPWM_D9()                                { }
        void init(void);
        void duty(uint8_t d)                        { OCR1A = d; }
};

void FastPWM_D9::init(void) {
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    noInterrupts();
    TCNT1   = 0;
    TCCR1B  = _BV(WGM13);                           // set mode as phase and frequency correct pwm, stop the timer
    TCCR1A  = 0;
    ICR1    = 256;
    TCCR1B  = _BV(WGM13) | _BV(CS10);               // Top value = ICR1, prescale = 1
    TCCR1A |= _BV(COM1A1);                          // XOR D9 on OCR1A, detached from D10
    OCR1A   = 0;                                    // Switch-off the signal on pin 9;
    interrupts();
}

//--------------------- Hot air gun manager using total sine shape to power on the hardware ---------------
class HOTGUN : public PID {
    public:
        HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin);
        void        init(void);
		bool		isOn(void)												{ return on; }
		void		setTemp(uint16_t t)										{ temp_set = t; }
		uint16_t	getTemp(void)											{ return temp_set; }
		uint16_t	getCurrTemp(void)										{ return h_temp.last(); }
		uint16_t 	tempAverage(void)                  						{ return h_temp.average(); }
        uint8_t     powerAverage(void)                                      { return h_power.average(); }
		uint8_t     appliedPower(void)                						{ return actual_power; }
		void		setFanSpeed(uint8_t f)									{ fan_speed = f; if (on) hg_fan.duty(f); }
		uint8_t	    getFanSpeed(void)   									{ return fan_speed; }
        uint16_t    tempDispersion(void)                                    { return h_temp.dispersion(); }
        void        switchPower(bool On);
        void        fixPower(uint8_t Power);                                // Set the specified power to the the hot gun
		void     	keepTemp(void);
        bool        areExternalInterrupts(void)                             { return millis() - last_period < period * 10; }
        uint8_t     getMaxFixedPower(void)                                  { return period; }
        bool        syncCB(void);											// Return true at the end of the power period
    private:
        FastPWM_D9  hg_fan;
		long     	power;                             						// The hot air gun power, calculated by the PID algorithm
		uint16_t	temp_set;												// The preset temperature of the hot air gun (internal units)
		uint16_t	temp_curr;												// The current temperature of the hot air gun
		uint8_t		fan_speed;
        uint8_t     sen_pin;
		uint8_t		gun_pin;
		HISTORY  	h_power;                           						// The history queue of power applied values
		HISTORY  	h_temp;                            						// The history queue of the temperature
		volatile    uint8_t     cnt;
        volatile    uint8_t     actual_power;
        volatile    bool        active;
        bool        on, fan, fix_power;
        bool        chill;                                                  // To chill the hot gun
        uint32_t    last_period;                                            // The time in ms when the counter reset
        const       uint8_t     period 			= 100;
		const		uint8_t		min_fan_speed	= 30;
        const       uint16_t    temp_gun_cold   = 80;                       // The temperature of the cold iron 
};

HOTGUN::HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin) {
    sen_pin = HG_sen_pin;
	gun_pin	= HG_pwr_pin;
}

void HOTGUN::init(void) {
    cnt         = 0;
    power       = 0;
    active      = false;
    on          = false;
    fan         = false;
    fix_power   = false;
    chill       = false;
    last_period = 0;
    pinMode(sen_pin, INPUT);
    pinMode(gun_pin, OUTPUT);
	digitalWrite(gun_pin, LOW);
    hg_fan.init();
	h_temp.init();
    resetPID();
}

bool HOTGUN::syncCB(void) {
    if (++cnt >= period) {
        cnt = 0;
        last_period = millis();                                             // Save the current time to check the external interrupts
        if (!active && (actual_power > 0)) {
            digitalWrite(gun_pin, HIGH);
            active = true;
        }
    } else if (cnt >= actual_power) {
        if (active) {
            digitalWrite(gun_pin, LOW);
            active = false;
        }
    }
	return (cnt == 0);														// End of the Power period (period AC voltage shapes)
}

void HOTGUN::switchPower(bool On) {
	on = On;
	if (!on) {
		digitalWrite(gun_pin, LOW);
        actual_power = 0;
	} else {
	    if (fan_speed < min_fan_speed)
		    fan_speed = min_fan_speed;
		hg_fan.duty(fan_speed);
        fan = true;
	}
}

// This routine is used to keep the hot air gun temperature near required value
void HOTGUN::keepTemp(void) {

	uint16_t temp = analogRead(sen_pin);             						// Check the hot air gun temperature

    h_temp.put(temp);
    if (!chill && on && temp > temp_set + 20) {
        digitalWrite(gun_pin, LOW);
        actual_power = 0;
        chill = true;
    }

	if (on) {
		if (chill) {
			if (temp < (temp_set - 8)) {
				chill = false;
				resetPID();
			} else {
				power = 0;
				actual_power = 0;
                return;
			}
		}
		power = reqPower(temp_set, temp);           						// Use PID algorithm to calculate power to be applied
		actual_power = constrain(power, 0, period);
		h_power.put(actual_power);
	} else {
        if (!fix_power) {
		    actual_power = 0;
		    digitalWrite(gun_pin, LOW);
        }
	}

    // Keep fan running till the hot gun become cold
    if (fan) {
        if ((actual_power == 0) && temp <= temp_gun_cold) {                // Switch off the fan when the gun become cold
            hg_fan.duty(0);
            fan = false;
        }
    } else {
        if (temp > temp_gun_cold) {
            hg_fan.duty(fan_speed);
            fan = true;
        }
    }
}

void HOTGUN::fixPower(uint8_t Power) {
    if (Power == 0) {                                                       // To switch off the hot gun, set the Power to 0
        fix_power = false;
        actual_power = 0;
        return;
    }

    if (Power > period) Power = period;
    power = Power;
    actual_power = power;
    fix_power = true;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
	public:
		SCREEN* next;                               						// Pointer to the next screen
		SCREEN() {
			next			= 0;
			update_screen  	= 0;
			scr_timeout    	= 0;
			time_to_return 	= 0;
		}
		virtual void    init(void)                     						{ }
		virtual SCREEN* show(void)                  						{ return this; }
		virtual SCREEN* menu(void)                  						{ return this; }
		virtual SCREEN* menu_long(void)             						{ if (this->next != 0)  return this->next;  else return this; }
        virtual SCREEN* reedSwitch(bool on)                                 { return this; }    
		virtual void    rotaryValue(int16_t value)     						{ }
		void            forceRedraw(void)                   				{ update_screen = 0; }
	protected:
		uint32_t update_screen;                     						// Time in ms when the screen should be updated
		uint32_t scr_timeout;                       						// Timeout is sec. to return to the main screen, canceling all changes
		uint32_t time_to_return;                    						// Time in ms to return to main screen
};

//---------------------------------------- class mainSCREEN [the hot air gun is OFF] ---------------------------
class mainSCREEN : public SCREEN {
	public:
		mainSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
			pHG 	= HG;
			pD      = DSP;
			pEnc    = ENC;
			pBz     = Buzz;
			pCfg    = Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual SCREEN* menu(void);
        virtual SCREEN* reedSwitch(bool on);
		virtual void	rotaryValue(int16_t value); 						// Setup the preset temperature
        SCREEN*     on;                                                     // Screen mode when the power is
	private:
		HOTGUN*		pHG;                            						// Pointer to the hot air gun instance
		DSPL*     	pD;                               						// Pointer to the DSPLay instance
		ENCODER*	pEnc;                             						// Pointer to the rotary encoder instance
		BUZZER*   	pBz;                              						// Pointer to the simple buzzer instance
		HOTGUN_CFG* pCfg;                             						// Pointer to the configuration instance
		uint32_t  	clear_used_ms;                    						// Time in ms when used flag should be cleared (if > 0)
		bool		mode_temp;												// Preset mode: change temperature or change fan speed
		bool      	used;                             						// Whether the IRON was used (was hot)
		bool      	cool_notified;                    						// Whether there was cold notification played
		const uint16_t period 				= 1000;               			// The period to update the screen
		const uint32_t cool_notify_period 	= 120000; 						// The period to display 'cool' message (ms)
		const uint16_t show_temp 			= 20000;           				// The period to show the preset temperature (ms)
};

void mainSCREEN::init(void) {
	pHG->switchPower(false);
	uint16_t temp_set 	= pHG->getTemp();
	uint16_t tempH 	    = pCfg->tempHuman(temp_set);         				// The preset temperature in the human readable units
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
	used = !pCfg->isCold(pHG->tempAverage());
	cool_notified = !used;
	if (used) {                                   							// the hot gun was used, we should save new data in EEPROM
		pCfg->save(temp_set, pHG->getFanSpeed());
	}
	mode_temp = true;
	clear_used_ms = 0;
    pD->clear();
	forceRedraw();
}

void mainSCREEN::rotaryValue(int16_t value) {
	if (mode_temp) {														// set hot gun temperature
		uint16_t temp = pCfg->tempInternal(value);
		pHG->setTemp(temp);
		pD->tSet(value);
	} else {																// set fan speed
		pHG->setFanSpeed(value);
		pD->fanSpeed(value);
	}
	update_screen  = millis() + period;
}

SCREEN* mainSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;

	if (clear_used_ms && (millis() > clear_used_ms)) {
		clear_used_ms = 0;
		used = false;
	}

    uint16_t temp_set = pHG->getTemp();
    pD->tSet(pCfg->tempHuman(temp_set));
	uint16_t temp  = pHG->tempAverage();
	uint16_t tempH = pCfg->tempHuman(temp);
	if (pCfg->isCold(temp)) {
		if (used) {
			pD->msgCold();
		} else {
			pD->msgOFF();
		}
		if (used && !cool_notified) {
		    pBz->lowBeep();
		    cool_notified = true;
		    clear_used_ms = millis() + cool_notify_period;
		}
	} else {
        pD->msgOFF();
	}
	pD->tCurr(tempH);
    pD->appliedPower(0, false);
    pD->fanSpeed(pHG->getFanSpeed());
	return this;
}

SCREEN* mainSCREEN::menu(void) {
	if (mode_temp) {                                                        // Prepare to adjust the fan speed
		uint8_t	fs = pHG->getFanSpeed();
		pEnc->reset(fs, 0, 255, 5, 20);
		mode_temp = false;
	} else {                                                                // Prepare to adjust the preset temperature
		uint16_t temp_set   = pHG->getTemp();
		uint16_t tempH 	    = pCfg->tempHuman(temp_set);
		pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
		mode_temp = true;
	}
    return this;
}

SCREEN* mainSCREEN::reedSwitch(bool on) {
    if (on && this->on)
        return this->on;
    return this; 
}

//---------------------------------------- class workSCREEN [the hot air gun is ON] ----------------------------
class workSCREEN : public SCREEN {
	public:
		workSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
			update_screen = 0;
			pHG 	= HG;
			pD    	= DSP;
			pBz   	= Buzz;
			pEnc  	= Enc;
			pCfg  	= Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual SCREEN* menu(void);
        virtual SCREEN* reedSwitch(bool on);
		virtual void    rotaryValue(int16_t value); 						// Change the preset temperature
	private:
		HOTGUN*     pHG;                            						// Pointer to the IRON instance
		DSPL*     	pD;                               						// Pointer to the DSPLay instance
		BUZZER*   	pBz;                              						// Pointer to the simple Buzzer instance
		ENCODER*  	pEnc;                             						// Pointer to the rotary encoder instance
		HOTGUN_CFG* pCfg;                             						// Pointer to the configuration instance
		bool      	ready;                            						// Whether the IRON have reached the preset temperature
		bool		mode_temp;												// Preset mode: temperature or fan speed
		const uint16_t period = 1000;               						// The period to update the screen (ms) 
};

void workSCREEN::init(void) {
	uint8_t fs = pHG->getFanSpeed();
    pEnc->reset(fs, 0, 255, 5, 20);
    mode_temp   = false;                                                    // By default adjust the fan speed
	pHG->switchPower(true);
	ready = false;
	pD->clear();
	forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {   							// Setup new preset temperature by rotating the encoder
	if (mode_temp) {
        ready = false;
		uint16_t temp = pCfg->tempInternal(value);      				    // Translate human readable temperature into internal value
		pHG->setTemp(temp);
		pD->tSet(value);
	} else {
		pHG->setFanSpeed(value);
		pD->fanSpeed(value);
	}
	update_screen = millis() + period;
}

SCREEN* workSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;

    int temp_set  = pHG->getTemp();
    int tempH_set = pCfg->tempHuman(temp_set);
    pD->tSet(tempH_set);
    int temp      = pHG->tempAverage();
    int tempH     = pCfg->tempHuman(temp);
    pD->tCurr(tempH);
    pD->msgON();
	uint8_t p 	= pHG->appliedPower();
	pD->appliedPower(p);
    pD->fanSpeed(pHG->getFanSpeed());

    
Serial.print("Diff = "); Serial.print(temp_set - temp);
uint32_t disp = pHG->tempDispersion();
Serial.print(", Dispersion = "); Serial.println(disp);

    if ((abs(temp_set - temp) < 5) && (pHG->tempDispersion() <= 20))  {
        if (!ready) {
            pBz->shortBeep();
            ready = true;
            pD->msgReady();
            Serial.println("Ready");
            update_screen = millis() + (period << 2);
            return this;
        }
    }
	return this;
}

SCREEN* workSCREEN::menu(void) {
	if (mode_temp) {
		uint8_t	fs = pHG->getFanSpeed();
		pEnc->reset(fs, 0, 255, 5, 20);
		mode_temp = false;
	} else {
		uint16_t temp_set   = pHG->getTemp();
		uint16_t tempH 	    = pCfg->tempHuman(temp_set);
		pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
		mode_temp = true;
	}
    return this;
}

SCREEN* workSCREEN::reedSwitch(bool on) {
    if (!on && next)
        return next;
    return this; 
}

//---------------------------------------- class errorSCREEN [the error detected] ------------------------------
class errorSCREEN : public SCREEN {
	public:
		errorSCREEN(HOTGUN* HG, DSPL* DSP, BUZZER* Buzz) {
			pHG 	= HG;
			pD    	= DSP;
			pBz   	= Buzz;
		}
		virtual void init(void)                                             { pHG->switchPower(false); pD->clear(); pD->msgFail(); pBz->failedBeep(); }
        virtual SCREEN* menu(void)                                          { if (this->next != 0)  return this->next;  else return this; }
	private:
		HOTGUN*    	pHG;                             						// Pointer to the got air gun instance
		DSPL*    	pD;                                						// Pointer to the display instance
		BUZZER*  	pBz;                               						// Pointer to the simple Buzzer instance
};

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
    public:
        configSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual SCREEN* menu(void);
        virtual void    rotaryValue(int16_t value);
        SCREEN*         calib;                                              // Pointer to the calibration SCREEN
        SCREEN*         tune;                                               // Pointer to the tune SCREEN
    private:
        HOTGUN*     pHG;                                                    // Pointer to the HOTGUN instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the config instance
        uint8_t     mode;                                                   // 0 - hotgun calibrate, 1 - tune, 2 - save, 3 - cancel, 4 - defaults
        const uint16_t period = 10000;                                      // The period in ms to update the screen
};

void configSCREEN::init(void) {
    pHG->switchPower(false);
    mode = 0;
    pEnc->reset(mode, 0, 4, 1, 0, true);          
    pD->clear();
    pD->setupMode(0);
    this->scr_timeout = 30;                                                 // This variable is defined in the superclass
}

SCREEN* configSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    pD->setupMode(mode);
    return this;
}

SCREEN* configSCREEN::menu(void) {
    switch (mode) {
        case 0:                                                             // calibrate hotgun
            if (calib) return calib;
            break;
        case 1:                                                             // Tune potentiometer
            if (tune) return tune;
            break;
        case 2:                                                             // Save configuration data
            menu_long();
        case 3:                                                             // Cancel, Return to the main menu
            if (next) return next;
            break;
        case 4:                                                             // Save defaults
            pCfg->setDefaults(true);
            if (next) return next;
            break;
    }
    forceRedraw();
    return this;
}

void configSCREEN::rotaryValue(int16_t value) {
    mode = value;
    forceRedraw();
}

//---------------------------------------- class calibSCREEN [ tip calibration ] -------------------------------
class calibSCREEN : public SCREEN {
    public:
        calibSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
            pBz     = Buzz;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
    private:
        uint16_t        selectTemp(byte index);                             // Calculate the value of the temperature limit depending on mode
        void            buildCalibration(uint16_t tip[3]);
        HOTGUN*         pHG;                                                // Pointer to the HOTGUN instance
        DSPL*           pD;                                                 // Pointer to the DSPLay instance
        ENCODER*        pEnc;                                               // Pointer to the rotary encoder instance
        HOTGUN_CFG*     pCfg;                                               // Pointer to the config instance
        BUZZER*         pBz;                                                // Pointer to the buzzer instance
        uint8_t         mode;                                               // Which parameter to change: t_min, t_mid, t_max
        uint16_t        calib_temp[2][3];                                   // Calibration temperature data measured at each of calibration points (Celsius, internal temp)
        uint16_t        preset_temp;                                        // The preset temp in human readable units
        bool            ready;                                              // Whether the temperature has been established
        bool            tune;                                               // Whether the parameter is modifiying
        const uint32_t  period   = 1000;                                    // Update screen period
        const uint16_t  temp_max = 900;
};

void calibSCREEN::init(void) {
    mode = 0;
    pEnc->reset(mode, 0, 2, 1, 0, true);                                    // Select the reference temperature: 0 - temp_tip[0], 1 - temp_tip[1], 2 - temp_tip[2]
    pHG->switchPower(false);
    tune  = false;
    ready = false;
    for (uint8_t i = 0; i < 3; ++i)
        calib_temp[0][i] = temp_tip[i];
    pCfg->getCalibrationData(&calib_temp[1][0]);
    pD->clear();
    pD->msgOFF();
    uint16_t temp = temp_tip[mode];
    preset_temp = pHG->getTemp();                                           // Preset Temp in internal units
    preset_temp = pCfg->tempHuman(preset_temp);                             // Save the preset temperature in Celsius
    uint16_t temp_set = pCfg->tempInternal(temp);
    pHG->setTemp(temp_set);
    forceRedraw();
}

SCREEN* calibSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen       = millis() + period;
    int temp            = pHG->tempAverage();                               // The Hot gun average value of the current temp. (internal)
    int temp_set        = pHG->getTemp();                                   // The preset 
    uint16_t tempH      = pCfg->tempHuman(temp);
    uint16_t temp_setH  = pCfg->tempHuman(temp_set);
    pD->tSet(temp_setH);
    pD->tCurr(tempH);

    uint8_t p = pHG->appliedPower();
    if (!pHG->isOn()) p = 0;
    pD->appliedPower(p);
    if (tune && (abs(temp_set - temp) < 5) && (pHG->tempDispersion() <= 20) && (p > 1))  {
        if (!ready) {
            pBz->shortBeep();
            pD->msgReady();
            ready = true;
            }
    }
    if (ready) {
        pD->tReal(pEnc->read());
    } else {
        if (pHG->isOn())
            pD->fanSpeed(pHG->getFanSpeed());
    }
    if (tune && !pHG->isOn()) {                                             // The hot gun was switched off by error
        pD->msgOFF();
        tune  = false;
        ready = false;
    }
    return this;
}

void calibSCREEN::rotaryValue(int16_t value) {
    update_screen = millis() + period;
    if (!tune) {                                                            // select the temperature to be calibrated, t_min, t_mid or t_max
        mode = value;
        if (mode > 2) mode = 2;
        uint16_t temp = temp_tip[mode];
        temp = pCfg->tempInternal(temp);
        pHG->setTemp(temp);
    }
    forceRedraw();
}

SCREEN* calibSCREEN::menu(void) { 
    if (tune) {                                                             // Calibrated value for the temperature limit jus has been setup
        tune = false;
        calib_temp[0][mode] = pEnc->read();                                 // Real temperature (Celsius)
        calib_temp[1][mode] = pHG->tempAverage();                           // The temperature on the hot gun
        pHG->switchPower(false);
        pD->msgOFF();
        pEnc->reset(mode, 0, 2, 1, 0, true);                                // The temperature limit has been adjusted, switch to select mode
        uint16_t tip[3];
        buildCalibration(tip);
        pCfg->applyCalibrationData(tip);
    } else {                                                                // Calibration point selected
        tune = true;
        uint16_t temp = temp_tip[mode];
        pEnc->reset(temp, 40, 600, 1, 5);
        temp = pCfg->tempInternal(temp);
        pHG->setTemp(temp);
        pHG->switchPower(true);
        pD->msgON();
    }
    ready = false;
    forceRedraw();
    return this;
}

SCREEN* calibSCREEN::menu_long(void) {
    pHG->switchPower(false);
    // temp_tip - array of calibration temperatures in Celsius
    uint16_t tip[3];
    buildCalibration(tip);
    pCfg->saveCalibrationData(tip);
    uint8_t fan = pHG->getFanSpeed();
    pCfg->save(preset_temp, fan);
    uint16_t temp = pCfg->tempInternal(preset_temp);
    pHG->setTemp(temp);
    if (next) return next;
    return this;
}

/*
 * Calculate hot gun calibration parameters using linear approximation by Ordinary Least Squares method
 * Y = a * X + b, where
 * Y - internal temperature, X - real temperature. a and b are double coefficients
 * a = (N * sum(Xi*Yi) - sum(Xi) * sum(Yi)) / ( N * sum(Xi^2) - (sum(Xi))^2)
 * b = 1/N * (sum(Yi) - a * sum(Xi))
 * N = 3 (3 reference temperature points are used)
 */
void calibSCREEN::buildCalibration(uint16_t tip[3]) {
    long sum_XY = 0;                                                        // sum(Xi * Yi)
    long sum_X  = 0;                                                        // sum(Xi)
    long sum_Y  = 0;                                                        // sum(Yi)
    long sum_X2 = 0;                                                        // sum(Xi^2)

    for (uint8_t i = 0; i < 3; ++i) {
        uint16_t X  = calib_temp[0][i];
        uint16_t Y  = calib_temp[1][i];
        sum_XY  += X * Y;
        sum_X   += X;
        sum_Y   += Y;
        sum_X2  += X * X;
    }

    double a = (double)(3 * sum_XY - sum_X * sum_Y) / (double)(3 * sum_X2 - sum_X * sum_X);
    double b = ((double)sum_Y - a * (double)sum_X) / 3.0;

    for (uint8_t i = 0; i < 3; ++i) {
        double temp = a * (double)temp_tip[i] + b;
        tip[i] = round(temp);
    }
    if (tip[2] > temp_max) tip[2] = temp_max;                               // Maximal possible temperature
}

//---------------------------------------- class tuneSCREEN [tune the potentiometer ] --------------------------
class tuneSCREEN : public SCREEN {
    public:
        tuneSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pBz     = Buzz;
        }
        virtual void    init(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
    private:
        HOTGUN*     pHG;                                                    // Pointer to the IRON instance
        DSPL*       pD;                                                     // Pointer to the display instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        BUZZER*     pBz;                                                    // Pointer to the simple Buzzer instance
        bool        on;                                                     // Wether the power is on
        uint32_t    heat_ms;                                                // Time in ms when power was on
        uint8_t     max_power;                                              // Maximum possible power to be applied
        const uint16_t period = 500;                                        // The period in ms to update the screen
};

void tuneSCREEN::init(void) {
    pHG->switchPower(false);
    max_power = pHG->getMaxFixedPower();
    pEnc->reset(max_power >> 2, 0, max_power, 1, 2);                        // Rotate the encoder to change the power supplied
    on = false;
    heat_ms = 0;
    pD->clear();
    pD->msgTune();
    pD->msgOFF();
    forceRedraw();
}

void tuneSCREEN::rotaryValue(int16_t value) {
    if (on) {
        heat_ms = millis();
        pHG->fixPower(value);
    }
    forceRedraw();
}

SCREEN* tuneSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    uint16_t temp   = pHG->getCurrTemp();
    uint8_t  power  = pHG->appliedPower();
    if (!on) {
        power = pEnc->read();
    }
    pD->tInternal(temp);
    pD->appliedPower(power);
    if (heat_ms && ((millis() - heat_ms) > 3000) && (pHG->tempDispersion() < 10) && (power > 1)) {
        pBz->shortBeep();
        heat_ms = 0;
    }
    return this;
}
  
SCREEN* tuneSCREEN::menu(void) {                                            // The rotary button pressed
    if (on) {
        pHG->fixPower(0);
        on = false;
        pD->msgOFF();
    } else {
        on = true;
        heat_ms = millis();
        uint8_t power = pEnc->read();                                       // applied power
        pHG->fixPower(power);
        pD->msgON();
    }
    return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
    pHG->fixPower(0);                                                       // switch off the power
    pHG->switchPower(false);
    if (next) return next;
    return this;
}

//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
	public:
		pidSCREEN(HOTGUN* HG, ENCODER* ENC) {
			pHG 	= HG;
			pEnc  	= ENC;
		}
		virtual void    init(void);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
	private:
		void     	showCfgInfo(void);                 						// show the main config information: Temp set, fan speed and PID coefficients
		HOTGUN*		pHG;                             						// Pointer to the IRON instance
		ENCODER* 	pEnc;                              						// Pointer to the rotary encoder instance
		uint8_t     mode;                              						// Which parameter to tune [0-5]: select element, Kp, Ki, Kd, temp, speed
		uint32_t 	update_screen;                     						// Time in ms when to print thee info
		int      	temp_set;
		const uint16_t period = 1100;
};

void pidSCREEN::init(void) {
	temp_set = pHG->getTemp();
	mode = 0;                                     							// select the element from the list
	pEnc->reset(1, 1, 5, 1, 1, true);             							// 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp, 5 - fan
	showCfgInfo();
	Serial.println("");
}

void pidSCREEN::rotaryValue(int16_t value) {
	if (mode == 0) {                              							// select element from the menu
		showCfgInfo();
		switch (value) {
			case 1:
				Serial.println("Kp");
				break;
			case 2:
				Serial.println("Ki");
				break;
			case 4:
				Serial.println(F("Temp"));
				break;
			case 5:
				Serial.println(F("Fan"));
                break;
			case 3:
			default:
				Serial.println("Kd");
			break;
		}
	} else {
		switch (mode) {
			case 1:
				Serial.print(F("Kp = "));
				pHG->changePID(mode, value);
				break;
			case 2:
				Serial.print(F("Ki = "));
				pHG->changePID(mode, value);
				break;
			case 4:
				Serial.print(F("Temp = "));
				temp_set = value;
				pHG->setTemp(value);
				break;
			case 5:
				Serial.print(F("Fan Speed = "));
				pHG->setFanSpeed(value);
				break;
			case 3:
			default:
				Serial.print(F("Kd = "));
				pHG->changePID(mode, value);
				break;
		}
		Serial.println(value);
	}
}

SCREEN* pidSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;
	if (pHG->isOn()) {
		char buff[80];
		int		 temp   = pHG->getCurrTemp();
		uint8_t	 pwr 	= pHG->powerAverage();
		uint8_t  fs		= pHG->getFanSpeed();
		fs = map(fs, 0, 255, 0, 100);
		sprintf(buff, "%3d: power = %3d%c, fan = %3d;", temp_set - temp, pwr, '%', fs);
		Serial.println(buff);
	}
	return this;
}
SCREEN* pidSCREEN::menu(void) {                 							// The encoder button pressed
	if (mode == 0) {                              							// select upper or lower temperature limit
		mode = pEnc->read();
		if (mode > 0 && mode < 4) {
			int k = pHG->changePID(mode, -1);
			pEnc->reset(k, 0, 10000, 1, 10);
		} else if (mode == 4) {
			pEnc->reset(temp_set, 0, 970, 1, 5);
		} else {
			pEnc->reset(pHG->getFanSpeed(), 0, 250, 5, 20);
		}
	} else {    
		mode = 0;
		pEnc->reset(1, 1, 5, 1, 1, true);           						// 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp, 5 - fan speed
	}
	return this;
}

SCREEN* pidSCREEN::menu_long(void) {
	bool on = pHG->isOn();
	pHG->switchPower(!on);
	if (on)
		Serial.println(F("The air gun is OFF"));
	else
		Serial.println(F("The air gun is ON"));
  return this;
}

void pidSCREEN::showCfgInfo(void) {
	Serial.print(F("Temp set: "));
	Serial.print(temp_set, DEC);
	Serial.print(F(", fan speed = "));
	Serial.print(pHG->getFanSpeed());
	Serial.print(F(", PID: ["));
	for (byte i = 1; i < 4; ++i) {
		int k = pHG->changePID(i, -1);
		Serial.print(k, DEC);
		if (i < 3) Serial.print(", ");
	}
	Serial.print("]; ");
}

//=========================================================================================================
HOTGUN 		hg(TEMP_GUN_PIN, HOT_GUN_PIN);
DSPL       	disp;
ENCODER    	rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON     	rotButton(R_BUTN_PIN);
SWITCH      reedSwitch(REED_SW_PIN);
HOTGUN_CFG 	hgCfg;
BUZZER     	simpleBuzzer(BUZZER_PIN);

mainSCREEN   offScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
workSCREEN   wrkScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
configSCREEN cfgScr(&hg,  &disp, &rotEncoder, &hgCfg);
calibSCREEN  clbScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
tuneSCREEN   tuneScr(&hg, &disp, &rotEncoder, &simpleBuzzer);
errorSCREEN  errScr(&hg,  &disp, &simpleBuzzer);
pidSCREEN    pidScr(&hg,  &rotEncoder);

SCREEN 	*pCurrentScreen = &offScr;

volatile bool	end_of_power_period = false;

void syncAC(void) {
    end_of_power_period = hg.syncCB();
}

void rotEncChange(void) {
	rotEncoder.changeINTR();
}

void setup() {
	Serial.begin(115200);
	disp.init();

	// Load configuration parameters
	hgCfg.init();
	hg.init();
	uint16_t temp 	= hgCfg.tempPreset();
	uint8_t  fan	= hgCfg.fanPreset();
	hg.setTemp(temp);
	hg.setFanSpeed(fan);

    reedSwitch.init(500, 3000);

	// Initialize rotary encoder
	rotEncoder.init();
	rotButton.init();
	delay(500);
	attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(AC_SYNC_PIN), syncAC, RISING);

	// Initialize SCREEN hierarchy
	offScr.next     = &cfgScr;
    offScr.on       = &wrkScr;
    wrkScr.next     = &offScr;
    cfgScr.next     = &offScr;
    cfgScr.calib    = &clbScr;
    cfgScr.tune     = &tuneScr;
    clbScr.next     = &offScr;
    tuneScr.next    = &offScr;
	errScr.next     = &offScr;

    pCurrentScreen->init();
}

void loop() {
    static bool     reset_encoder   = true;
	static int16_t  old_pos 	    = 0;
	static uint32_t ac_check 	    = 5000;

  
	int16_t pos = rotEncoder.read();
    if (reset_encoder) {
        old_pos = pos;
        reset_encoder = false;
    } else {
	    if (old_pos != pos) {
		    pCurrentScreen->rotaryValue(pos);
		    old_pos = pos;
	    }
    }

    SCREEN* nxt = pCurrentScreen->reedSwitch(reedSwitch.status());
    if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
        reset_encoder = true;
        return;
    }
    
	uint8_t bStatus = rotButton.buttonCheck();
	switch (bStatus) {
		case 2:                                     						// long press;
			nxt = pCurrentScreen->menu_long();
			if (nxt != pCurrentScreen) {
				pCurrentScreen = nxt;
				pCurrentScreen->init();
                reset_encoder = true;
			}
			break;
		case 1:                                     						// short press
			nxt = pCurrentScreen->menu();
			if (nxt != pCurrentScreen) {
				pCurrentScreen = nxt;
				pCurrentScreen->init();
                reset_encoder = true;
			}
			break;
		case 0:                                     						// Not pressed
		default:
			break;
	}

	nxt = pCurrentScreen->show();
	if (nxt && pCurrentScreen != nxt) {           							// Be paranoiac, the returned value must not be null
		pCurrentScreen = nxt;
		pCurrentScreen->init();
        reset_encoder = true;
	}
	
	if (end_of_power_period) {												// Calculate the required power
		hg.keepTemp();
		end_of_power_period = false;
	}

	if (millis() > ac_check) {
		ac_check = millis() + 1000;
		if (!hg.areExternalInterrupts()) {
			nxt = &errScr;
			if (nxt != pCurrentScreen) {
				pCurrentScreen = nxt;
				pCurrentScreen->init();
                reset_encoder = true;
			}
		}
	}
}

#ifndef _RTCLIB_H_
#define _RTCLIB_H_

class TimeSpan;

/** Registers */
#define PCF8523_ADDRESS 0x68       ///< I2C address for PCF8523
#define PCF8523_CLKOUTCONTROL 0x0F ///< Timer and CLKOUT control register
#define PCF8523_CONTROL_1 0x00     ///< Control and status register 1
#define PCF8523_CONTROL_2 0x01     ///< Control and status register 2
#define PCF8523_CONTROL_3 0x02     ///< Control and status register 3
#define PCF8523_TIMER_B_FRCTL 0x12 ///< Timer B source clock frequency control
#define PCF8523_TIMER_B_VALUE 0x13 ///< Timer B value (number clock periods)
#define PCF8523_OFFSET 0x0E        ///< Offset register
#define PCF8523_STATUSREG 0x03     ///< Status register

#define DS1307_ADDRESS 0x68 ///< I2C address for DS1307
#define DS1307_CONTROL 0x07 ///< Control register
#define DS1307_NVRAM 0x08   ///< Start of RAM registers - 56 bytes, 0x08 to 0x3f

#define DS3231_ADDRESS 0x68   ///< I2C address for DS3231
#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_TEMPERATUREREG 0x11 
#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000       

class DateTime 
{
public:
  DateTime(uint32_t t = SECONDS_FROM_1970_TO_2030);
  DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0,
           uint8_t min = 0, uint8_t sec = 0);
  DateTime(const DateTime &copy);
  DateTime(const char *date, const char *time);
  DateTime(const __FlashStringHelper *date, const __FlashStringHelper *time);
  bool isValid() const;
  char *toString(char *buffer);

  uint16_t year() const { return 2000 + yOff; }
  uint8_t month() const { return m; }
  uint8_t day() const { return d; }
  uint8_t hour() const { return hh; }
  uint8_t twelveHour() const;
  uint8_t isPM() const { return hh >= 12; }
  uint8_t minute() const { return mm; }
  uint8_t second() const { return ss; }
  uint8_t dayOfTheWeek() const;
  uint32_t secondstime() const;
  uint32_t unixtime(void) const;
  enum timestampOpt 
{
    TIMESTAMP_FULL, //!< `YYYY-MM-DDThh:mm:ss`
    TIMESTAMP_TIME, //!< `hh:mm:ss`
    TIMESTAMP_DATE  //!< `YYYY-MM-DD`
};
  String timestamp(timestampOpt opt = TIMESTAMP_FULL);

  DateTime operator+(const TimeSpan &span);
  DateTime operator-(const TimeSpan &span);
  TimeSpan operator-(const DateTime &right);
  bool operator<(const DateTime &right) const;

  bool operator>(const DateTime &right) const { return right < *this; }

  bool operator<=(const DateTime &right) const { return !(*this > right); }
  bool operator>=(const DateTime &right) const { return !(*this < right); }
  bool operator==(const DateTime &right) const;
  bool operator!=(const DateTime &right) const { return !(*this == right); }

protected:
  uint8_t yOff; ///< Year offset from 2000
  uint8_t m;    ///< Month 1-12
  uint8_t d;    ///< Day 1-31
  uint8_t hh;   ///< Hours 0-23
  uint8_t mm;   ///< Minutes 0-59
  uint8_t ss;   ///< Seconds 0-59
};

class TimeSpan {
public:
  TimeSpan(int32_t seconds = 0);
  TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
  TimeSpan(const TimeSpan &copy);

  int16_t days() const { return _seconds / 86400L; }
 
  int8_t hours() const { return _seconds / 3600 % 24; }

  int8_t minutes() const { return _seconds / 60 % 60; }

  int8_t seconds() const { return _seconds % 60; }

  int32_t totalseconds() const { return _seconds; }

  TimeSpan operator+(const TimeSpan &right);
  TimeSpan operator-(const TimeSpan &right);

protected:
  int32_t _seconds; ///< Actual TimeSpan value is stored as seconds
};


enum Ds1307SqwPinMode 
{
  DS1307_OFF = 0x00,            // Low
  DS1307_ON = 0x80,             // High
  DS1307_SquareWave1HZ = 0x10,  // 1Hz square wave
  DS1307_SquareWave4kHz = 0x11, // 4kHz square wave
  DS1307_SquareWave8kHz = 0x12, // 8kHz square wave
  DS1307_SquareWave32kHz = 0x13 // 32kHz square wave
};

class RTC_DS1307 
{
public:
  boolean begin(void);
  static void adjust(const DateTime &dt);
  uint8_t isrunning(void);
  static DateTime now();
  static Ds1307SqwPinMode readSqwPinMode();
  static void writeSqwPinMode(Ds1307SqwPinMode mode);
  uint8_t readnvram(uint8_t address);
  void readnvram(uint8_t *buf, uint8_t size, uint8_t address);
  void writenvram(uint8_t address, uint8_t data);
  void writenvram(uint8_t address, uint8_t *buf, uint8_t size);
};

enum Ds3231SqwPinMode 
{
  DS3231_OFF = 0x01,            // Off
  DS3231_SquareWave1Hz = 0x00,  // 1Hz square wave
  DS3231_SquareWave1kHz = 0x08, // 1kHz square wave
  DS3231_SquareWave4kHz = 0x10, // 4kHz square wave
  DS3231_SquareWave8kHz = 0x18  // 8kHz square wave
};

enum Ds3231Alarm1Mode 
{
  DS3231_A1_PerSecond = 0x0F,
  DS3231_A1_Second = 0x0E,
  DS3231_A1_Minute = 0x0C,
  DS3231_A1_Hour = 0x08,
  DS3231_A1_Date = 0x00,
  DS3231_A1_Day = 0x10
};

enum Ds3231Alarm2Mode 
{
  DS3231_A2_PerMinute = 0x7,
  DS3231_A2_Minute = 0x6,
  DS3231_A2_Hour = 0x4,
  DS3231_A2_Date = 0x0,
  DS3231_A2_Day = 0x8
};

class RTC_DS3231 
{
public:
  boolean begin(void);
  static void adjust(const DateTime &dt);
  bool lostPower(void);
  static DateTime now();
  static Ds3231SqwPinMode readSqwPinMode();
  static void writeSqwPinMode(Ds3231SqwPinMode mode);
  bool setAlarm1(const DateTime &dt, Ds3231Alarm1Mode alarm_mode);
  bool setAlarm2(const DateTime &dt, Ds3231Alarm2Mode alarm_mode);
  void disableAlarm(uint8_t alarm_num);
  void clearAlarm(uint8_t alarm_num);
  bool alarmFired(uint8_t alarm_num);
  static float getTemperature(); // in Celcius degree
};

enum Pcf8523SqwPinMode 
{
  PCF8523_OFF = 7,             /**< Off */
  PCF8523_SquareWave1HZ = 6,   /**< 1Hz square wave */
  PCF8523_SquareWave32HZ = 5,  /**< 32Hz square wave */
  PCF8523_SquareWave1kHz = 4,  /**< 1kHz square wave */
  PCF8523_SquareWave4kHz = 3,  /**< 4kHz square wave */
  PCF8523_SquareWave8kHz = 2,  /**< 8kHz square wave */
  PCF8523_SquareWave16kHz = 1, /**< 16kHz square wave */
  PCF8523_SquareWave32kHz = 0  /**< 32kHz square wave */
};

enum PCF8523TimerClockFreq 
{
  PCF8523_Frequency4kHz = 0,   /**< 1/4096th second = 244 microseconds,
                                    max 62.256 milliseconds */
  PCF8523_Frequency64Hz = 1,   /**< 1/64th second = 15.625 milliseconds,
                                    max 3.984375 seconds */
  PCF8523_FrequencySecond = 2, /**< 1 second, max 255 seconds = 4.25 minutes */
  PCF8523_FrequencyMinute = 3, /**< 1 minute, max 255 minutes = 4.25 hours */
  PCF8523_FrequencyHour = 4,   /**< 1 hour, max 255 hours = 10.625 days */
};

enum PCF8523TimerIntPulse 
{
  PCF8523_LowPulse3x64Hz = 0,  /**<  46.875 ms   3/64ths second */
  PCF8523_LowPulse4x64Hz = 1,  /**<  62.500 ms   4/64ths second */
  PCF8523_LowPulse5x64Hz = 2,  /**<  78.125 ms   5/64ths second */
  PCF8523_LowPulse6x64Hz = 3,  /**<  93.750 ms   6/64ths second */
  PCF8523_LowPulse8x64Hz = 4,  /**< 125.000 ms   8/64ths second */
  PCF8523_LowPulse10x64Hz = 5, /**< 156.250 ms  10/64ths second */
  PCF8523_LowPulse12x64Hz = 6, /**< 187.500 ms  12/64ths second */
  PCF8523_LowPulse14x64Hz = 7  /**< 218.750 ms  14/64ths second */
};

enum Pcf8523OffsetMode 
{
  PCF8523_TenSeconds= 0x00, /**< Offset made every ten seconds */
  PCF8523_OneMinute = 0x80 /**< Offset made every minute */
};

class RTC_PCF8523 
{
public:
  boolean begin(void);
  void adjust(const DateTime &dt);
  boolean lostPower(void);
  boolean initialized(void);
  static DateTime now();
  Pcf8523SqwPinMode readSqwPinMode();
  void writeSqwPinMode(Pcf8523SqwPinMode mode);
  void enableSecondTimer(void);
  void disableSecondTimer(void);
  void enableCountdownTimer(PCF8523TimerClockFreq clkFreq, uint8_t numPeriods,
                            uint8_t lowPulseWidth);
  void enableCountdownTimer(PCF8523TimerClockFreq clkFreq, uint8_t numPeriods);
  void disableCountdownTimer(void);
  void deconfigureAllTimers(void);
  void calibrate(Pcf8523OffsetMode mode, int8_t offset);
};

class RTC_Millis 
{
public:
  static void begin(const DateTime &dt) 
 { 
   adjust(dt); 
 }
  static void adjust(const DateTime &dt);
  static DateTime now();

protected:
  static uint32_t lastUnix;   
  static uint32_t lastMillis; 
};

class RTC_Micros 
{
public:
  static void begin(const DateTime &dt) 
  { 
    adjust(dt); 
  }
  static void adjust(const DateTime &dt);
  static void adjustDrift(int ppm);
  static DateTime now();

protected:
  static uint32_t microsPerSecond; 
  static uint32_t lastUnix;   
  static uint32_t lastMicros; 
};

#endif // _RTCLIB_H_

RTC_DS1307 rtc; 
int CS_Pin = 10; // for check run SD Card

#ifndef DS1307RTC_h
#define DS1307RTC_h

#ifndef _DateTime_h
#define _DateTime_h

#ifndef DateTimeStrings_h
#define DateTimeStrings_h

#define dt_MAX_STRING_LEN 9 // length of longest string (excluding terminating null)

class DateTimeStringsClass
{
private:
  char buffer[dt_MAX_STRING_LEN+1];
public:
  DateTimeStringsClass();
  char* monthStr(byte month);
  char* dayStr(byte day);
};

extern DateTimeStringsClass DateTimeStrings;  // make an instance for the user

#endif /* DateTimeString_h */

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define DAYS_PER_WEEK (7L)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52L)
#define SECS_YR_2000  (946681200UL)
 
/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define dayOfWeek(_time_)  (( _time_ / SECS_PER_DAY + 4)  % DAYS_PER_WEEK) // 0 = Sunday
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  // this is number of days since Jan 1 1970
#define elapsedSecsToday(_time_)  (_time_ % SECS_PER_DAY)   // the number of seconds since last midnight 
#define previousMidnight(_time_) (( _time_ / SECS_PER_DAY) * SECS_PER_DAY)  // time at the start of the given day
#define nextMidnight(_time_) ( previousMidnight(_time_)  + SECS_PER_DAY ) // time at the end of the given day 
#define elapsedSecsThisWeek(_time_)  (elapsedSecsToday(_time_) +  (dayOfWeek(_time_) * SECS_PER_DAY) )   

typedef enum 
{
    dtSunday, dtMonday, dtTuesday, dtWednesday, dtThursday, dtFriday, dtSaturday
} dtDays_t;

class DateTime
{
public:
  DateTime();
  DateTime(time_t time);

  void setTime(time_t time);
  void setTime(int8_t sec, int8_t min, int8_t hour, int8_t day, int8_t month, int16_t year);
  bool parseHttpDate(String httpDate);
  bool isNull();

  time_t toUnixTime();
  String toShortDateString();
  String toShortTimeString(bool includeSeconds = false);
  String toFullDateTimeString();

  void addMilliseconds(long add);

  static void convertFromUnixTime(time_t timep, int8_t *psec, int8_t *pmin, int8_t *phour, int8_t *pday, int8_t *pwday, int8_t *pmonth, int16_t *pyear);
  static time_t convertToUnixTime(int8_t sec, int8_t min, int8_t hour, int8_t day, int8_t month, int16_t year); // returns time_t from components

public:
  int8_t Hour;
  int8_t Minute;
  int8_t Second;
  int16_t Milliseconds;
  int8_t Day;
  int8_t DayofWeek; // Sunday is day 0
  int8_t Month; // Jan is month 0
  int16_t Year;  // Full Year numer
};

#endif /* DateTime_h */

class DS1307RTC
{
  // user-accessible "public" interface
  public:
    DS1307RTC();
    static time_t get();
    static bool set(time_t t);
    static bool read(tmElements_t &tm);
    static bool write(tmElements_t &tm);
    static bool chipPresent() { return exists; }
    static unsigned char isRunning();
    static void setCalibration(char calValue);
    static char getCalibration();

  private:
    static bool exists;
    static uint8_t dec2bcd(uint8_t num);
    static uint8_t bcd2dec(uint8_t num);
};

#ifdef RTC
#undef RTC
#endif
extern DS1307RTC RTC;
#endif

void Init_YUV422()
{
  WriteOV7670(0x12, 0x00);//COM7
  WriteOV7670(0x8C, 0x00);//RGB444
  WriteOV7670(0x04, 0x00);//COM1
  WriteOV7670(0x40, 0xC0);//COM15
  WriteOV7670(0x14, 0x1A);//COM9
  WriteOV7670(0x3D, 0x40);//COM13
}

void Init_QVGA()
{
  WriteOV7670(0x0C, 0x04);//COM3 - Enable Scaling
  WriteOV7670(0x3E, 0x19);//COM14
  WriteOV7670(0x72, 0x11);//
  WriteOV7670(0x73, 0xF1);//
  WriteOV7670(0x17, 0x16);//HSTART
  WriteOV7670(0x18, 0x04);//HSTOP
  WriteOV7670(0x32, 0xA4);//HREF
  WriteOV7670(0x19, 0x02);//VSTART
  WriteOV7670(0x1A, 0x7A);//VSTOP
  WriteOV7670(0x03, 0x0A);//VREF
}

void Init_OV7670()
{
  WriteOV7670(0x12,0x80);
  delay(100);
  WriteOV7670(0x3A, 0x04); //TSLB
  WriteOV7670(0x13, 0xC0); //COM8
  WriteOV7670(0x00, 0x00); //GAIN
  WriteOV7670(0x10, 0x00); //AECH
  WriteOV7670(0x0D, 0x40); //COM4
  WriteOV7670(0x14, 0x18); //COM9
  WriteOV7670(0x24, 0x95); //AEW
  WriteOV7670(0x25, 0x33); //AEB
  WriteOV7670(0x13, 0xC5); //COM8
  WriteOV7670(0x6A, 0x40); //GGAIN
  WriteOV7670(0x01, 0x40); //BLUE
  WriteOV7670(0x02, 0x60); //RED
  WriteOV7670(0x13, 0xC7); //COM8
  WriteOV7670(0x41, 0x08); //COM16
  WriteOV7670(0x15, 0x20); //COM10 - PCLK does not toggle on HBLANK
}

void WriteOV7670(byte regID, byte regVal)
{
  Wire.beginTransmission(0x21);
  Wire.write(regID);  
  Wire.write(regVal);
  Wire.endTransmission();
  delay(1);
}

void ReadOV7670(byte regID)
{
  Wire.beginTransmission(0x21); // 7-bit Slave address
  Wire.write(regID);  // reading from register 0x11
  Wire.endTransmission();
  Wire.requestFrom(0x21, 1);
  Serial.print("Read request Status:");
  Serial.println(Wire.available());
  Serial.print(regID,HEX);
  Serial.print(":");
  Serial.println(Wire.read(),HEX);
}

void XCLK_SETUP(void)
{
  pinMode(9, OUTPUT); //Set pin 9 to output  //Initialize timer 1
  TCCR1A = (1 << COM1A0) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); 
  OCR1A = 0;
}

void TWI_SETUP(void)
{
  TWSR &= ~3;
  TWBR = 72;
}

void OV7670_PINS(void)
{
  DDRC &= ~15;//low d0-d3 camera
  DDRD &= ~252;
}

void QVGA_Image(String title)
{
  int h,w;
  File dataFile = SD.open(title, FILE_WRITE);
  while (!(PIND & 8));//wait for high
  while ((PIND & 8));//wait for low

    h = 240;
  while (h--)
  {
        w = 320;
       byte dataBuffer[320];
    while (w--)
    {
      while ((PIND & 4));   //wait for low
        dataBuffer[319-w] = (PINC & 15) | (PIND & 240);
      while (!(PIND & 4));  //wait for high
      while ((PIND & 4));   //wait for low
      while (!(PIND & 4));  //wait for high
    }
    dataFile.write(dataBuffer,320);    
  }

    dataFile.close();
    delay(100);
}

String print_time(DateTime timestamp) // print_time() returns a string in the format mmddyyyyhhmmss
{
  char message[120];
  int Year = timestamp.year();
  int Month = timestamp.month();
  int Day = timestamp.day();
  int Hour = timestamp.hour();
  int Minute = timestamp.minute();
  int Second= timestamp.second();

  sprintf(message, "%d%d%d%02d%02d%02d", Month, Day, Year, Hour, Minute, Second);
  return message;
}

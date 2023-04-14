/*
  Drone_sensor.h - Library for drone mounted chlorphyll sensor.
  Created by Zechariah Kitzhaber, February 18, 2022.
  Released into the public domain. MIT license.
*/

#ifndef Drone_sensor_h
#define Drone_sensor_h

#include "Arduino.h"
#include "mbed.h"                 // library for NRF52480 (uc chip)
#include "SdFat.h"                // SD library for data storage
#include "RTClib.h"               // Real Time Clock library
#include <Arduino_LSM9DS1.h>      // Library for IMU (gyroscope and accelerometer)
#include <Arduino_HTS221.h>       // Library for temp and humidity sensor
#include <PDM.h>                  // Library for microphone

//#include "defined_constants.h"

#define SENSOR_REPORT_FREQUENCY 10
#define SENSOR_LASER_DELAY 5000
#define SENSOR_WRITE_SYNC_DELAY 1000


class Drone_sensor {
public:
    Drone_sensor();               // Constructor creates an instance of Drone_sensor class and sets defaults

    bool init();                  // Initialize sensor (called in setup)
    bool start(bool laserState = true);                 // start ADC
    void shutdown();              // shutdown ADC
    bool laser(bool laserState);  // turn laser on/off
    bool shutter(bool shutterState); // open/close shutter

    inline bool dataReady() {           // true when ADC buffer is full
        return(NRF_SAADC->EVENTS_END);
    }
    inline bool isActive() {
        return(_active);
    }
    inline void setMode(int mode) {
        _mode = mode;
    }
    inline int getMode() {
        return(_mode);
    }

    bool newFolder();
    bool newFile();
    uint32_t _blockSize = 512;                  // SD card is written in blocks of 512 bytes


    RTC_DS1307 rtc;                   // create Real Time Clock object

    SdExFat sd;                       // create SD and File objects with exFAT file system
    ExFile file;

    char* readErrors();
    int _errorCase = 0;

    /* IMU (gyroscope and accelerometer) data is float type*/
    typedef float* IMUdata_t;
    float IMUdata[6] = { 0 };
    void getIMU() {
        while (!IMU.accelerationAvailable()) {}
        IMU.readAcceleration(IMUdata[0], IMUdata[1], IMUdata[2]);

        while (!IMU.gyroscopeAvailable()) {}
        IMU.readGyroscope(IMUdata[3], IMUdata[4], IMUdata[5]);
    };


    /* Struct for PDM data (from microphone) */
    typedef struct {
        short sampleBuffer[512] = { 0 };
        int bytesAvailable = 0;
        volatile int samplesRead = 0;
        bool flag = false;
    }PDMdata_t;
    PDMdata_t PDMdata;

    /* Struct for HTS data (temp and humidity) */
    typedef struct {
        float temperature = 0;
        float humidity = 0;
    }HTSdata_t;

    HTSdata_t HTSdata;
    
    void getHTS() {
        HTSdata.temperature = HTS.readTemperature();
        HTSdata.humidity = HTS.readHumidity();
    };

    /* class for ADC channels */
    template <int N>
    class ADC_channel {
    public:
        ADC_channel(uint8_t i, nrf_saadc_input_t pp) :   // class constructor allows user to set oversample and pin
          oversample{i}, 
          config{  
            NRF_SAADC_RESISTOR_DISABLED,  // resistor_p
            NRF_SAADC_RESISTOR_DISABLED,  // resistor_n
            NRF_SAADC_GAIN4,              // gain
            NRF_SAADC_REFERENCE_INTERNAL, // reference (voltage)
            NRF_SAADC_ACQTIME_5US,        // acq_time
            NRF_SAADC_MODE_SINGLE_ENDED,  // mode
            NRF_SAADC_BURST_DISABLED,     // burst
            pp,                           // pin_p
            NRF_SAADC_INPUT_DISABLED      // pin_n
          } 
        {} // end constructor
        uint8_t channelNumber = 0;
        uint8_t index = 0;                    // this depends on the number of active channels
        uint8_t oversample;               // how many samples to average per result
        float results[N] = { 0 };
        int resultSize = N;
        float uVperLSB = 0;
        nrf_saadc_channel_config_t config;

        inline uint8_t getChannelNumber() {
            return(channelNumber);
        }
        inline uint8_t getIndex() {
            return(index);
        }
        inline uint8_t getOversample() {
            return(oversample);
        }
        void setGain(nrf_saadc_gain_t gain) {
            config.gain = gain;
        };
        void setGain(int gain) {
            config.gain = nrf_saadc_gain_t(gain);
        };
        void define_uVperLSB() {
            float uVfullScale = 600000; // always use internal 0.6 V reference
            float gainVal = 0;
            switch (config.gain) {
            case NRF_SAADC_GAIN1_6:
                gainVal = float(1.0 / 6.0);
                break;
            case NRF_SAADC_GAIN1_5:
                gainVal = float(1.0 / 5.0);
                break;
            case NRF_SAADC_GAIN1_4:
                gainVal = float(1.0 / 4.0);
                break;
            case NRF_SAADC_GAIN1_3:
                gainVal = float(1.0 / 3.0);
                break;
            case NRF_SAADC_GAIN1_2:
                gainVal = float(1.0 / 2.0);
                break;
            case NRF_SAADC_GAIN1:
                gainVal = float(1);
                break;
            case NRF_SAADC_GAIN2:
                gainVal = float(2);
                break;
            case NRF_SAADC_GAIN4:
                gainVal = float(4);
                break;
            }
            float LSBfullScale = pow(2, (12 - (config.mode == NRF_SAADC_MODE_DIFFERENTIAL)));
            uVperLSB = uVfullScale / gainVal / LSBfullScale;
        }
    };

    /* class for ADC core */
    template <int N>
    class ADC {
    public:
        int16_t buffer[N] = { 0 };
        int16_t* bufferPtr = buffer;
        uint32_t bufferSize = N;
        byte _numChannels = 0;                // Number of active/enabled channels
       // ADC_channel* channelList[9] = { 0 };
        int _samplePeriod;
        bool OOR_flag = false;

        bool init() {

            if (!initSampleTimer()) {
                return(false);
            }
            initPPI();

            nrf_saadc_disable();

            nrf_saadc_resolution_t resolution = NRF_SAADC_RESOLUTION_12BIT;

            nrf_saadc_oversample_t oversample = NRF_SAADC_OVERSAMPLE_DISABLED;

            NRF_SAADC->RESULT.PTR = (uint32_t)bufferPtr;
            NRF_SAADC->RESULT.MAXCNT = (uint32_t)bufferSize;                 // Set up Result buffer

            nrf_saadc_resolution_set(resolution);

            nrf_saadc_oversample_set(oversample);

            NRF_SAADC->EVENTS_END = 0;
            nrf_saadc_int_enable(NRF_SAADC_INT_END);              // Set up Interrupt
            //NVIC_SetPriority(SAADC_IRQn, 1UL);
            NVIC_DisableIRQ(SAADC_IRQn);
            nrf_saadc_enable();

            calibrate();

            return (NRF_SAADC->STATUS == SAADC_STATUS_STATUS_Ready); // return true if SAADC ready
        };

        void calibrate() {
            NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;                   // Calibrate SAADC
            while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
            NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
            while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));
        };
        inline bool dataReady() {
            return(NRF_SAADC->EVENTS_END);
        }

        template<int NN>
        byte enableChannel(ADC_channel<NN>* channel) {
            channel->channelNumber = _numChannels;
            channel->index = _numChannels;
            //channelList[_numChannels] = channel;
            channel->setGain(NRF_SAADC_GAIN4); // start at max gain
            nrf_saadc_channel_init(channel->channelNumber, &channel->config);
            _numChannels++;
            channel->define_uVperLSB();

            return(_numChannels);
        };

        template<int NN>
        byte disableChannel(ADC_channel<NN>* channel) {
            /* need to finish this code for edge cases
            for (int i = channel->index; i < _numChannels; i++) {
                channelList[i] = channelList[i + 1];
                if (channelList[i]) {
                    channelList[i]->index = i;
                }
            }
            */
            nrf_saadc_channel_input_set(channel->channelNumber, NRF_SAADC_INPUT_DISABLED, NRF_SAADC_INPUT_DISABLED);
            return(_numChannels--);
        };

        /* Separates data for a given channels and averages oversampled data*/
        template<int NN>
        float getData(ADC_channel<NN>* channel) {
            int32_t resultSum = 0;
            float result = 0;

            if (channel->resultSize > (bufferSize / _numChannels / channel->oversample)) { // make sure we don't overrun the ADC buffer
                channel->resultSize = (bufferSize / _numChannels / channel->oversample);
            }
            
            int OOR = 0;
            int OOR_max = (bufferSize / _numChannels / 16) + 1; // the max number of acceptable OOR values is 1/16th of the channel results (with a min value of 1)
            int16_t LSBfullScale = pow(2, (11 - (channel->config.mode == NRF_SAADC_MODE_DIFFERENTIAL)));

            for (int i = 0; i < channel->resultSize; i++) {
                channel->results[i] = 0.0;
                int32_t oversampleSum = 0;
                for (int j = channel->index; j < (channel->oversample * _numChannels); j += _numChannels) {
                    oversampleSum += buffer[(channel->oversample * _numChannels * i) + j];         // average oversampling
                    if(OOR < OOR_max){                                 
                      if(buffer[(channel->oversample * _numChannels * i) + j] >= LSBfullScale){    // check for Out Of Range values
                        OOR++;                                                                     // and count the total
                      } 
                    }else{                                                                         // stop counting after the max is reached
                      OOR_flag = true;                                                             // set flag to autoscale the gain (the flag is not cleared automatically within this or any other function)
                    }
                }
                channel->results[i] = float(oversampleSum) * channel->uVperLSB / channel->oversample;
                resultSum += oversampleSum;
            }

            if(OOR_flag && channel->config.gain){ // if signal is out of range, and we're not already at the lowest gain setting
              nrf_saadc_disable();
              channel->setGain(channel->config.gain - 1);            // decrease the gain
              nrf_saadc_channel_init(channel->channelNumber, &channel->config); // apply new gain setting
              channel->define_uVperLSB();
              nrf_saadc_enable();
            }

            NRF_SAADC->EVENTS_STARTED = 0;
            result = resultSum * channel->uVperLSB / channel->resultSize / channel->oversample;
            return(result);
        };

    private:
        /* If called with no argument, use default value */
        bool initSampleTimer() {
            if (_numChannels == 0 || bufferSize == 0) {  // 
                return(false);
            }
            // ((1000000 us per s / reports per s / 2 (periods per report)) - delay us) / ( samples / channels )
            _samplePeriod = ((1000000 / SENSOR_REPORT_FREQUENCY / 2) - SENSOR_LASER_DELAY) / (bufferSize / _numChannels);
            initSampleTimer(_samplePeriod);
            return(true);
        }

        /* Allows user to specify a sampling frequency. Argument is time between samples in us. */
        void initSampleTimer(int samplePeriod) {
            _samplePeriod = samplePeriod;
            NRF_TIMER2->TASKS_STOP = 1; // timer must be stopped to change settings
            NRF_TIMER2->TASKS_CLEAR = 1;

            /* Timer 3 will trigger sampling on the SAADC */
            NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
            NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
            NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
            NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;  // reset the timer when it reaches CC[0]
            NRF_TIMER2->PRESCALER = 4; // Timer freq = 16MHz/(2^PRESCALE) = 1 MHz
            NRF_TIMER2->CC[0] = _samplePeriod; // sets time between samples in us
        }

        /* The Programmable Peripheral Interconnect (PPI) enables connections between task and event registers of different peripherals */
        void initPPI() {
            NRF_PPI->CH[4].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];   // When Timer 2 reaches compare 0
            NRF_PPI->CH[4].TEP = (uint32_t)&NRF_SAADC->TASKS_SAMPLE;         // Sample on SAADC (all channels)

            NRF_PPI->CH[5].EEP = (uint32_t)&NRF_SAADC->EVENTS_END;           // when the ADC buffer is full
            NRF_PPI->CH[5].TEP = (uint32_t)&NRF_TIMER2->TASKS_STOP;          // stop the sample timer   
            NRF_PPI->FORK[5].TEP = (uint32_t)&NRF_SAADC->TASKS_STOP;         // stop the ADC

            NRF_PPI->CHENSET = (1UL << 4) |      // enable PPI channels 4 and 5
                (1UL << 5);
        }
    };

    template<int N>
    bool initADC(ADC<N>* adc) {
        if (!adc->init()) {
            _errorCase = 5;
            _mode = 0;
            return(false);
        }
        return(true);
    };

    // various functions for writing data to the SD cards, all overloaded to writeFile()
    template <int N>
    uint32_t writeFile(ADC_channel<N> channel) {
        const void* filePtr = (const void*)channel.results;     // write the result buffer for a specific channel to file
        int blockSize = channel.resultSize * 4;
        file.write(filePtr, blockSize);
        if (millis() > _writeTimer) {                           // sync file once per second
            file.sync();
        }
        _writeTimer = millis()+SENSOR_WRITE_SYNC_DELAY;
        return(_writeTimer);
    };
    template <int N>
    void writeFile(ADC<N> adc) {
        const void* filePtr = (const void*)adc.bufferPtr;       // write the entire ADC buffer to file
        int blockSize = adc.bufferSize * 2;
        file.write(filePtr, blockSize);
        if (millis() > _writeTimer) {                           // sync file once per second
            file.sync();
        }
        _writeTimer = millis()+SENSOR_WRITE_SYNC_DELAY;
    };
    void writeFile(IMUdata_t IMUdata) {
        int blockSize = 24;
        const void* filePtr = (const void*)IMUdata;             // write IMU data to file
        file.write(filePtr, blockSize);
        if (millis() > _writeTimer) {                           // sync file once per second
            file.sync();
        }
        _writeTimer = millis()+SENSOR_WRITE_SYNC_DELAY;
    }
    void writeFile(PDMdata_t PDMdata) {
        int blockSize = PDMdata.bytesAvailable;
        const void* filePtr = (const void*)PDMdata.sampleBuffer; // write microphone data to file
        if (millis() > _writeTimer) {                            // sync file once per second
            file.sync();
        }
        _writeTimer = millis()+SENSOR_WRITE_SYNC_DELAY;
    }
    void writeFile(HTSdata_t HTSdata) {
        file.print(HTSdata.temperature);
        file.print("\t");
        file.println(HTSdata.humidity);
        file.sync();
    }

private:
    const byte shutterEN = 4;             // pin 4 is the enable pin for the shutter
    const byte shutterPos = 5;
    const byte shutterNeg = 6;


    /* Sets up pin 9 (laser control) with tasks and events */
    void initGPIOTE() {
        NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |          // enable tasks on GPIOTE channel 0
            (0x1BUL << GPIOTE_CONFIG_PSEL_Pos) |                           // select pin 27 (maps to pin 9 on Nano)
            (0x0UL << 0xDUL) |                                             // this sets the port to 0
            (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) | // set up toggle task
            (GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos);     // pin will be initialized to high
    }

    char _fileName[11] = { 'd','a','t','a','0','0','.','t','x','t','\0' };

    bool _active = false;
    bool _laserState = false;
    bool _shutterState = false;

    int _mode = 0;

    int _laserDelay = 5000;               // Time in us for laser to turn on/off

    uint32_t _writeTimer = 0;
};

#endif
/*
  Drone_sensor.cpp - Library for drone mounted chlorphyll sensor.
  Created by Zechariah Kitzhaber, February 18, 2022.
  Released into the public domain. MIT license.
*/

/* TODO:
    - implement startup routine
*/


#include "Drone_sensor.h"
/* Construct instance of Drone_sensor class and set defaults (call before setup()) */
Drone_sensor::Drone_sensor() {

}

/* Should be called in setup() */
bool Drone_sensor::init() {
  // Declare pin modes for the shutter controls
  pinMode(shutterEN, OUTPUT);
  pinMode(shutterPos, OUTPUT);
  pinMode(shutterNeg, OUTPUT);

  initGPIOTE();

  if (!rtc.begin()) {
      _errorCase = 1;
      return(false);
  }
  if (!HTS.begin()) {
      _errorCase = 2;
      return(false);
  }
  if (!IMU.begin()) {
      _errorCase = 3;
      return(false);
  }
  if (!sd.begin(SS)) {
      _errorCase = 4;
      return(false);
  }

  return(true);  // only true if all (rtc, sd, IMU, HTS) begin successfully
}

bool Drone_sensor::start( bool laserState ) {
    if (!_active) {
        _active = true;
        laser(laserState);
        NRF_SAADC->EVENTS_STARTED = 0;
    }
    if (NRF_SAADC->EVENTS_STARTED == 0) {
        NRF_TIMER2->TASKS_CLEAR = 1;
        NRF_SAADC->TASKS_START = 1;      // start the SAADC
        NRF_SAADC->EVENTS_END = 0;
        while (NRF_SAADC->EVENTS_STARTED == 0);

        NRF_TIMER2->TASKS_START = 1;
        return(true);
    }
    else {
        _errorCase = 5;
        _mode = 0;
        return(false);
    }
}

void Drone_sensor::shutdown() {
  if ( _active ) {
    NRF_TIMER2->TASKS_STOP = 1;     // stop and clear timer
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_SAADC->TASKS_STOP = 1;      // stop the SAADC
    // Disable all ADC channels
    for (uint8_t chan = 0; chan < 8; chan++) {
        nrf_saadc_channel_input_set(chan, NRF_SAADC_INPUT_DISABLED, NRF_SAADC_INPUT_DISABLED);
    }
    laser(false);                       // laser off
    _active = false;
  }
  file.close();
}



/* Laser and shutter controls */
bool Drone_sensor::laser( bool laserState ) {
    if(_laserState != laserState){                  // only do something if the input is different from the current laser state
      _laserState = laserState;
      if ( _laserState ) {
          NRF_GPIOTE->TASKS_CLR[0] = 1;             // turn laser on
      } else {
          NRF_GPIOTE->TASKS_SET[0] = 1;             // turn laser off
      }
      unsigned long timeIn = micros();
      while (micros() < timeIn + SENSOR_LASER_DELAY) {} // wait for the laser output to stabilize
    }
    return( _laserState );
}

bool Drone_sensor::shutter( bool shutterState ) {
  _shutterState = shutterState;
  digitalWrite(shutterEN, HIGH);              // enable shutter
  digitalWrite(shutterPos, _shutterState);
  digitalWrite(shutterNeg, !_shutterState);
  delay(200);
  digitalWrite(shutterEN, LOW);               // disable shutter
  digitalWrite(shutterPos, LOW);
  digitalWrite(shutterNeg, LOW);
  return(_shutterState);
}

bool Drone_sensor::newFolder() {
    DateTime now = rtc.now();
    char folderTemplate[] = "DD_MM_YY_hhmm";
    const char* currentFolder = now.toString(folderTemplate);

    sd.chdir("/");
    if (!sd.exists(currentFolder)) {
        if (!sd.mkdir(currentFolder)) {
            _errorCase = 6;
            _mode = 0;
            return(false);
        }
    }
    sd.chdir(currentFolder);
    return(true);
}

bool Drone_sensor::newFile() {
    uint8_t fileNumber = 0;
    switch (_mode) {
    case 2:
      _fileName[0] = 'c';
      _fileName[1] = 'a';
      _fileName[2] = 'l';
      _fileName[3] = 'i';
      _fileName[4] = '0';
      _fileName[5] = '0';
      break;
    case 14:
      _fileName[0] = 'a';
      _fileName[1] = 'd';
      _fileName[2] = 'c';
      _fileName[3] = 'b';
      _fileName[4] = '0';
      _fileName[5] = '0';
      break;
    case 15:
      _fileName[0] = 'f';
      _fileName[1] = 'r';
      _fileName[2] = 's';
      _fileName[3] = 'c';
      _fileName[4] = '0';
      _fileName[5] = '0';
      break;
    default:
      _fileName[0] = 'd';
      _fileName[1] = 'a';
      _fileName[2] = 't';
      _fileName[3] = 'a';
      _fileName[4] = '0';
      _fileName[5] = '0';
      break;
    }

    while (sd.exists(_fileName)) {                                // if file already exists
      fileNumber++;                                            // try the next number
      _fileName[4] = fileNumber / 10 + '0';                       // sets the 10's place
      _fileName[5] = fileNumber % 10 + '0';                       // sets the 1's place
    }

    if (!file.open(_fileName, O_CREAT | O_TRUNC | O_WRITE)) {     // create file
      _errorCase = 7;
      _mode = 0;
      if(!sd.begin()){
        _errorCase = 8;
      }
      return(false);
    }

    if (_mode == 10) {
      file.println("Temperature\tHumidity");
    }

    return (true);
}

char* Drone_sensor::readErrors() {
    int errorCase = _errorCase;
    _errorCase = 0;
    switch (errorCase) {
        case 0:
            break;
        case 1:
            return(F("RTC error"));
            break;
        case 2:
            return(F("HTS error"));
            break;        
        case 3:
            return(F("IMU error"));
            break;        
        case 4:
            return(F("SD error"));
            break;        
        case 5:
            return(F("ADC error"));
            break;        
        case 6:
            return(F("SD folder error"));
            break; 
        case 7:
            return(F("SD file error"));
            break;    
        default:
            return("Unexpected error");
            break;
    }
}

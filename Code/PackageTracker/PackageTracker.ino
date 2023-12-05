
//Machine Learning
#include <Package_Assurance_inferencing.h>
#include "Arduino_BMI270_BMM150.h"
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
float threshold = 0.7;

//Notecard
#include <Wire.h>
#include <Notecard.h>
#define productUID "com.gmail.nekiary07:package_assurance_system"
Notecard notecard;
#define serialDebug Serial


// GPS
#include <Adafruit_GPS.h>
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false
uint32_t timer = millis();
float gps_latitude;
float gps_longitude;

struct Coordinates {
    float loc_lat;
    float loc_lon;
};

void setup()
{
 
    serialDebug.begin(9600);
    notecard.setDebugOutputStream(serialDebug);

    while (!Serial);
    Serial.println("Package Tracker");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

      Wire.begin();
      
      notecard.begin(0x17);
      J *restore = notecard.newRequest("card.restore");
      JAddBoolToObject(restore, "delete", true);
      notecard.sendRequest(restore);
      delay(5000);
      
      J *req = notecard.newRequest("hub.set");
      JAddStringToObject(req, "product", productUID);
      JAddStringToObject(req, "mode", "continuous");
      JAddBoolToObject(req, "sync", true);
      JAddNumberToObject(req, "inbound", 1);
      JAddNumberToObject(req, "outbound", 1);
      notecard.sendRequestWithRetry(req, 5);
      delay(2000);


    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
  
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
  
    delay(1000);
  
    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);


  Serial.println("Notecard Setting Up...");
  delay(120000);
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

Coordinates getLocation(){
  Coordinates present_location;
  int lat_d, lon_d;
  float lat_m, lon_m;
  int fix_quality = 0;

  while (fix_quality < 1) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trying to print out data
      Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        continue; // we can fail to parse a sentence in which case we should just wait for another
    }
  
    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) 
    {
      timer = millis(); // reset the timer
      
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      fix_quality = int(GPS.fixquality);
      if (GPS.fix) 
      {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
        
        int lat_d = int(GPS.latitude)/100;
        float lat_m = fmod(GPS.latitude, 100.0);
        present_location.loc_lat = lat_d + (lat_m/60.0);
        int lon_d = int(GPS.longitude)/100;
        float lon_m = fmod(GPS.longitude, 100.0);
        present_location.loc_lon = lon_d + (lon_m/60.0);
        
        if (String(GPS.lat)=="S")
        {
          present_location.loc_lat *= -1;
        }
        if (String(GPS.lon)=="W")
        {
          present_location.loc_lon *= -1;
        }
      }
    }
  }
  return present_location;
}

void loop()
{
    ei_printf("\nStarting inferencing in 2 seconds...\n");

    delay(300);

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) 
    {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }

    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("anomaly score: %.3f\n", result.anomaly);
    if(result.anomaly > 1)
    {
      Coordinates present_location = getLocation();
      Serial.print("Latitude: ");
      Serial.println(present_location.loc_lat, 6);
      Serial.print("Longitude: ");
      Serial.println(present_location.loc_lon, 6);
      
      int current_time = 0;
      J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
      if (rsp != NULL)
      {
          current_time = JGetNumber(rsp, "time");
          notecard.deleteResponse(rsp);
      }
      Serial.print("Time: ");
      Serial.println(current_time);
      
      Serial.println("Sending Values to the Notehub");
      J *req = notecard.newRequest("note.add");
      if (req != NULL)
      {
          //JAddBoolToObject(req, "sync", true);
          J *body = JAddObjectToObject(req, "body");
          if (body != NULL)
          {
              JAddNumberToObject(body, "latitude", present_location.loc_lat);
              JAddNumberToObject(body, "longitude", present_location.loc_lon);
              JAddNumberToObject(body, "timestamp", current_time);
          }
          notecard.sendRequest(req);
      }
      delay(2000);

 }
    
#endif
}

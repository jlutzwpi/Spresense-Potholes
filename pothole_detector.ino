#include <Spresense-Potholes_inferencing.h>
#include <Camera.h>
#include <SDHCI.h>
#include <RTC.h>
#include <GNSS.h>
#include <LTE.h>
#include <ArduinoMqttClient.h>
#include "gnss_nmea.h"
#include "certs.h"

/* Defines to center crop and resize a image to the Impulse image size the speresense hardware accelerator
   NOTE: EI_CLASSIFIER_INPUT width and height must be less than RAW_HEIGHT * SCALE_FACTOR, and must
   simultaneously meet the requirements of the spresense api:
   https://developer.sony.com/develop/spresense/developer-tools/api-reference/api-references-arduino/group__camera.html#ga3df31ea63c3abe387ddd1e1fd2724e97
*/
#define SCALE_FACTOR 1
#define RAW_WIDTH CAM_IMGSIZE_QVGA_H
//320
#define RAW_HEIGHT CAM_IMGSIZE_QVGA_V
//240
#define CLIP_WIDTH (EI_CLASSIFIER_INPUT_WIDTH * SCALE_FACTOR)
//96*1 = 96
#define CLIP_HEIGHT (EI_CLASSIFIER_INPUT_HEIGHT * SCALE_FACTOR)
//96
#define OFFSET_X  ((RAW_WIDTH - CLIP_WIDTH) / 2)
//(320-96) / 2 = 112
#define OFFSET_Y  ((RAW_HEIGHT - CLIP_HEIGHT) / 2)
//(240-960 / 2 = 72
// enable for very verbose logging from edge impulse sdk
#define DEBUG_NN false

/* Definitions for LTE login */
// APN name
#define APP_LTE_APN "hologram" // replace your APN
// no APN username/password required
#define APP_LTE_USER_NAME "" // replace with your username
#define APP_LTE_PASSWORD  "" // replace with your password
// APN IP type
#define APP_LTE_IP_TYPE (LTE_NET_IPTYPE_V4V6) // IP : IPv4v6
// APN authentication type
#define APP_LTE_AUTH_TYPE (LTE_NET_AUTHTYPE_NONE) // Authentication : NONE
// RAT to use
#define APP_LTE_RAT (LTE_NET_RAT_CATM) // RAT : LTE-M (LTE Cat-M1)
// MQTT broker
#define BROKER_NAME        "aw5kzq8z8mq4r-ats.iot.us-east-1.amazonaws.com"
#define BROKER_PORT        8883               // port 8883 is the default for MQTT over TLS.
#define MQTT_TOPIC         "pothole/coords"
// MQTT publish interval settings
#define PUBLISH_INTERVAL_SEC   5   // MQTT publish interval in sec
#define MAX_NUMBER_OF_PUBLISH  60  // Maximum number of publish

#define CLASSIFIER_THRESHOLD 0.7
#define CLASSIFIER_POTHOLE_INDEX 1

/* static variables */
static CamImage sized_img;
static ei_impulse_result_t ei_result = { 0 };
int take_picture_count = 0;
LTE lteAccess;
LTETLSClient client;
MqttClient mqttClient(client);
SpGnss Gnss;
SDClass  theSD;
int numOfPubs = 0;
unsigned long lastPubSec = 0;
char broker[] = BROKER_NAME;
int port = BROKER_PORT;
char topic[]  = MQTT_TOPIC;
float curLat;
float curLong;

/* prototypes */
void printError(enum CamErr err);
void CamCB(CamImage img);

/**
   Print error message
*/
void printError(enum CamErr err)
{
  Serial.print("Error: ");
  switch (err)
  {
    case CAM_ERR_NO_DEVICE:
      Serial.println("No Device");
      break;
    case CAM_ERR_ILLEGAL_DEVERR:
      Serial.println("Illegal device error");
      break;
    case CAM_ERR_ALREADY_INITIALIZED:
      Serial.println("Already initialized");
      break;
    case CAM_ERR_NOT_INITIALIZED:
      Serial.println("Not initialized");
      break;
    case CAM_ERR_NOT_STILL_INITIALIZED:
      Serial.println("Still picture not initialized");
      break;
    case CAM_ERR_CANT_CREATE_THREAD:
      Serial.println("Failed to create thread");
      break;
    case CAM_ERR_INVALID_PARAM:
      Serial.println("Invalid parameter");
      break;
    case CAM_ERR_NO_MEMORY:
      Serial.println("No memory");
      break;
    case CAM_ERR_USR_INUSED:
      Serial.println("Buffer already in use");
      break;
    case CAM_ERR_NOT_PERMITTED:
      Serial.println("Operation not permitted");
      break;
    default:
      break;
  }
}

// grayscale to rgb
static inline void mono_to_rgb(uint8_t mono_data, uint8_t *r, uint8_t *g, uint8_t *b) {
  uint8_t v = mono_data;
  *r = *g = *b = v;
}

/* Attach to the LTE network */
void doAttach()
{
  char apn[LTE_NET_APN_MAXLEN] = APP_LTE_APN;
  LTENetworkAuthType authtype = APP_LTE_AUTH_TYPE;
  char user_name[LTE_NET_USER_MAXLEN] = APP_LTE_USER_NAME;
  char password[LTE_NET_PASSWORD_MAXLEN] = APP_LTE_PASSWORD;

  while (true) {
    /* Power on the modem and Enable the radio function. */
    if (lteAccess.begin() != LTE_SEARCHING) {
      Serial.println("Could not transition to LTE_SEARCHING.");
      Serial.println("Please check the status of the LTE board.");
      for (;;) {
        sleep(1);
      }
    }

    /* The connection process to the APN will start.
     * If the synchronous parameter is false,
     * the return value will be returned when the connection process is started.
     */
    if (lteAccess.attach(APP_LTE_RAT,
                         apn,
                         user_name,
                         password,
                         authtype,
                         APP_LTE_IP_TYPE,
                         false) == LTE_CONNECTING) {
      Serial.println("Attempting to connect to network.");
      break;
    }

    /* If the following logs occur frequently, one of the following might be a cause:
     * - APN settings are incorrect
     * - SIM is not inserted correctly
     * - If you have specified LTE_NET_RAT_NBIOT for APP_LTE_RAT,
     *   your LTE board may not support it.
     */
    Serial.println("An error has occurred. Shutdown and retry the network attach preparation process after 1 second.");
    lteAccess.shutdown();
    sleep(1);
  }
}

int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t bytes_left = length;
  size_t out_ptr_ix = 0;

  uint8_t *buffer = sized_img.getImgBuff();

  // read byte for byte
  while (bytes_left != 0) {

    // grab the value and convert to r/g/b
    uint8_t pixel = buffer[offset];

    uint8_t r, g, b;
    mono_to_rgb(pixel, &r, &g, &b);

    // then convert to out_ptr format
    float pixel_f = (r << 16) + (g << 8) + b;
    out_ptr[out_ptr_ix] = pixel_f;

    // and go to the next pixel
    out_ptr_ix++;
    offset++;
    bytes_left--;
  }

  // and done!
  return 0;
}

/**
   @brief run inference on the static sized_image buffer using the provided impulse
*/
static void ei_pothole_camera_classify(bool debug) {
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_cutout_get_data;

  EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, DEBUG_NN);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERROR: Failed to run classifier (%d)\n", err);
    return;
  }
  // print the predictions
  if (debug) {
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              ei_result.timing.dsp, ei_result.timing.classification, ei_result.timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: ", ei_result.classification[ix].label);
      ei_printf_float(ei_result.classification[ix].value);
      ei_printf("\n");
    }
  }

  return;
}

/**
 * @brief callback that checks for the presence of an animal in the camera preview window, and then 
 *   executes ei_wildlife_camera_snapshot() if found
 */
void CamCB(CamImage img)
{
  if (!img.isAvailable()) return; // fast path if image is no longer ready
  CamErr err;
  Serial.println("INFO: new frame processing...");
  // resize to prepare for inferencing
  err = img.clipAndResizeImageByHW(sized_img
                                   , OFFSET_X, OFFSET_Y
                                   , OFFSET_X + CLIP_WIDTH - 1
                                   , OFFSET_Y + CLIP_HEIGHT - 1
                                   , EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
  if (err) 
  { 
    Serial.println("Clip and resize error!");
    printError(err);
  }
  String outputStr = "Image input height/width: " + String(sized_img.getHeight()) + ", " +
    String(sized_img.getWidth());
  Serial.println(outputStr);
  //take_picture_count++;
  
  err = sized_img.convertPixFormat(CAM_IMAGE_PIX_FMT_GRAY);
  if (err) {
    Serial.println("Convert to grayscale error!");
    printError(err);
  }

  // get inference results on resized grayscale image
  ei_pothole_camera_classify(true);

  if (ei_result.classification[CLASSIFIER_POTHOLE_INDEX].value >= CLASSIFIER_THRESHOLD) {
    Serial.println("INFO: Pothole detected!");
    ei_pothole_camera_snapshot(true);
    // if an pothole snapshot is captured, pause to check for connectivity and GNSS prior to taking followup photos
    //err = theCamera.startStreaming(false, CamCB);
  }
}

/**
   @brief initialize the camera for continuous monitoring of video feed
*/
void ei_pothole_camera_start_continuous(bool debug) {
  CamErr err;

  err = theCamera.begin(1, CAM_VIDEO_FPS_5, RAW_WIDTH, RAW_HEIGHT);
  if (err && debug) 
  {
    Serial.println("Camera begin error!");
    printError(err);
  }

  // start streaming the preview images to the classifier
  err = theCamera.startStreaming(true, CamCB);
  if (err && debug) 
  {
    Serial.println("Start streaming error!");
    printError(err);
  }
    
  // still image format must be jpeg to allow for compressed storage/transmit
  err = theCamera.setStillPictureImageFormat(
    RAW_WIDTH,
    RAW_HEIGHT,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err && debug) 
  {
    Serial.println("SetStillPictureImageFOrmat error!");
    printError(err);
  }

  if (debug) Serial.println("INFO: started pothole camera recording");
}


/**
   @brief take a jpeg picture, save to SD card
*/
void ei_pothole_camera_snapshot(bool debug)
{
  //take a picture and publish to MQTT broker, filename has GPS coords
  unsigned long currentTime = lteAccess.getTime();
  String curLatStr = String(curLat, 6);
  String curLongStr = String(curLong, 6);
  if (currentTime >= lastPubSec + PUBLISH_INTERVAL_SEC) {
    // Publish to broker
    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    Serial.print("Publish: ");
    String jsonString = "{\"deviceid\": \"spresense\",\"timestamp\":" + 
        String(currentTime) +
        ",\"latitude\":" + curLatStr +
        ",\"longitude\":" + curLongStr + "}";
    Serial.println(jsonString);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print(jsonString);
    //mqttClient.print(out);
    mqttClient.endMessage();
    lastPubSec = currentTime;
    delay(1000);
    numOfPubs++;
  }
  char filename[200];
  // snapshot and save a jpeg
  if(take_picture_count < 20) {
    CamImage img = theCamera.takePicture();
    if (img.isAvailable()) {
      //split string by decimal point and combine
      int latIndex = curLatStr.indexOf('.');
      int longIndex = curLongStr.indexOf('.');
      String latFile = String(curLatStr.substring(0,latIndex)) + String(curLatStr.substring(latIndex+1,curLatStr.length()-1));
      String longFile = String(curLongStr.substring(0,longIndex)) + String(curLatStr.substring(longIndex+1,curLongStr.length()-1));
      String coordsExt = latFile + longFile;
      sprintf(filename, "photos/pothole_%s.jpg", coordsExt.c_str());
      if (debug) ei_printf("INFO: saving %s to SD card...", filename);
      theSD.remove(filename);
      File myFile = theSD.open(filename, FILE_WRITE);
      myFile.write(img.getImgBuff(), img.getImgSize());
      myFile.close();
    } else if (debug) {
      Serial.println("failed to compress and save image, check that camera and SD card are connected properly");
    }
  }
  take_picture_count++;
  Serial.println("Image count incremented.");
}

/**
 * @brief Initialize camera
 */
void setup()
{
  CamErr err;
  Serial.println("Starting...");
  /* Open serial communications and wait for port to open */
  Serial.begin(115200);
  while (!Serial)
  {
      ; /* wait for serial port to connect. Needed for native USB port only */
  }

  /* Initialize SD */
  while (!theSD.begin()) 
  {
    /* wait until SD card is mounted. */
    Serial.println("Insert SD card.");
  }
  /* Connect LTE network */
  doAttach();
  int result;
  /* Activate GNSS device */
  result = Gnss.begin();
  assert(result == 0);
  /* Start positioning */
  result = Gnss.start();
  assert(result == 0);
  Serial.println("Gnss setup OK");
  // Wait for the modem to connect to the LTE network.
  Serial.println("Waiting for successful attach.");
  LTEModemStatus modemStatus = lteAccess.getStatus();

  while(LTE_READY != modemStatus) {
    if (LTE_ERROR == modemStatus) {
      Serial.println("An error has occurred. Shutdown and retry the network attach process after 1 second.");
      lteAccess.shutdown();
      sleep(1);
      doAttach();
    }
    sleep(1);
    modemStatus = lteAccess.getStatus();
  }
  Serial.println("attach succeeded.");
  // Set local time (not UTC) obtained from the network to RTC.
  RTC.begin();
  unsigned long currentTime;
  while(0 == (currentTime = lteAccess.getTime())) {
    sleep(1);
  }
  RtcTime rtc(currentTime);
  RTC.setTime(rtc);

  // Set certifications via a file on the SD card before connecting to the MQTT broker
  client.setCACert(AWS_CERT_CA);
  client.setCertificate(AWS_CERT_CRT);
  client.setPrivateKey(AWS_CERT_PRIVATE);
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  while(!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    sleep(3);
  }
  Serial.println("You're connected to the MQTT broker!");

  //camera starts continuously classifying video feed at 30fps
  //ei_pothole_camera_start_continuous(true);
}

void loop()
{
  //sleep(1); /* wait for one second to take still picture. */
  /* Check update. */
  if (Gnss.waitUpdate(-1)) {
    /* Get navData. */
    SpNavData navData;
    Gnss.getNavData(&navData);

    bool posFix = ((navData.posDataExist) && (navData.posFixMode != FixInvalid));
    //if position is fixed, start inferencing
    if (posFix) {
      Serial.println("Position is fixed.");
      String nmeaString = getNmeaGga(&navData);
      if (strlen(nmeaString.c_str()) != 0) {
        curLat = navData.latitude;
        curLong = navData.longitude;
        ei_pothole_camera_start_continuous(true);
      }
    } else {
      Serial.print("Position is not fixed.");
      Serial.print("  Number of sats: ");
      Serial.print(navData.numSatellites);
      Serial.println(".\n");
    }
  }

  if (numOfPubs >= MAX_NUMBER_OF_PUBLISH) {
    Serial.println("Publish end");
    // do nothing forevermore:
    for (;;)
      sleep(1);
  }
}

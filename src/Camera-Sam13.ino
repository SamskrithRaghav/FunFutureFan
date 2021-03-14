#include <ESP32Servo.h>
#include "esp_camera.h"
#include "fd_forward.h"
#include "WiFi.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model: We're using the AI Thinker model with ESP32-CAM
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// Wifi setup
const char* ssid = "ATTaj5393A";
const char* password = "?cpcs9eci7qz";
void startCameraServer();//wifi unused

mtmn_config_t mtmn_config = {0}; //global variable dec for cam
int detections = 0; //detection variable define

Servo myservo;  // create servo object to control a servo

/*
   Type: Utility function.
   Purpose: Setup the camera configuration in the ESP32-CAM module.
   Arguments: None
   Returns: Nothing
*/
void setup_camera() { //setup the camera
  camera_fb_t * frame1; //get the pic

  camera_config_t config; // configure the camera

  //configure the camera
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the brightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
  // MTMN is a combination of MTCNN and MobileNets
  // MTCNN: Multi-Task Convolutional Neural Network
  // MTCNN is used for face detection.
  mtmn_config = mtmn_init_config();
}

/*
   Type: Utility function.
   Purpose: Setup the Serial Monitor.
   Arguments: the Serial Monitor baud rate, and whether or not to debug the code
   Returns: Nothing
*/
void setup_serial(int baud_rate, bool debug_code) {
  Serial.begin(baud_rate);
  Serial.setDebugOutput(debug_code);
  Serial.println();
}

/*
   Type: Utility function.
   Purpose: Setup the servo and attach it to the ESP32.
   Arguments: servo_pin
   Returns: Nothing
*/
void setup_servo(int servo_pin) {//servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servo_pin); // attaches the servo on pin 18 to the servo object
}

/*
   Type: Utility function.
   Purpose: Convert the x cordinate given into an angle for the servo
   Arguments: x coordinate
   Returns: Nothing
*/
int calc_angle(int x)//calculate angle from coordinate
{
  float a;

  if (x <= 0) {
    a = 0;
  }
  else if (x >= 360) {
    a = 150;
  }
  else if (x == 180) {
    a = 90;
  }
  else if (0 < x < 180) {
    a = 90 + 30 - x * 0.1667 ;
  }
  else if (180 < x < 360) {
    a = 90 + x * 0.1667 - 30 ;
  }
  Serial.printf(" x = %d, NEW SERVO ANGLE + %d\n", x, (int) a);
  return (int) a;
}

/*
   Type: Utility function.
   Purpose: Make the servo do the sweep for whichever values are given.
   Arguments: lp, spos, epos
   Returns: Output to servo
*/
void sweep_servo(int lp, int spos, int epos) {//actual servo sweep command

  int pos;

  for ( ; lp-- > 0; ) {
    for (pos = spos; pos <= epos; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);    // tell servo to go to position in variable 'pos'
      delay(15);             // waits 15ms for the servo to reach the position
    }
    for (pos = epos; pos >= spos; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);    // tell servo to go to position in variable 'pos'
      delay(15);             // waits 15ms for the servo to reach the position
    }
  }
}

/*
   Type: Template function.
   Purpose: Call the functions, run the iteration loop, and send output to servo.
   Arguments: None
   Returns: Nothing
*/
void setup() {
  pinMode(33, OUTPUT);
  camera_fb_t * frame1;
  camera_config_t config;

  setup_serial(115200, true);  // setup the computer serial monitor
  setup_servo(12);  // setup the fan
  setup_camera();   // setup the camera

  while (true) {   // forever
    // put your main code here, to run repeatedly:
    box_t * box;
    camera_fb_t * frame;
    int startx, endx;

    startx = endx = 0;
    for (int i1 = 0; i1++ < 5; ) {
      Serial.printf("Iteration #:%d", i1);

      frame = esp_camera_fb_get(); // capture the image frame from the camera
      if (frame == NULL) {
        Serial.println("frame2 null; esp_camera_fb_get() FAILED!!!");
        setup_camera();
        continue;
      }
      else {
        Serial.println("frame2 NOT NULL; esp_camera_fb_get() PASSED!!!");
      }

      dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, frame->width, frame->height, 3);  // allocate memory for image analysis
      fmt2rgb888(frame->buf, frame->len, frame->format, image_matrix->item);  // convert the format to rgb
      esp_camera_fb_return(frame);  // release the frame memory
      box_array_t * boxes = face_detect(image_matrix, &mtmn_config);  // Use the AI to detect faces and return the (x,y) coordinates

      if (boxes == NULL) { // Found nobody
        Serial.println("AM I ALONE?");
        for (int i2 = 0; i2++ < 3;) {
          digitalWrite(33, LOW);
          delay(100);
          digitalWrite(33, HIGH);
          delay(100);
          //setup_servo(12);
          //sweep_servo(5, 0, 180);
        }
        continue;
      }  // I'm not alone!
      else if (boxes != NULL) {
        detections = detections + 1;
        Serial.printf("Faces detected %d times, %d \n ", detections, boxes->len);
        digitalWrite(33, HIGH);
        int i = 0;
        uint16_t p[4];
        for (i = 0; i < boxes->len; i++) {
          for (int j = 0; j < 4; j++) {
            p[j] = (uint16_t)boxes->box[i].box_p[j];
          }
          Serial.printf("%d %d %d %d \n", p[0], p[1], p[2], p[3]);
        }
        Serial.println("Loop begins! 6");

        //            dl_lib_free(boxes->score);
        //            dl_lib_free(boxes->box);
        //            dl_lib_free(boxes->landmark);
        //            dl_lib_free(boxes);

        if (startx == 0 || (p[0] < startx))
          startx = p[0];
        if (endx == 0 || (p[2] > endx))
          endx = p[2];
      }
      if (image_matrix != NULL) {
        dl_matrix3du_free(image_matrix);
        image_matrix = NULL;
      }
    }
    Serial.printf("Startx = %d; Endx = %d \n", startx, endx);
    // Switch the start/end because the image is mirrored!
    int endangle = calc_angle(startx);
    int startangle = calc_angle(endx);
    Serial.printf("Startangle = %d; Endangle = %d \n", startangle, endangle);
    if (startangle <= 0 || endangle >= 180 || startangle >= 180 || endangle <= 0) {
      startangle = 0;
      endangle = 180;
    }

    setup_servo(12);
    myservo.write(startangle);
    if (startangle == 0 && endangle == 180) {
     sweep_servo(1, startangle, endangle);
    } else {
     sweep_servo(6, startangle, endangle);
    }
    
    // myservo.detach(12);
    // myservo.detach(); // detaches the servo on pin 18 to the servo object
  } // end While
}

/*
   Type: Template function.
   Purpose: None; Necessary to run program because Arduino IDE checks for it. Loop implemented in void setup.
   Arguments: None.
   Returns: Nothing.
*/
void loop() {
  // put your main code here, to run repeatedly:
}

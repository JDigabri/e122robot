#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <Ultrasonic.h>
//#include <WemosInit.h>
#include <Servo.h>
#include "SSD1306.h"
#include <Wire.h>
#include <ESP8266WiFi.h>
#include "PubSubClient.h"
#include "WiFiManager.h"
#include "math.h"

//MQTT Communication associated variables
char payload_global[100];
boolean flag_payload;

//Change the pin definitions below-My 2019 & 2020 motor is D1 R1 -- Use pins D0 and D2 for motor for D1R1
#define motor1pin D0   //Driver Side Motor  D0   
#define motor2pin D2   //Passenger Side Motor D2         

//MQTT Setting variables
const char* mqtt_server = "155.246.62.110";   //MQTT Broker(Server) Address
const char* MQusername = "jojo";              //MQTT username
const char* MQpassword = "hereboy";           //MQTT password
//const char* MQtopic = "louis_lidar1";         //MQTT Topic for Arena_1 (EAS011 - South)
const char* MQtopic = "louis_lidar2";       //MQTT Topic for Arena_2 (EAS011 - North)
const int mqtt_port = 1883;                   //MQTT port#

//Stevens WiFi Setting variables
const char* ssid = "Stevens-IoT";             //Stevens Wi-Fi SSID (Service Set IDentifier)
const char* password = "nMN882cmg7";          //Stevens Wi-Fi Password

//WiFi Define
WiFiClient espClient;
PubSubClient client(espClient);
int distance_in_d, distance_in_c, distance_in_p;  // create 3 integer type variables named distance_in_d for driver, distance_in_c for center and distance_in_p for passenger

Ultrasonic ultrasonic_driver(D9, D6);  // Create 3 instances for an ultrasonic sensor (trigger pin, echo pin)
Ultrasonic ultrasonic_center(D8, D5);
Ultrasonic ultrasonic_pass(D10, D7);
Servo motor1;  // Creates a servo object called "motor1"
Servo motor2;  // Creates a servo object called "motor2"

SSD1306  display(0x3C, D14, D15); //Address set here 0x3C that I found in the scanner, and pins defined as D14 (SDA/Serial Data), and D15 (SCL/Serial Clock).
// 0x3c is the I2C address from slide #7 Right hand side of Slide #8 will be used when LIDAR is in use

//Prototype the function names
void forward();   //Subroutine to have the robot move forward
void align_left(); //Subroutine to have the robot redirct left while moving forward
void align_right(); //Subroutine to have the robot redirect right while moving forward
void reverse();   //Subroutine to have the robot move backwards
void right();     //Subroutine to have the robot turn right
void left();      //Subroutine to have the robot turn left
void halt();      //Subroutine to have the robot stop

void setup_wifi() {
  delay(10);
  // We start by connecting to a Stevens WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    payload_global[i] = (char)payload[i];
  }
  payload_global[length] = '\0';
  flag_payload = true;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQusername, MQpassword)) {
      client.subscribe(MQtopic);              //EAS011 South - "louis_lidar1"       EAS011 North - "louis_lidar2"
    } else {
      // Wait 2.5 seconds before retrying
      delay(2500);
    }
  }
}
int loop_c = 0;
int start_ = 0;
int x_t = 1400;
int y_t = 150;
int target = 1;
void setup() {
  Serial.begin(115200);
  // tells the ESP8266 to set up the pins to act as servos
  setup_wifi();
  delay(1000);
  Serial.println("Wemos POWERING UP ......... ");
  client.setServer(mqtt_server, mqtt_port);   //This 1883 is a TCP/IP port number for MQTT
  client.setCallback(callback);
  motor1.attach(motor1pin);  // D0 will be a servo motor pin – Driver Side
  motor2.attach(motor2pin);  // D2 will be a servo motor pin – Passenger Side
  motor1.write(90); // Turns motor 1 off
  motor2.write(90); // Turns motor 2 off
  display.init();
  display.flipScreenVertically();    // Needed when OLED pin-outs are on top.
  display.display();

}

void loop() {
  //subscribe the data from MQTT server
  if (!client.connected()) {
    halt();
    Serial.print("...");
    reconnect();
  }
  client.loop();

  String payload(payload_global);
  int testCollector[10];
  int count = 0;
  int prevIndex, delimIndex;

  prevIndex = payload.indexOf('[');
  while ( (delimIndex = payload.indexOf(',', prevIndex + 1) ) != -1) {
    testCollector[count++] = payload.substring(prevIndex + 1, delimIndex).toInt();
    prevIndex = delimIndex;
  }
  delimIndex = payload.indexOf(']');
  testCollector[count++] = payload.substring(prevIndex + 1, delimIndex).toInt();

  int x, y;
  //Robot location x,y from MQTT subscription variable testCollector
  delay(200);
  /*while (x == testCollector[0] && y == testCollector[1]) { 
    halt();
    delay(500);
  }
  */
  x = testCollector[0];
  y = testCollector[1];
  Serial.println("Coords  X:" + String(x) + "  Y:" + String(y));
  display.clear();  // Clears Screen To pepeare for next action
  display.setFont(ArialMT_Plain_10);  //Sets Font
  display.drawString(0, 0, "ENGR-122-K G-3");  // Header Always Present
  display.drawString(0, 10, "Coords  X:" + String(x) + "  Y:" + String(y));
  distance_in_d = ultrasonic_driver.read(INC);   // read driver side sensor
  distance_in_c = ultrasonic_center.read(INC);  // read front center sensor
  distance_in_p = ultrasonic_pass.read(INC);    // read passenger side sensor
  Serial.print("Distance Driver(D8,D5) Center(D9,D6) Pass(D10,D7):   ");  // Print a caption
  Serial.print(distance_in_d);
  Serial.print(" in. ");
  Serial.print(distance_in_c);
  Serial.print(" in. ");
  Serial.print(distance_in_p);
  Serial.println(" in. ");
  int driver = distance_in_d;
  int center = distance_in_c;
  int passanger = distance_in_p;
  String x_str = "Dist.(INC) D-" + String(driver) + " C-" + String(center) + " P-" + String(passanger); //Creates string to print
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, x_str); //Displays distances of all sensors
  display.display();
  // Vector Algebra
  int vx_p, vy_p, vx_t, vy_t, dot, v_p;
  float v_t_m, v_p_m, v_ang, t_ang, p_ang, po_ang;
  int x_i, y_i;
  if (start_ == 0) { // Starting conditions for x_i and y_i
    int x_i, y_i = -1000;
    start_++;
  }
  if ( loop_c == 5 || start_ == 1) { // Every 5 loops of function robot checks its position
    if (x_i != -1000 || y_i != -1000) { // Starts Vector Algebra after 
      //if (x_i != x && y_i != y){
      vx_p = x - x_i;
      vy_p = y - y_i;
      vx_t = x_t - x;
      vy_t = y_t - y;
      dot = (vx_p * vx_t) + (vy_p * vy_t);
      v_p_m = sqrt(sq(vx_p) + sq(vy_p));
      v_t_m = sqrt(sq(vx_t) + sq(vy_t));
      v_ang = acos(dot / (v_p_m * v_t_m));
      v_p = (vx_p * vy_t)-(vy_p * vx_t);
      //t_ang = atan2(vy_t,vx_t);
      //p_ang = atan2(vy_p,vx_p);
      //po_ang = (p_ang - M_PI);
      //if (po_ang < 0){
      //  po_ang = (p_ang + M_PI);
      //}
      //}
    }
    //(M_PI/2)
    x_i = x;
    y_i = y;
    if ((x >= (x_t - 100) && x <= (x_t + 100)) && (y >= (y_t - 100) && y <= (y_t + 100))) { // Target Reached!
      display.setFont(ArialMT_Plain_10);
      display.clear();
      display.drawString(0, 30, "Stopped-------D-90   P-90"); // Displays motor speeds and action
      display.drawString(0, 40, "Reached Destination"); // Displays motor speeds and action
      display.display();
      halt();
      delay(2000);
      target++;
      if (target == 2){
        x_t = 650;
        y_t = 150;
      }
      else if (target == 3) {
        x_t = 150;
        y_t = 150;
      }
      else if (target == 4) {
        x_t = 2000;
        y_t = 700;
      }
    }
    else   if (distance_in_c <= 4) { // dodges obstecals and turns toward target
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 30, "Stopped-------D-90   P-90"); // Displays motor speeds and action
    display.display();
    halt();
    delay(100);
    if (v_p > 0) { // Turns left
      display.drawString(0, 40, "Left---------D-110  P-70"); // Displays motor speeds and action
      display.display();
      left();
      delay(400);
    }
    else if (v_p < 0) { // Turns right
      display.drawString(0, 40, "right---------D-70  P-110"); // Displays motor speeds and action
      display.display();
      right();
      delay(400);
    }
    }
    else if (v_ang >= (M_PI / 2) && v_p > 0) { // Robot is to the left of the target
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Stopped-------D-90   P-90"); // Displays motor speeds and action
      display.display();
      halt();
      delay(100);
      display.drawString(0, 40, "Left---------D-120  P-60"); // Displays motor speeds and action
      display.display();
      left();
      delay(400);
      forward();
      delay(400);
    }
    else if (v_ang >= (M_PI / 2) && v_p < 0) { //Robot is to the right of the target
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Stopped-------D-90   P-90"); // Displays motor speeds and action
      display.display();
      halt();
      delay(100);
      display.drawString(0, 40, "Right---------D-60  P-120"); // Displays motor speeds and action
      display.display();
      right();
      delay(400);
      forward();
      delay(400);
    }
    /*
    else if (v_ang < (M_PI / 2) && v_p > 0) { // Robot is headed left of the target
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Alining------D-70  P-63"); // Displays motor speeds and action
      display.display();
      align_left();
    }
    else if (v_ang < (M_PI / 2) && v_p < 0) { // Robot is headed right of the target
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Alining------D-60  P-70"); // Displays motor speeds and action
      display.display();
      align_right();
    }
    */
    else { // Means robot is headed for target
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Forward------D-60  P-58"); // Displays motor speeds and action
      display.display();
      forward();
    }
    loop_c = 0;
    start_++;
  }
  else if(start_ == 0){ // Moves Robot at Start
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 30, "Forward------D-60  P-58"); // Displays motor speeds and action
    display.display();
    forward();
  }
  /* if (distance_in_c > 4) { //
    if(distance_in_p > 6) { // Decides if it is nessesary to go closer to the wall
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Alining------D-50  P-60"); // Displays motor speeds and action
      display.display();
      motor1.write(50);
      motor2.write(60);
      }
    else if(distance_in_p < 3) { // Decides if it is nessesary to go farther from wall
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Alining------D-60  P-53"); // Displays motor speeds and action
      display.display();
      motor1.write(60);
      motor2.write(53);

    }
    else{ // Goes straight
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 30, "Forward------D-60  P-58"); // Displays motor speeds and action
      display.display();
      forward();
    }
    }*/
  /*
    else { // Stops if USC aren't working
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 30, "Stopped       0   0"); // Displays motor speeds and action
    display.display();
    halt();
    }
  */
  loop_c++;
}
// Subroutine listings
void forward()  // Has the motors go forward- even for 2020
{
  //motor1.write(70); // Driver motor D0 - furthest from 90 goes faster. 60 is faster than 80
  //motor2.write(68); // Passenger motor D2
  motor1.write(107);
  motor2.write(102);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void align_left() // Has the motors turn Robot left as it goes forward
{
  motor1.write(112);
  motor2.write(102);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void align_right() // Has the motors turn Robot right as it goes forward
{
  motor1.write(102);
  motor2.write(112);
  
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void reverse()  // Has the motors go in reverse - even for 2020
{
  motor1.write(68);  //left  motor D0 - furthest from 90 goes faster. 120 is faster than 100
  motor2.write(70);  //right motor D2 - 100 is slow but good for testing; 100 is much slower than 80 is fast
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void halt() // Has the motors stop
{
  motor1.write(90); //Turns motor1 off
  motor2.write(90); //Turns motor2 off
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void left() // Has the motors turn left
{
  motor1.write(110);
  motor2.write(70);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void right() // Has the motors turn right
{
  motor1.write(70);
  motor2.write(110);
}

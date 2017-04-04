/*

 Access digital and analog pins of the board
 through REST calls. It demonstrates how you
 can create your own API when using REST style
 calls through the browser.

 Possible commands created in this shetch:

 * "/arduino/digital/13"     -> digitalRead(13)
 * "/arduino/digital/13/1"   -> digitalWrite(13, HIGH)
 * "/arduino/analog/2/123"   -> analogWrite(2, 123)
 * "/arduino/analog/2"       -> analogRead(2)
 * "/arduino/mode/13/input"  -> pinMode(13, INPUT)
 * "/arduino/mode/13/output" -> pinMode(13, OUTPUT)

 This example code is part of the public domain

 created 28 February 2017
 by Andrea Cannistrá
 modified 10 March 2017
 by Sergio Tomasello
 */

#include <WiFiLink.h>

char ssid[] = "yourNetwork";      // your network SSID (name)
char pass[] = "yourPassword";   // your network password

int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(80);
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Check if communication with wifi module has been established
  if (WiFi.status() == WL_NO_WIFI_MODULE_COMM) {
    Serial.println("Communication with WiFi module not established.");
    while (true); // don't continue:
  }

  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid,pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
}

void loop() {
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    // Process request
    process(client);
    // Close connection and free resources.
    delay(1);
    client.stop();
  }

  delay(50); // Poll every 50ms
}

void process(WiFiClient client) {
  // read the command//
  if(listen_service(client, "arduino")){
    String command = client.readStringUntil('/');
    // is "digital" command?
    if (command == "digital") {
      digitalCommand(client);
    }

    // is "analog" command?
    else if (command == "analog") {
      analogCommand(client);
    }

    // is "mode" command?
    else if (command == "mode") {
      modeCommand(client);
    }

    // unknow command
    else{
      client.println("HTTP/1.1 200 OK");
      client.println();
      client.print(F("error: Unknow "));
      client.print(command);
      client.print(F(" command!"));
      return;
    }
  }

}

void digitalCommand(WiFiClient client) {
  int pin, value;

  // Read pin number
  String pinNumber;
  char c = client.read();
  // If the next character is a '/' it means we have an URL
  // with a value like: "/digital/13/1"
  // If the next character is a ' ' it means we have an URL
  // with a value like: "/digital/13"

  while(c != ' ' && c != '/'){
    pinNumber+=c;
    c = client.read();
  }
  pin = pinNumber.toInt();

  if (c == '/') {
    value = client.parseInt();
    digitalWrite(pin, value);
  }
  else {
    value = digitalRead(pin);
  }

  // Send feedback to client
  client.println("HTTP/1.1 200 OK");
  client.println();
  client.print(F("Pin D"));
  client.print(pin);
  client.print(F(" set to "));
  client.println(value);

}

void analogCommand(WiFiClient client) {
  int pin, value;
  // Read pin number
  String pinNumber;
  char c = client.read();

  // If the next character is a '/' it means we have an URL
  // with a value like: "/analog/5/120"
  // If the next character is a ' ' it means we have an URL
  // with a value like: "/analog/13"
  while(c != ' ' && c != '/'){
    pinNumber+=c;
    c = client.read();
  }
  pin = pinNumber.toInt();
  // If the next character is a '/' it means we have an URL
  // with a value like: "/analog/5/120"
  if (c == '/') {
    // Read value and execute command
    String analogValue;
    char c= client.read();
    while(c != ' ' && c != '/'){
      analogValue+=c;
      c = client.read();
    }
    value = analogValue.toInt();
    analogWrite(pin, value);

    // Send feedback to client
    client.println("HTTP/1.1 200 OK");
    client.println();
    client.print(F("Pin D"));
    client.print(pin);
    client.print(F(" set to analog "));
    client.println(value);

  }
  else {
    // Read analog pin
    value = analogRead(pin);

    // Send feedback to client
    client.println("HTTP/1.1 200 OK");
    client.println();
    client.print(F("Pin A"));
    client.print(pin);
    client.print(F(" reads analog "));
    client.println(value);
  }
}

void modeCommand(WiFiClient client) {
  int pin;

  // Read pin number
  String pinNumber;
  char c = client.read();
  while(c != ' ' && c != '/'){
    pinNumber+=c;
    c = client.read();
  }
  pin = pinNumber.toInt();
  // If the next character is not a '/' we have a malformed URL
  if (c != '/') {
    client.println("HTTP/1.1 200 OK");
    client.println();
    client.println(F("error"));
    return;
  }

  String mode = client.readStringUntil(' ');

  if (mode == "input") {
    pinMode(pin, INPUT);
    // Send feedback to client
    client.println("HTTP/1.1 200 OK");
    client.println();
    client.print(F("Pin D"));
    client.print(pin);
    client.print(F(" configured as INPUT!"));
    return;
  }

  if (mode == "output") {
    pinMode(pin, OUTPUT);
    // Send feedback to client
    client.println("HTTP/1.1 200 OK");
    client.println();
    client.print(F("Pin D"));
    client.print(pin);
    client.print(F(" configured as OUTPUT!"));
    return;
  }

  client.println("HTTP/1.1 200 OK");
  client.println();
  client.print(F("error: invalid mode "));
  client.print(mode);

}


bool listen_service(WiFiClient client, String service){

  //check service
  String currentLine="";
  while (client.connected()) {
    if (client.available()){
      char c= client.read();
      currentLine +=c;
      if (c == '\n') {
        client.println("HTTP/1.1 200 OK");
        client.println();
        return 0;         
      }
      else if (currentLine.endsWith(service+"/")){
        return 1;
      }
    }
  }

}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

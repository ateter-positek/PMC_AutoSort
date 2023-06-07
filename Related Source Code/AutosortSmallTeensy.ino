/*
 Name:		AutoSort_Simple.ino
 Created:	1/26/2023 10:31:34 AM
 Author:	ateter
*/

#include <SPI.h>
#include <Ethernet.h>

int lastHeartbeatTime = 0;
int heartBeatCount = 0;
int garmentCount = 0;
int sendPLC = 0;
int loopCount = 0;
int heartbeatLast = 0;
String sendPLCdata = "";
String postData = "";

bool awaitApiResponse = false;
String apiResponseString = 0;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10, 10, 10, 250);
EthernetClient client;

void setup() {
    Ethernet.begin(mac, ip);
    delay(1000);
    Serial.begin(115200);
    Serial2.begin(115200);
    delay(500);
    checkClientConnect();
    Serial.println("-------------------BEGIN--------------------");
}

void loop() {

    readPLCSerial(); //check for new serial from PLC
    if (sendPLC != 0) //if we have a new send trigger from PLC send the buffered data
    {
        Serial.println(sendPLCdata);
        Serial2.println(sendPLCdata);
        String sendPLCdata = "";
    }
    checkClientConnect(); //monitor the API connection and log erros / reconnect as necessary

    apiResponse(); //check to see if we have a response

    if (heartBeatCount != 0 && !awaitApiResponse) //New heartbeat Post API call
    {
        callAPI();
    }
}

void readPLCSerial() {
    int hbCount = 0;
    while (Serial2.available()) {
        char inChar = Serial2.read();
        if (inChar == '[') {
            String inputString = "";
            while (inChar != ']') {
                if (Serial2.available()) {
                    inChar = Serial2.read();
                    inputString += inChar;
                }
            }
            inputString.remove(inputString.length() - 1); // remove the closing ']'
            

            switch (inputString.toInt())
            { 
            case 01:
                hbCount++;
                break;
            case 02:
                garmentCount++;
                break;
            case 03:
                hbCount++;
                garmentCount++;
                break;
            case 20:
                sendPLC++;
                break;
            case 21:
                sendPLC++;
                hbCount++;
                break;
            case 22:
                sendPLC++;
                garmentCount++;
                break;
            case 23:
                sendPLC++;
                hbCount++;
                garmentCount++;
                break;
            }          
            if (hbCount > 0 && ((millis() - lastHeartbeatTime) > 100))
            {
                heartBeatCount++;
                lastHeartbeatTime = millis();
            }
        }
    }
}

void callAPI()
{
    if (client.available())
    {
        bool garmentDetect;
        if (garmentCount > 0) { garmentDetect = true; }
        String  postData = "{\"Garment\":\"";
                postData += garmentDetect;
                postData += "\",\"Heartbeat\":\"";
                postData += millis() - heartbeatLast;
                postData += "\"}";
                heartbeatLast = millis();
        client.println("POST /api/autosortpost HTTP/1.1");
        client.println("Host: 10.10.10.10");
        client.println("User-Agent: Arduino/1.0");
        client.println("Content-Type: application/json");
        client.println("Connection: keep-alive");
        client.println("Content-Length: " + String(postData.length()));
        
        client.println();
        client.println(postData);
        client.println();
        Serial.print("Sent Data: ");
        Serial.println(postData);

        awaitApiResponse = true;
    }
    else
    {
        Serial.println("API client connection unavailable while trying to send");
    }
}


void apiResponse()
{
    String response = "";
    int timeout = 0;

    while (client.connected())
    {
        if (client.available())
        {
            // read an incoming byte from the server and print it to serial monitor:
            response += client.read();
        }
    }

    Serial.println(response);
    if (response.length() > 0)
    {
        // check if Content-Length is greater than 0
        int contentLengthStart = response.indexOf("Content-Length: ") + 16;
        int contentLengthEnd = response.indexOf("\n", contentLengthStart);
        String contentLengthString = response.substring(contentLengthStart, contentLengthEnd);
        int contentLength = contentLengthString.toInt();

        if (contentLength > 0) {
            // find the start and end of the post body
            int postBodyStart = response.indexOf("\"") + 1;
            int postBodyEnd = response.indexOf("\"", postBodyStart);
            String postBody = response.substring(postBodyStart, postBodyEnd);

            if (postBody != "$$$$")
            {
                apiResponseString = response;
                awaitApiResponse = false;
            }
            Serial.print("SW Response: ");
            Serial.println(postBody);
        }
    }
}

void checkClientConnect() 
{
        while (!client.available())
        {
            if (client.connect("10.10.10.10", 16000)) {
                Serial.println("!!!!!!!!!! Socket Client Re-Connected !!!!!!!!!!");
            }
            else {
                Serial.println("!!!!!!!!!! Socket Client Connection Failed !!!!!!!!!!");
                delay(1000);
            }
            
        }
        
 }
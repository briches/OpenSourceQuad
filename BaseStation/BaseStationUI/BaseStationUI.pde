import processing.serial.*;
import java.text.*;
import java.util.*;
import java.io.*;

PImage background; // Background image
PFont mono; // Font
String time; // Prog time
String flightTimeStr; // Flight millis
byte flightTime1; // Flight time from start, unsigned long = 4 bytes
byte flightTime2;
byte flightTime3;
byte flightTime4;
boolean connection = false; // Connected to copter, or no.

// Button states
boolean overSendButton;
boolean haveClicked = false;

// txPacket
byte[] txPacket = new byte[5];
byte[] rxPacket = new byte[5];
int startByte = 0, mID = 1, data1 = 2, data2 = 3, data3 = 4;
int[] pGain = new int[3];
int[] iGain = new int[3];
int[] dGain = new int[3];

// Display parameters
int[] fillColors = new int[3];

int windowX = 800;
int windowY = 600;

int bannerY = 60;
int bannerX = windowX;

int txPacketLX = 240;
int txPacketLY = 35;
int txPacketSX = 20;
int txPacketSY = 30;

int sendButtonX = 285;
int sendButtonY = 105;
int sendButtonSize =  25;
int sendButtonRadius = 0;

Serial myPort;

/*================================================================================
     Processing doesnt have structs.
     -----------------------------------------------------------------------------*/
static class PID
{
    static double setP, setI, setD;
    static double rateP, rateI, rateD;
    static boolean nested, single;
}

/*================================================================================
     Setup of UI window
     -----------------------------------------------------------------------------*/
void setup()
{
    //Set up the serial port
    println(Serial.list());
    myPort = new Serial(this, Serial.list()[0], 19200);
    print("Connected to serial port: ");
    println(Serial.list()[0]);


    // Set up window
    size(windowX, windowY);
    ellipseMode(CENTER);
    strokeWeight(2);
    stroke(0);

    // Load Fonts
    mono = loadFont("Monospaced.bold-20.vlw");
    fillColors[0] = 255;
    fillColors[1] = 255;
    fillColors[2] = 255;
}
/*================================================================================
     Draw() - main loop in processing
     -----------------------------------------------------------------------------*/
void draw()
{
    // Set up the title text
    background(0);
    fill(90,90,90, 120);
    rect(0, 0, bannerX, bannerY);
    fill(255);
    textFont(mono, 30);
    text("OSQ Base Station", 15, 25);

    // Setup other text.
    textFont(mono, 15);
    
    text("Keyboard Options:", 25, 150);
    text("X : Disarm", 25, 175);
    text("S : Start", 25, 200);
    text("B : Broadcast", 25, 225);
    text("P : Set P", 25, 250);
    text("I : Set I", 25, 275);
    text("D : Set D", 25, 300);
    text("R : Reset Attitude", 25, 325);
    
    // Display prog time and flight time
    displayTimes();

    // Display connection status;
    checkConnection();

    // Display PID parameters
    //displayPID();

    // Check for over buttons
    checkOverButton();
    
    // Display buttons
    fill(fillColors[0], fillColors[1], fillColors[2]);
    rect(sendButtonX, sendButtonY, sendButtonSize, sendButtonSize, sendButtonRadius);
    
    // Reset the fill
    fillColors[0] = 255;
    fillColors[1] = 255;
    fillColors[2] = 255;

    // Display txPacket
    fill(fillColors[0], fillColors[1], fillColors[2]);
    displaytxPacket();
}  
/*================================================================================
     End Draw!!
     -----------------------------------------------------------------------------*/

/*================================================================================
     Serial Event: Used to RX data
     -----------------------------------------------------------------------------*/
void serialEvent(Serial myPort)
{
    connection = true;
    if(myPort.available() >= 5)
    {
        rxPacket[0] = byte(myPort.read());
        if(rxPacket[0] == 0xFF)
        {
            rxPacket[1] = byte(myPort.read());
            rxPacket[2] = byte(myPort.read());
            rxPacket[3] = byte(myPort.read());
            rxPacket[4] = byte(myPort.read());
            
            flightTime1 = rxPacket[1];
            flightTime2 = rxPacket[2];
            flightTime3 = rxPacket[3];
            flightTime4 = rxPacket[4];
            
            println(flightTime1);
            println(flightTime2);
            println(flightTime3);
            println(flightTime4);
        }
        else
        {
            for(int i = 0; i<= myPort.available(); i++)
            {
                rxPacket[0] = byte(myPort.read());
                println(rxPacket[0]);
            }
        }
    }
}

/*================================================================================
     Scan Keyboard Input
     -----------------------------------------------------------------------------*/
void keyPressed()
{
    println(key);
    
    switch(key)
    {
        case 'X':
        case 'x':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x00);
            txPacket[2] = byte(0x00);
            txPacket[3] = byte(0x00);
            txPacket[4] = byte(0x00);
            break;
        case 'S':
        case 's':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x02);
            txPacket[2] = byte(0x00);
            txPacket[3] = byte(0x00);
            txPacket[4] = byte(0x00);
            break;
        case 'B':
        case 'b':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x03);
            txPacket[2] = byte(0x00);
            txPacket[3] = byte(0x00);
            txPacket[4] = byte(0x00);
            break;
        case 'P':
        case 'p':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0C);
            txPacket[2] = byte(pGain[2]);
            txPacket[3] = byte(pGain[1]);
            txPacket[4] = byte(pGain[0]);
            break;
        case 'I':
        case 'i':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0D);
            txPacket[2] = byte(iGain[2]);
            txPacket[3] = byte(iGain[1]);
            txPacket[4] = byte(iGain[0]);
            break;
        case 'D':
        case 'd':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0E);
            txPacket[2] = byte(dGain[2]);
            txPacket[3] = byte(dGain[1]);
            txPacket[4] = byte(dGain[0]);
            break;
            
        default:
            // Hell yeah
            break;
    }
            
}

/*================================================================================
     Button Actions and menu stuff abouts here
     -----------------------------------------------------------------------------*/
void mousePressed()
{
    // Check states
    if (overSendButton && mouseButton == LEFT)
    {
        println("You left-clicked the button");
        fillColors[0] = 255;
        fillColors[1] = 0;
        fillColors[2] = 0;
        
        // Send dat txPacket, yo
        sendtxPacket();
    }
    if (overSendButton && mouseButton == RIGHT)
    {
        println("Why would you right click this, what do you want.");
        fillColors[0] = 0;
        fillColors[1] = 255;
        fillColors[2] = 0;
    }
}

/*================================================================================
     Send off the txPacket
     -----------------------------------------------------------------------------*/
void sendtxPacket()
{
    for(int i = 0; i<5; i++)
    {
        myPort.write(txPacket[i]);
    }
}

/*================================================================================
     Over the txPacket yo?
     -----------------------------------------------------------------------------*/
void checkOverButton()
{
    // Check for send button
    if (mouseX > sendButtonX && mouseX < sendButtonX + sendButtonSize &&
        mouseY > sendButtonY && mouseY < sendButtonY + sendButtonSize)
    {
        overSendButton = true;
    }
    else
    {
        overSendButton = false;
    }
    
            
}

/*================================================================================
     Print dat txPacket
     -----------------------------------------------------------------------------*/
void displaytxPacket()
{
    //startByte = 0, mID = 1, data1 = 2, data2 = 3, data3 = 4
    text("Outgoing txPacket Contents: ", 25, 90);
    text("Send",280,90);
    fill(90, 90, 90, 120);
    rect(25, 100, txPacketLX, txPacketLY);
    fill(255);
    text(hex(txPacket[0]), 30, 120);
    text(",", 65, 120);
    text(hex(txPacket[1]), 75, 120);
    text(",", 110, 120);
    text(hex(txPacket[2]), 120, 120);
    text(",", 155, 120);
    text(hex(txPacket[3]), 165, 120);
    text(",", 200, 120);
    text(hex(txPacket[4]), 210, 120);
}

/*================================================================================
     What time is it
     -----------------------------------------------------------------------------*/
void displayTimes()
{
    // Display current time.
    time = str(hour()) + ":" + str(minute()) + ":" + str(second()); 
    text(time, 15, 50);

    // Display flight time, from start.
    flightTimeStr = str((int(flightTime1)*16777216) + (int(flightTime2)*65536) + (int(flightTime3)*256) + int(flightTime4));
    text("Flight time: ", 550, 50);
    text(flightTimeStr, 700, 50);
}

/*================================================================================
     Connected? Print it.
     -----------------------------------------------------------------------------*/
void checkConnection()
{
    text("Connection Status: ", 496, 25);
    text(str(connection), 700, 25);
}



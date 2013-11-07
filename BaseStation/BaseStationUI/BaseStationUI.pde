import processing.serial.*;
import java.text.*;
import java.util.*;
import java.io.*;

PImage background; // Background image
PFont mono; // Font
String time; // Prog time
char[] flightTime = new char[7]; // Flight time from start
boolean connection = false; // Connected to copter, or no.
int[] serialSelection = new int[3];

// Button states
boolean overSendButton;
boolean haveClicked = false;

// Packet
byte[] packet = new byte[5];
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

int packetLX = 240;
int packetLY = 35;
int packetSX = 20;
int packetSY = 30;

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

    // Display packet
    fill(fillColors[0], fillColors[1], fillColors[2]);
    displayPacket();
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
            packet[0] = byte(0xFF);
            packet[1] = byte(0x00);
            packet[2] = byte(0x00);
            packet[3] = byte(0x00);
            packet[4] = byte(0x00);
            break;
        case 'S':
        case 's':
            packet[0] = byte(0xFF);
            packet[1] = byte(0x02);
            packet[2] = byte(0x00);
            packet[3] = byte(0x00);
            packet[4] = byte(0x00);
            break;
        case 'B':
        case 'b':
            packet[0] = byte(0xFF);
            packet[1] = byte(0x03);
            packet[2] = byte(0x00);
            packet[3] = byte(0x00);
            packet[4] = byte(0x00);
            break;
        case 'P':
        case 'p':
            packet[0] = byte(0xFF);
            packet[1] = byte(0x0C);
            packet[2] = byte(pGain[2]);
            packet[3] = byte(pGain[1]);
            packet[4] = byte(pGain[0]);
            break;
        case 'I':
        case 'i':
            packet[0] = byte(0xFF);
            packet[1] = byte(0x0D);
            packet[2] = byte(iGain[2]);
            packet[3] = byte(iGain[1]);
            packet[4] = byte(iGain[0]);
            break;
        case 'D':
        case 'd':
            packet[0] = byte(0xFF);
            packet[1] = byte(0x0E);
            packet[2] = byte(dGain[2]);
            packet[3] = byte(dGain[1]);
            packet[4] = byte(dGain[0]);
            break;
            
        default:
            // Hell yeah
            break;
    }
            
}

/*================================================================================
         // Button Actions and menu stuff abouts here
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
        
        // Send dat packet, yo
        sendPacket();
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
         Send off the packet
         -----------------------------------------------------------------------------*/
void sendPacket()
{
    for(int i = 0; i<5; i++)
    {
        myPort.write(packet[i]);
        println(packet[i]);
    }
}

/*================================================================================
         Over the packet yo?
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
         Print dat packet
         -----------------------------------------------------------------------------*/
void displayPacket()
{
    //startByte = 0, mID = 1, data1 = 2, data2 = 3, data3 = 4
    text("Outgoing Packet Contents: ", 25, 90);
    text("Send",280,90);
    fill(90, 90, 90, 120);
    rect(25, 100, packetLX, packetLY);
    fill(255);
    text(hex(packet[0]), 30, 120);
    text(",", 65, 120);
    text(hex(packet[1]), 75, 120);
    text(",", 110, 120);
    text(hex(packet[2]), 120, 120);
    text(",", 155, 120);
    text(hex(packet[3]), 165, 120);
    text(",", 200, 120);
    text(hex(packet[4]), 210, 120);
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
    getFlightTime(flightTime);
    String myString = new String(flightTime);
    text("Flight time: ", 550, 50);
    text(myString, 700, 50);
}

/*================================================================================
         Connected? Print it.
         -----------------------------------------------------------------------------*/
void checkConnection()
{
    text("Connection Status: ", 496, 25);
    text(str(connection), 700, 25);
}

/*================================================================================
         What time is it over there
         -----------------------------------------------------------------------------*/
void getFlightTime(char[] flightTime)
{
    // Lol function prototypes, who needs those.

    flightTime[0] = 'h';
    flightTime[1] = 'e';
    flightTime[2] = 'l';
    flightTime[3] = 'l';
    flightTime[4] = 'o';
    flightTime[5] = '!';
    flightTime[6] = '!';
}

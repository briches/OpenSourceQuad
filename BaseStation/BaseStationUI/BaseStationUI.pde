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


// Packet
char[] packet = new char[5];
int startByte = 0, mID = 1, data1 = 2, data2 = 3, data3 = 4;

// Display parameters
int windowX = 800;
int windowY = 600;

int bannerY = 60;
int bannerX = windowX;

int packetLX = 240;
int packetLY = 35;
int packetSX = 20;
int packetSY = 30;




Serial myPort;

static class PID
{
    static double setP, setI, setD;
    static double rateP, rateI, rateD;
    static boolean nested, single;
}


void setup()
{
    //Set up the serial port
    println(Serial.list());
    myPort = new Serial(this, Serial.list()[0], 19200);
    print("Connected to serial port: ");
    println(Serial.list()[0]);
    
    
    // Set up window
    size(windowX,windowY);
    ellipseMode(CENTER);
    strokeWeight(2);
    stroke(0);
    
    // Load Fonts
    mono = loadFont("Monospaced.bold-20.vlw");
    
}

void draw()
{
    // Set up the title text
    background(0);
    fill(90,90,90,120);
    rect(0,0,bannerX, bannerY);
    fill(255);
    textFont(mono,30);
    text("OSQ Base Station", 15, 25);
    
    // Setup other text.
    textFont(mono,15);
    
    // Display prog time and flight time
    displayTimes();
    
    // Display connection status;
    checkConnection();
    
    // Display PID parameters
    displayPID();
    
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFF;
    packet[3] = 0xFF;
    packet[4] = 0xFF;
    
    // Display packet
    displayPacket();
    
}  
    
void displayPacket()
{
    //startByte = 0, mID = 1, data1 = 2, data2 = 3, data3 = 4
    text("Outgoing Packet Contents: ", 25, 90);
    fill(90,90,90,120);
    rect(25,100,packetLX, packetLY);
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

void displayPID()
{
    
}

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

void checkConnection()
{
    text("Connection Status: ", 496, 25);
    text(str(connection), 700, 25);
}

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

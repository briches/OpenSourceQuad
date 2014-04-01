import processing.serial.*;
import java.text.*;
import java.util.*;
import java.io.*;

PImage background; // Background image
PFont mono; // Font
String time; // Prog time
String flightTimeStr; // Flight millis
long connectTime;
byte flightTime1; // Flight time from start, unsigned long = 4 bytes
byte flightTime2;
byte flightTime3;
byte flightTime4;
boolean connection = false; // Connected to copter, or no.

int roll, pitch;

int rollSetpoint, pitchSetpoint;

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
    
    text("Keyboard Options:        Press ENTER to send", 25, 150);
    text("X : Disarm", 25, 175);
    text("S : Start", 25, 200);
    text("B : Broadcast", 25, 225);
    text("P : Go UP", 25, 250);
    text("I : Go DOWN", 25, 275);
    text("UP arrow: Pitch up", 25, 300);
    text("DOWN arrow: Pitch down", 25, 325);
    text("LEFT arrow: Roll up", 25, 350);
    text("RIGHT arrow: Roll down", 25, 375);
    text("R : Reset Attitude", 25, 400);
    
    text("Attitude: ", 500, 90);
    
    text("Pitch Set: ", 500, 115);
    text(str(pitchSetpoint), 500, 140);
    text("Pitch Actual: ", 600, 115);
    text(str(pitch), 600, 140);
    
    text("Roll Set: ", 500, 190);
    text(str(rollSetpoint), 500, 215);
    text("Roll Actual: ", 600, 190);
    text(str(roll), 600, 215);
    
    
    // Display prog time and flight time
    displayTimes();

    // Display connection status;
    checkConnection();

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
    if(myPort.available() > 0)
    {
        connection = true;
        connectTime = millis();
        
        if(myPort.available() > 4)
        {
            int firstbyte = myPort.read();
            if(firstbyte == 0xFF)
            {
                int roll0 = myPort.read();
                int roll1 = myPort.read();
                int pitch0 = myPort.read();
                int pitch1 = myPort.read();
                
                roll = roll0 | (roll1 << 8);
                if(roll > 10000)
                {
                    roll = 65535 - roll;
                }
                pitch = pitch0 | (pitch1 << 8);
                if(pitch > 10000)
                {
                    pitch = 65535 - pitch;
                }
            }
        }
    }
}

/*================================================================================
     Scan Keyboard Input
     -----------------------------------------------------------------------------*/
void keyPressed()
{
    if(key == CODED)
    {
        println(key);
    }
    
    switch(key)
    {
        case 'X':
        case 'x':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x00);
            txPacket[2] = byte(0x00);
            txPacket[3] = byte(0x00);
            txPacket[4] = byte(0x00);
            sendtxPacket();
            break;
        case 'S':
        case 's':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x02);
            txPacket[2] = byte(0x00);
            txPacket[3] = byte(0x00);
            txPacket[4] = byte(0x00);
            sendtxPacket();
            break;
        case 'B':
        case 'b':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x03);
            txPacket[2] = byte(0x00);
            txPacket[3] = byte(0x00);
            txPacket[4] = byte(0x00);
            sendtxPacket();
            break;
        case 'P':
        case 'p':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0C);
            txPacket[2] = byte(pGain[2]);
            txPacket[3] = byte(pGain[1]);
            txPacket[4] = byte(pGain[0]);
            sendtxPacket();
            break;
        case 'I':
        case 'i':
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0D);
            txPacket[2] = byte(iGain[2]);
            txPacket[3] = byte(iGain[1]);
            txPacket[4] = byte(iGain[0]);
            sendtxPacket();
            break;
            
        case ENTER:
            sendtxPacket();
            
        default:
            // Hell yeah
            break;
    }
    switch(keyCode)
    {
        
        case UP: // Increase Pitch
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0E);
            txPacket[2] = byte(0);
            txPacket[3] = byte(0);
            txPacket[4] = byte(0);
            sendtxPacket();
            break;
            
        case DOWN: // Decrease Pitch
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x0F);
            txPacket[2] = byte(0);
            txPacket[3] = byte(0);
            txPacket[4] = byte(0);
            sendtxPacket();
            break;
            
        case LEFT: // increase Roll
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x10);
            txPacket[2] = byte(0);
            txPacket[3] = byte(0);
            txPacket[4] = byte(0);
            sendtxPacket();
            break;
            
        case RIGHT: // decrease Roll
            txPacket[0] = byte(0xFF);
            txPacket[1] = byte(0x11);
            txPacket[2] = byte(0);
            txPacket[3] = byte(0);
            txPacket[4] = byte(0);
            sendtxPacket();
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
    switch(txPacket[1])
    {
        case 0x00:
        {
            connection = false;
            connectTime = 0;
            pitchSetpoint = 0;
            rollSetpoint = 0;
            roll = 0;
            pitch = 0;
            break;
        }
        case 0x0E:
        {
            pitchSetpoint = pitchSetpoint + 5;
            break;
        }
        case 0x0F:
        {
            pitchSetpoint = pitchSetpoint - 5;
            break;
        }
        case 0x10:
        {
            rollSetpoint = rollSetpoint + 1;
            break;
        }
        case 0x11:
        {
            rollSetpoint = rollSetpoint - 1;
            break;
        }
        default:
        {
            break;
        }
    }
            
            
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
    flightTimeStr = str(millis() - connectTime);
    if(connection == true)
    {
        text("Flight time: ", 550, 50);
        text(flightTimeStr, 700, 50);
    }
}

/*================================================================================
     Connected? Print it.
     -----------------------------------------------------------------------------*/
void checkConnection()
{
    if(millis() - connectTime > 5000)
    {
        connection = false;
    }
    text("Connection Status: ", 496, 25);
    text(str(connection), 700, 25);
}

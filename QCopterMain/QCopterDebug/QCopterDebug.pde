 /*=========================================================================
Name: QCopterMain.ino
Authors: Brandon Riches, Patrick Fairbanks, Andrew Coulthard
Date: May 2013


This program is a debugging data visualization tool, using Processing 1.5.1
    -----------------------------------------------------------------------*/
    
/*=========================================================================
    Logging and saving data
    -----------------------------------------------------------------------*/  
PrintWriter output;
String mo = str(month());
String da = str(day());
String yr = str(year());
String filename = "Data_" + mo + "_" + da + "_" + yr +".csv";
    
/*=========================================================================
    Sensor 
    -----------------------------------------------------------------------*/ 
import processing.serial.*;
Serial myPort;    // The serial port    
float accelval;
/*=========================================================================
    Rendering 
    -----------------------------------------------------------------------*/ 
int xPos = 1;     // Horizontal position of the graph in the window


void setup()
{
  size(600,600);             // Sets the size of the window
  
  println(Serial.list());    // Prints the available serial ports
  
  myPort = new Serial(this, Serial.list()[0], 115200);
  
  myPort.bufferUntil('\n');  // Buffer until a newline
  
  background(0); // Set initial background
  
  output = createWriter(filename);
}

void draw() {
  output.println(millis() + "," + accelval);
}

void keyPressed() {
  if (key == ESC) {
    stop();
    exit();
  }
}

void stop(){
  // Writes the remaining data to the file
  // and closes it before closing sketch
  output.flush(); 
  output.close();
  println("Data Saved");
  super.stop();
}

void serialEvent (Serial myPort)
{
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  
  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    // convert to an int and map to the screen height:
    float inByte = float(inString); 
    println(inByte);
    accelval = inByte;
    inByte = map(inByte, -10, 10, 0, height);

    // draw the line:
    stroke(255);
    strokeWeight(4);
    line(xPos, height, xPos, height - inByte);
    
    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0); 
    } 
    else {
      // increment the horizontal position:
      xPos++;
    }
    println(xPos);
  }
}
    
  


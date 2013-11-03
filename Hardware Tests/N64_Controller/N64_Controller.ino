
double xmin, ymin, xmax, ymax, x, y;
double logBase = 2;
double ymid, xmid;

double xSamples[2][100];
double ySamples[2][100];
int counter = 0;

void getSamples(int i)
{
        Serial.println("Sampling...");
        xmax = xmid;
        xmin = xmid;
        ymax = ymid;
        ymin = ymid;
        
        for(int a = 0; a < 100; a++)
        {
                x = log((double)analogRead(1))/log(logBase);
                y = log((double)analogRead(0))/log(logBase);
                
                if(x > xmax) 
                {
                        xmax = x;
                }
                if(x < xmin) 
                {
                        xmin = x;
                }
                if(y > ymax) 
                {
                        ymax = y;
                }
                if(y < ymin) 
                {
                        ymin = y;
                }
                
                Serial.println("Sampling");
                
                counter++;
                if(counter >= 100)
                {
                        counter = 0;
                        Serial.println("Results: ");
                        Serial.print("Xmin: ");
                        Serial.print(xmin);
                        Serial.print(" Lower x range: ");
                        Serial.print(xmid - xmin);
                        Serial.print(" Xmid: ");
                        Serial.print(xmid);
                        Serial.print(" Upper x range: ");
                        Serial.print(xmax - xmid);
                        Serial.print(" Xmax: ");
                        Serial.println(xmax);
                        
                        Serial.print("Ymin: ");
                        Serial.print(ymin);
                        Serial.print(" Lower y range: ");
                        Serial.print(ymid - ymin);
                        Serial.print(" Ymid: ");
                        Serial.print(ymid);
                        Serial.print(" Upper y range: ");
                        Serial.print(ymax - ymid);
                        Serial.print("Ymax: ");
                        Serial.println(ymax);
                        
                        xSamples[0][i] = logBase;
                        xSamples[1][i] = 1 / (abs((xmax - xmid) - (xmid - xmin)) + 1);
                        
                        ySamples[0][i] = logBase;
                        ySamples[1][i] = 1 / (abs((ymax - ymid) - (ymid - ymin)) + 1);
        
                }
                delay(10);
        }
}

void setup()
{
        
        Serial.begin(38400);
        Serial.println("Hello World");
        
}

void loop()
{        
        x = log((double)analogRead(1))/log(2);
        y = log((double)analogRead(0))/log(2);
        Serial.print("Initial X: ");
        Serial.print(x);
        Serial.print(" Initial Y: ");
        Serial.println(y);
        
        delay(1000);
        
        xmin = x;
        xmax = x;
        ymin = y;
        ymax = y;
        
        ymid = y;
        xmid = x;
        
        Serial.println("Hello World");
        for(int i = 0; i< 25; i++)
        {
                getSamples(i);
                logBase += 0.5;
                Serial.println("Increasing logBase");
        }
        
        double maxQualityX = 0;
        double maxQualityY = 0;
        int thisX, thisY;
       
        
        for(int a = 0; a< 100; a++)
        {
                if(xSamples[1][a] > maxQualityX) 
                {
                        maxQualityX = xSamples[1][a];
                        thisX = a;
                }
                if(ySamples[1][a] > maxQualityY) 
                {
                        maxQualityY = ySamples[1][a];
                        thisY = a;
                }
        }
        
        Serial.println("Final Results: ");
        Serial.print("Best log scale x: ");
        Serial.print(xSamples[0][thisX]);
        Serial.print(" Best log scale y: ");
        Serial.print(ySamples[0][thisY]);
        
        while(1);
        
}
        


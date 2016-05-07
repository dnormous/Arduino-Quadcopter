//LIBRARIES
  #include <Wire.h> 
  //#include <Average.h>
  #include <FUTABA_SBUS.h>
  #include <Streaming.h>
  #include <PID_v1.h>
  #include <Servo.h>

//*****************************
//---------VARIABLES--------
//*****************************
     //Variables for reading gyroscope
        #define gyro 0x6B                  //Address for L3GD20H gyroscope (0011110b)
        #define L3G_CTRL_REG1 0x20         //L3GD20H register address
        #define L3G_CTRL_REG4 0x23         //L3GD20H register address
        #define L3G_OUT_X_L 0x28           //First axis output register
        #define L3G_Sens_2000 (0.070F)  //Gyroscope sensitivity dps/digit    
        #define DPS_TO_RAD (0.017453293F)  //Constant for converting Degree/Second to Radians

    //Data holders for raw gyroscope readings
        byte gbuff[6];
        float gx, gy, gz;
        double gyroX, gyroY, gyroZ, gyroAngleX, gyroAngleY, gyroAngleZ; 
                
    //Variables for reading accelerometer
        #define acc  0b0011101         //Address for LSM303D (0011110b)
        #define CTRL1  0x20            //LSM303D register address
        #define CTRL2  0x21            //LSM303D register address
        #define OUT_X_L_A  0x28        //First axis output register
        #define Acc_Sens_2G 0.000061   //Accelerometer sensitivity g/LSB
        #define GRAV 9.80665F          //Gravity in m/s^2  

    //Data holders for raw accelerometer readings
        byte abuff[6];
        float ax, ay, az, anorm;
        float radianPitch, radianRoll, accPitch, accRoll;
     
    //Variables for reading magnetometer
        #define mag  0b0011101         //Address for LSM303D (0011110b)
        #define CTRL5  0x24            //LSM303D register address
        #define CTRL6  0x25            //LSM303D register address
        #define CTRL7  0x26            //LSM303D register address
        #define OUT_X_L_M  0x08        //First axis output register
        #define Mag_Sens_2G 0.080      //Magnetometer sensitivity mgauss/LSB

    //Data holders for raw magnetometer readings
        byte mbuff[6];
        float mx, my, mz;
        float MagMinX = -450.88; float MagMinY = -482.56; float MagMinZ = -408.16;
        float MagMaxX = 580.32;  float MagMaxY = 546.88;  float MagMaxZ = 609.92;
        float var, magAdjust, magX, magY, magZ, magxcomp, magycomp, Heading;
        float lastHeading;
        float mxc, myc, mzc;  
  
    //Data holders for complimentary filter & sketch timing
        double Roll, Pitch, Yaw, YawMap;
        float timeStep, startTime;
        float test, start, finish, Hz;
        float count = 0;
    
    //Data holders for transmitter reading (SBUS)
        FUTABA_SBUS sBus;
        #define ALL_CHANNELS
        float THR, RIN, PIN, YIN, frameLoss, GxIN, GyIN;
        float S1x, S2x, LSx, RSx, SAx, SBx, SCx, SDx, SEx, SFx, SGx, SHx;
        float stickMin = 172, stickMax = 1811;
        float S1min, S1max, S2min, S2max, LSmin, LSmax, RSmin, RSmax; 
    
    //Data holders for PID rate control    
        //X-axis rate loop  
          float xPIDinput, xPIDsetpoint, xPIDsampleTime, xPIDsampleTimeInSec, xPIDlastTime;
          float xPIDkp, xPIDKp, xPIDki, xPIDKi, xPIDkd, xPIDKd, xPIDoutMax, xPIDoutMin;
          float xPIDdInput, xPIDerror, xPIDiTerm, xPIDlastInput, xPIDoutput; 
          unsigned long xPIDnow;
          int xPIDtimeChange;
        //Y-axis rate loop 
          float yPIDinput, yPIDsetpoint, yPIDsampleTime, yPIDsampleTimeInSec, yPIDlastTime;
          float yPIDkp, yPIDKp, yPIDki, yPIDKi, yPIDkd, yPIDKd, yPIDoutMax, yPIDoutMin;
          float yPIDdInput, yPIDerror, yPIDiTerm, yPIDlastInput, yPIDoutput; 
          unsigned long yPIDnow;
          int yPIDtimeChange;
        //Z-axis rate loop 
        
    
    //Data holders for stabilization PID loops 

    
     //Set variables for Servo control of motors---
       Servo escFL; Servo escFR;  
       Servo escBL; Servo escBR;  
       int FLmotorInput; int FRmotorInput;
       int BLmotorInput; int BRmotorInput;
    

//*****************************
//---------FUNCTIONS--------
//*****************************

//Writes val to address register on device 
    void writeTo(int device, byte reg, byte val) {
     Wire.beginTransmission(device);   //start transmission to device (accelerometer, magnetometer, gyro, etc.) 
     Wire.write(reg);               //send register address to send byte value to
     Wire.write(val);                   //send byte value to write
     Wire.endTransmission();           //end transmission
    }


//Reads num bytes starting from the given register and adds the bytes to the buff array
    void readFrom(int device, byte reg, int num, byte buff[]) {
      Wire.beginTransmission(device); //start transmission to ACC 
      Wire.write(reg | (1<<(num+1)));        //sends address to read from
      Wire.endTransmission(); //end transmission
      
      Wire.requestFrom(device, num);    // request 6 bytes from ACC   
      int i = 0;
      while(Wire.available())    //ACC may send less than requested (abnormal)
      { 
        buff[i] = Wire.read(); // receive a byte
        i++;
      }
      Wire.endTransmission(); //end transmission
    }

//Median filter of three values
    float gxra, gyra, gzra, gxrb, gyrb, gzrb, gxrc, gyrc, gzrc;
    float gstep, gyroXf, gyroYf, gyroZf;
    float mdnfilter(float a, float b, float c) {     
      float middle;
      if ((a <= b) && (a <= c))
       {
         middle = (b <= c) ? b : c;
        }
      else if ((b <= a) && (b <= c))
       {
         middle = (a <= c) ? a : c;
       }
      else
       {
         middle = (a <= b) ? a : b;
       }
      return middle;
     }


//*****************************
//---------SETUP--------
//*****************************

void setup() {
  
  Wire.begin();        //Start the wire I2C connection
  Serial.begin(115200);  // start serial for output

  //Initiate gyroscope
    writeTo(gyro, L3G_CTRL_REG1, 0xCF); // *Normal power mode, all axes enabled. 0xCF = 0b11001111
    writeTo(gyro, L3G_CTRL_REG4, 0x20);  //Full scale selection set to 2000 dps
  
  //Initiate accelerometer
    writeTo(acc, CTRL2, 0x00); //*Enable the accelerometer (AFS = 0 --> +/- 2 g full scale)
    writeTo(acc, CTRL1, 0x87); //*AODR = 1000 (400 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
 
  //Initiate magnetometer  
    writeTo(mag, CTRL5, 0x74);  //*(0x74 = 0b01110100) M_RES = 11 (high resolution mode); M_ODR = 101 (100 Hz ODR)
    writeTo(mag, CTRL6, 0x00);  //*(0x20 = 0b00000000) MFS = 00 (+/- 2 gauss full scale) 
    writeTo(mag, CTRL7, 0x00);  //*(0x00 = 0b00000000) MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
 
  //Initiate SBUS communication with RC transmitter
    sBus.begin();
    
  //Startup mortor control system
    escFL.attach(7);   //Front left motor pin connection
    escFR.attach(6);   //Front right motor pin connection
    escBL.attach(4);   //Back left motor pin connection
    escBR.attach(5);   //Back right motor pin connection
         
  //Serial.println("Setup Complete");  //For verification of setup code completion
}


//*****************************
//---------LOOP--------
//*****************************

void loop() {
  
   /********************************
     STARTUP MOTORS 
   ********************************/    
          if (count < 1) {                    //Count used from "COMPLIMENTARY FILTER" section
            escFL.writeMicroseconds(1000);    //Turn on front left motor
            escFR.writeMicroseconds(1000);    //Turn on front right motor
            escBL.writeMicroseconds(1000);    //Turn on back left motor
            escBR.writeMicroseconds(1000);    //Turn on back right motor
           }
     

   /********************************
     READ GYROSCOPE
   ********************************/	
        readFrom(gyro, L3G_OUT_X_L, 6, gbuff);         //I2C function to read device register
        
        //Convert raw bytes to integer 
          gx = (gbuff[1] << 8 | gbuff[0]); //Shift values to create properly formed integer
    	  gy = (gbuff[3] << 8 | gbuff[2]); // and Multiply by gyrosensivity to get readings in mDPS
    	  gz = (gbuff[5] << 8 | gbuff[4]);

          //  if (gx > -10 && gx < 18) gx = 0;

      
      //Axis rotation in DPS (degree per second)
        gyroX = gx*L3G_Sens_2000; 
        gyroY = gy*L3G_Sens_2000; 
        gyroZ = gz*L3G_Sens_2000; 
      
//750 Hz

      //Drift corrected gyro axis angle in degrees for raw readings (useful for calibration)
        //gyroAngleX += (gyroX*(micros() - startTime)/1000000); 
        //gyroAngleY = gyroAngleY + (gyroY*(micros() - startTime)/1000000);  
        //gyroAngleZ = gyroAngleZ + (gyroZ*(micros() - startTime)/1000000);
          
       
   /********************************
     READ ACCELEROMETER
   ********************************/
        readFrom(acc, OUT_X_L_A, 6, abuff);    //I2C function to read device register
      
        //Convert raw bytes to integer 
          ax = (abuff[1]<<8|abuff[0])*Acc_Sens_2G; //Shift values to create properly formed integer
          ay = (abuff[3]<<8|abuff[2])*Acc_Sens_2G; // and Multiply by gyrosensivity to get readings in G
          az = (abuff[5]<<8|abuff[4])*Acc_Sens_2G; 
//565 Hz
          anorm = sqrt(ax*ax+ay*ay+az*az);
          ax /= anorm;                      //Normalize acceleration measurements 
          ay /= anorm;                         // (limits range from 0 to 1)
          az /= anorm;
//517 Hz        
        //Calculate pitch and roll in radians
          radianPitch = asin(-ax);
          var = ay/cos(radianPitch);
            if (var > 1) var = 1;
            if (var < -1) var = -1;    
          radianRoll = asin(var);  

        //Calculate pitch and roll in degrees
          accPitch = radianPitch*57.2957795;
          accRoll = radianRoll*57.2957795;  
          
//405 Hz    
          
   /********************************
     READ MAGNETOMETER
   ********************************/
        readFrom(mag, OUT_X_L_M, 6, mbuff);    //I2C function to read device register

        //Convert raw bytes to integer
          mx = (mbuff[1]<<8|mbuff[0])*Mag_Sens_2G; //Shift values to create properly formed integer
          my = (mbuff[3]<<8|mbuff[2])*Mag_Sens_2G; // and Multiply by sensivity to get readings in mGauss
          mz = (mbuff[5]<<8|mbuff[4])*Mag_Sens_2G;    
//345 Hz   
             //Magnetometer Calibration: Find max/min
             //  if (mx > mxc) mxc = mx;
             //  if (my > myc) myc = my;
             //  if (mz > mzc) mzc = mz;  
   
        //Use calibration values to shift and scale magnetometer measurements
          magX = (mx-MagMinX)/(MagMaxX-MagMinX)*2-1;  
          magY = (my-MagMinY)/(MagMaxY-MagMinY)*2-1;  
          magZ = (mz-MagMinZ)/(MagMaxZ-MagMinZ)*2-1;  
    
        //Tilt compensate magnetic sensor measurements
          float magxcomp = magX*cos(radianPitch)+magZ*sin(radianPitch);
          float magycomp = magX*sin(radianRoll)*sin(radianPitch)+magY*cos(radianRoll)-magZ*sin(radianRoll)*cos(radianPitch);
        
        //Arctangent of y/x converted to degrees + mag adjustment
          magAdjust = 0;
          Heading = (180*atan2(magycomp,magxcomp)/PI) + magAdjust;
          
          Heading = (Heading+lastHeading)/2;         
          lastHeading = Heading;    
          
        //Loop heading to fit between 0-360 degrees
          if (Heading < 0) Heading += 360;
//260 Hz

   /* GYRO MEDIAN FILTER  --> Clears out the random spikes */
      gstep +=1;
      
      if (gstep == 1) 
        {
          gxra = gyroX;
          gyra = gyroY;
          gzra = gyroZ;
        }
      if (gstep == 2) 
        {
          gxrb = gyroX;
          gyrb = gyroY;
          gzrb = gyroZ;
        }
      if (gstep == 3) 
        {
          gxrc = gyroX;
          gyrc = gyroY;
          gzrc = gyroZ;
          gstep = 0;
        }
      gyroXf = mdnfilter(gxra, gxrb, gxrc);
      gyroYf = mdnfilter(gyra, gyrb, gyrc);
      gyroZf = mdnfilter(gzra, gzrb, gzrc);
 
   /***********************************************
     COMPLIMENTARY FILTER FOR ROLL, PITCH, YAW
   ***********************************************/   
        //Start filter with current accelerometer & magnetometer readings
          if (count < 1) Roll = accRoll; 
          if (count < 1) Pitch = accPitch; 

          count += 1; 
        
        //Loop time for complimentary filter
          timeStep = micros() - startTime; 
          startTime = micros();
        
        //Complimentary filter calculation for Rall & Pitch
          Roll = ((0.98)*(Roll + (gyroXf*((micros() - startTime)/1000000)))) + (0.02*accRoll);
          Pitch = ((0.98)*(Pitch + (gyroYf*((micros() - startTime)/1000000)))) + (0.02*accPitch);
        
//250 Hz      
     
   /********************************
     READ TRANSMITTER SIGNAL (SBUS)
   ********************************/
         sBus.FeedLine();             //SBUS library call to read incoming signal
           if (sBus.toChannels == 1){   
               sBus.UpdateChannels();
               sBus.toChannels = 0; }
   
           THR = sBus.channels[0];    //Throttle input
           RIN = sBus.channels[1];    //Roll input
           PIN = sBus.channels[2];    //Pitch input
           YIN = sBus.channels[3];    //Yaw input
           S1x = sBus.channels[4];    //S1 dial input
           S2x = sBus.channels[5];    //S2 dial input
           LSx = sBus.channels[6];    //LS dial input
           RSx = sBus.channels[7];    //RS dial input
           SAx = sBus.channels[8];    //SA switch input
           SBx = sBus.channels[9];    //SB switch input
           SCx = sBus.channels[10];   //SC switch input
           SDx = sBus.channels[11];   //SD switch input 
        // SEx = sBus.channels[12];   //SE switch input
        // SFx = sBus.channels[13];   //SF switch input
        // SGx = sBus.channels[14];   //SG switch input
        // SHx = sBus.channels[15];   //SH switch input
          
           frameLoss = sBus.Failsafe();  //Measures transmitter signal loss 

         //Map transmitter signals to useable value range
           THR = map(THR, stickMin, stickMax,0,750);  //THOUGHT--> Max throttle must be less than real max
             GxIN = map(RIN, stickMin, stickMax,-500,500);  //Control rotation rate of x-axis (roll)
           RIN = map(RIN, stickMin, stickMax,-4000,4000);  //           in order to maintain stabilization
             GyIN = map(PIN, stickMin, stickMax,-1000,1000);  //Control rotation rate of y-axis (roll)      
           PIN = map(PIN, stickMin, stickMax,-4000,4000);
           YIN = map(YIN, stickMin, stickMax,-1000,1000);       //Control rotation rate of z-axis (roll)
           S1x = map(S1x, stickMin, stickMax,0,10000);    //Mapped to rate Kp tuning
           S2x = map(S2x, stickMin, stickMax,0,10000);    //Mapped to rate Ki tuning
           LSx = map(LSx, stickMin, stickMax,0,5000);    //Mapped to stabilization Kp tuning       
           RSx = map(RSx, stickMin, stickMax,0,5000);    //Mapped to stabilization Ki tuning       

         //Create center buffer zone for transmitter sticks
           if(RIN < 20 && RIN > -20) RIN = 0;
           if(PIN < 20 && PIN > -20) PIN = 0;
           if(YIN < 20 && YIN > -20) YIN = 0;

//230 HZ

   /**************************************
     PID LOOP for stabilization angles (STABILIZATION MODE)
   ***************************************/  




   /********************************
     PID LOOP for rotation rates (ACRO MODE) 
   ********************************/        
    if(THR > 150) {               //Only activate PID when throttle is above a certain point
      //PID for X-axis rate    
         //PID settings 
           xPIDKp = S1x/1000;   
           xPIDKi = 0; //S2x/1000;
           xPIDKd = 0;
           
           xPIDoutMax = THR;          //Using THR as a check to keep from huge values
           xPIDoutMin = -THR+100;      //Adjust these later
           
           xPIDsampleTime = 10;    //  <--STARTER VALUE (0.01 sec = loop updates 100x/sec)
           
           xPIDsampleTimeInSec = xPIDsampleTime/1000;
           xPIDkp = xPIDKp;
           xPIDki = xPIDKi * xPIDsampleTimeInSec;
           xPIDkd = xPIDKd / xPIDsampleTimeInSec;
         
         //PID input, settings, calculation & output             
           xPIDinput = gyroXf;           
           xPIDsetpoint = GxIN/10; 
           xPIDnow = millis();
           xPIDtimeChange = xPIDnow - xPIDlastTime;
           if(xPIDtimeChange >= xPIDsampleTime)
             {
                xPIDerror = xPIDsetpoint - xPIDinput;
                xPIDiTerm += (xPIDki * xPIDerror);
                if(xPIDiTerm > xPIDoutMax) xPIDiTerm = xPIDoutMax;
                else if(xPIDiTerm < xPIDoutMin) xPIDiTerm = xPIDoutMin;
                double xPIDdInput = (xPIDinput - xPIDlastInput);
           
                /*Compute PID Output*/
                xPIDoutput = xPIDkp * xPIDerror + xPIDiTerm- xPIDkd * xPIDdInput;
                if(xPIDoutput > xPIDoutMax) xPIDoutput = xPIDoutMax;
                else if(xPIDoutput < xPIDoutMin) xPIDoutput = xPIDoutMin;
           
                /*Remember some variables for next time*/
                xPIDlastInput = xPIDinput;
                xPIDlastTime = xPIDnow;
                //if(xPIDerror < __) xPIDoutput = __;
             }

     //PID for Y-axis rate    
         //PID settings 
           yPIDKp = S1x/1000;   
           yPIDKi = 0; //S2x/1000;
           yPIDKd = 0;
           
           yPIDoutMax = THR;          //Using THR as a check to keep from huge values
           yPIDoutMin = -THR+100;      //Adjust these later
           
           yPIDsampleTime = 10;    //  <--STARTER VALUE (0.01 sec = loop updates 100x/sec)
           
           yPIDsampleTimeInSec = yPIDsampleTime/1000;
           yPIDkp = yPIDKp;
           yPIDki = yPIDKi * yPIDsampleTimeInSec;
           yPIDkd = yPIDKd / yPIDsampleTimeInSec;
         
         //PID input, settings, calculation & output             
           yPIDinput = gyroYf;           
           yPIDsetpoint = GyIN/10; 
           yPIDnow = millis();
           yPIDtimeChange = yPIDnow - yPIDlastTime;
           if(yPIDtimeChange >= yPIDsampleTime)
             {
                yPIDerror = yPIDsetpoint - yPIDinput;
                yPIDiTerm += (yPIDki * yPIDerror);
                if(yPIDiTerm > yPIDoutMax) yPIDiTerm = yPIDoutMax;
                else if(yPIDiTerm < yPIDoutMin) yPIDiTerm = yPIDoutMin;
                double yPIDdInput = (yPIDinput - yPIDlastInput);
           
                /*Compute PID Output*/
                yPIDoutput = yPIDkp * yPIDerror + yPIDiTerm- yPIDkd * yPIDdInput;
                if(yPIDoutput > yPIDoutMax) yPIDoutput = yPIDoutMax;
                else if(yPIDoutput < yPIDoutMin) yPIDoutput = yPIDoutMin;
           
                /*Remember some variables for next time*/
                yPIDlastInput = yPIDinput;
                yPIDlastTime = yPIDnow;
             }
       
       //PID for Z-axis rate after that
       
     }  //Close IF statement for THR check on PIDs
     
   /********************************
     MOTOR SPEED CONTROL
   ********************************/
        //#### ADD IF SIGNAL LOSS, CUT POWER by X/loop ####
        
        //Calculate individuals motor speeds
          //Roll of the quad is coded as pitch and vice versa
            //X=Pitch , Y=Roll 
          
         //X-formation inputs
          FRmotorInput = 1000 + THR + xPIDoutput + yPIDoutput; //- GzPIDout;  
          FLmotorInput = 1000 + THR + xPIDoutput - yPIDoutput; // + GzPIDout;
          BRmotorInput = 1000 + THR - xPIDoutput + yPIDoutput; // + GzPIDout;
          BLmotorInput = 1000 + THR - xPIDoutput - yPIDoutput; // - GzPIDout; 


           
        //Safety switches!
          if(SAx < 1750 || SDx < 1750) {
            FLmotorInput = BLmotorInput = FRmotorInput = BRmotorInput = 900;  //Might have to stay >= 1000  
            } 

          if(frameLoss > 2) {
            FLmotorInput = BLmotorInput = FRmotorInput = BRmotorInput = 900;  //Might have to stay >= 1000  
            } 

        //RADIO FRAME LOSS SAFETY POWER DROP
            // == IF FRAMELOSS > 1, THEN THR - 1/count

          
          //ADD LED SIGNAL LIGHT CODES HERE


        //Send PWM signals to individual motors
          escFL.writeMicroseconds(FLmotorInput); 
          escBL.writeMicroseconds(BLmotorInput); 
          escFR.writeMicroseconds(FRmotorInput);  
          escBR.writeMicroseconds(BRmotorInput);  

//200 Hz

   /********************************
     MEASURE SKETCH SPEED
   ********************************/
          //finish = micros() - start;
          //start = micros();
          //Hz = 1000000/finish;

   /********************************
     Print to Serial or LCD
   ********************************/
       Serial.print(gyroXf);
       Serial.print(",");
       Serial.print(Roll);
       Serial.print(",");
       Serial.print(xPIDoutput);
       Serial.print(",");
       Serial.println(xPIDerror);


//delayMicroseconds(100);   //Used for testing purposes

 //END OF LOOP
}




   



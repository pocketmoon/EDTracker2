// 28/05/2014 Scale head movement to mimic in-game scaling
//            Add toggle for this behaviour
// 31/05/2014 Show actual raw z (not biased here for g)
// 10/06/2014 Enlarge window. Add smarfter com port handling
// 11/06/2014 Enable updating of accel biax axis individually
import javax.swing.JFrame;

PFrame f;
secondApplet s;


import toxi.geom.*;
import toxi.geom.mesh.*;
import toxi.geom.*; 
import toxi.processing.*;
import processing.serial.*;

/*q
 Commands
 M send toggle monitor
 C Full Calibrate
 I Request Info
 D Debug Info
 */


TriangleMesh mesh;
ToxiclibsSupport gfx;

float rad2deg = 57.29578;
float rad2FSR = 10430.06;

float maxGX, maxGY, maxGZ;
float maxAX, maxAY, maxAZ;

float scaleAdjust=1.0;
String   scaleMode="Unknown";

String info = "Unknown Device";

String []messages = {
  "", "", "", "", "", "", "", "", "", ""
};

float temperature;

//float headScale = 1.0;
Serial  arduinoPort; // Usually the last port 
int     portNumber = 2;
String  portName ;
String  buffer;      //String for testing serial communication

float DMPRoll, DMPPitch, DMPYaw;

boolean expScaleMode = false;// false for linear
float scaleFactor=1.0;

int  rawGyroX, rawGyroY, rawGyroZ;
int  rawAccelX, rawAccelY, rawAccelZ;

float driftScale = 10.0;
float yawDrift =0.0;
float pitchDrift =0.0;
boolean monitoring = false;

float  yawDriftComp=0.0;

boolean driftComp = false;

int adjustX    = 1;
int adjustY    = 1;
int adjustZ    = 1;


float[] yawHist = new float [500];
float[] pitchHist = new float [500];
float[] yawDriftHist = new float [500];
float[] tempHist = new float [500];

int hs = 500;



long lastPress=0;
PFont mono;

iLowPass lpX, lpY, lpZ;

iLowPass aX, aY, aZ;
iLowPass gX, gY, gZ;
boolean found = false;



void setup() {
  size(1000, 800, P3D);
  
  
  PFrame f = new PFrame();
  
  // The font "AndaleMono-48.vlw"" must be located in the 
  // current sketch's "data" directory to load successfully

  //mono =  createFont("Courier New", 64, false);
  //mono = loadFont("Eureka-48.vlw");
  mono = loadFont("CourierNewPSMT-24.vlw");

  background(0);
  textFont(mono);

  maxGX= maxGY= maxGZ =0.0;
  maxAX= maxAY= maxAZ =0.0;

  frameRate(60);
  mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("head3.stl"), STLReader.TRIANGLEMESH);
  //mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("mesh-flipped.stl"),STLReader.TRIANGLEMESH).flipYAxis();

  mesh.computeFaceNormals();
  mesh.computeVertexNormals();
  Vec3D forward = new Vec3D (0.0, 1.0, 0.0); 
  mesh.scale(2.0);
  mesh.pointTowards(forward);
  mesh.rotateZ(3.1415926);
  gfx=new ToxiclibsSupport(this);

  f.setText("Scan for ED Tracker...");

  int lastPort = Serial.list().length -1;
  
  while (lastPort<0)
  {
    f.setText("No com ports in use. Rescanning...");
    delay(1000);
    lastPort = Serial.list().length -1;
  }
    
 f.setText("Locating ED TRacker...");

  println(Serial.list());

  while (!found)
  {
    portName = Serial.list()[lastPort];
    f.setText("Connecting to -> " + portName);
    delay(200);

    try {
      arduinoPort = new Serial(this, portName, 115200);
      arduinoPort.clear();
      arduinoPort.bufferUntil(10);  
      arduinoPort.write('H');

      int l = 5;
      while (!found && l >0)
      {
        delay(200);
        f.setText("Waiting for response from device on " + portName);
        l--;
      }
      
      if (!found)
      {
        f.setText("No response from device on " + portName);
        arduinoPort.clear();      
        arduinoPort.stop();
        delay(200);
      }
      
    }
    catch (Exception e) {
      f.setText("Exception connecting to " + portName);
      println(e);
    }

    lastPort--;
    if (lastPort <0)
      lastPort = Serial.list().length -1;
  }

  arduinoPort.clear();
  arduinoPort.write('I');
  delay(1000);
  arduinoPort.write('V');

  ellipseMode(CENTER);

  lpX = new iLowPass(160);  //The argument is the FIFO queue length
  lpY = new iLowPass(160);  
  lpZ = new iLowPass(160);  

  aX = new iLowPass(160);
  aY = new iLowPass(160);
  aZ = new iLowPass(160);

  gX = new iLowPass(160);
  gY = new iLowPass(160);
  gZ = new iLowPass(160);
  
  f.hide();
}


void serialEvent(Serial p) {

  String dataIn = (arduinoPort.readString());
  char c = dataIn.charAt(0);

  //println(dataIn);
  monitoring = true;// if we're getting data

  try 
  {
    if (c == 'S')
    {
      monitoring = false;
    } else if (c =='H')
    {
      found=true;
    } else if (c =='a')
    {
      adjustX = adjustY=adjustZ= 1;
    } else if (c=='x')
    {
      adjustX = 1;
      adjustY=adjustZ= 0;
    } else if (c=='y')
    {
      adjustY = 1;
      adjustX=adjustZ= 0;
    } else if (c=='z')
    {
      adjustX = adjustY=0;
      adjustZ= 1;
    } else if (c == 'V')
    {
      monitoring = true;
    } else
    {
      String data[] = split(dataIn, '\t');

      if (data[0].equals ("M"))
      {
        for (int i = 0;i<9;i++)
          messages[i] = messages [i+1];
        messages[9] = data[1];
      } 
      else  if (data[0].equals("s"))
      {
        expScaleMode =  (int(data[1]) == 1);
        scaleFactor = float(data[2]);        
      } 
      else  if (data[0].equals("I"))
      {
        // println("Info");
        info =  new String(data[1].substring(0, data[1].length()-2));
      } 
      else  if (data[0].equals("D"))
      {
        //println("Info");
        yawDrift = float(data[1]);
        yawDriftComp = float(data[2]);
        for (int i=0; i<hs-1;i++)
        {
          yawDriftHist[i] = yawDriftHist[i+1];
        }
        yawDriftHist[hs-1]=yawDrift;
      } else  if (data[0].equals("T"))
      {
        //println(data[1]);
        temperature  = float(data[1])/32767.0;
        for (int i=0; i<hs-1;i++)
        {
          tempHist[i] = tempHist[i+1];
        }
        tempHist[hs-1]=temperature;
      } else  if (data[0].equals("R"))
      {
        println("Recentered");
        yawDrift=0.0;
        pitchDrift=0.0;
      } else if (data[0].equals("S"))
      {
        println("Silent");
        monitoring = false;
      } else if (data[0].equals("s"))
      {
        //println("Scaling mod");
        if (data[1].equals("R"))
          scaleMode = "Linear";
        else
          scaleMode = "Exponential";

        scaleAdjust =  float(data[2]);
      } else if (data[0].equals("V"))
      {
        println("Verbose");
        monitoring = true;
      } else {
        //println("YPR");
        DMPYaw = float(data[0])/10430.06;
        DMPPitch = float(data[1])/10430.06;
        DMPRoll  = -float(data[2])/10430.06;

        //        rawAccelX = int(data[3]);
        //        rawAccelY = int(data[4]);
        //        rawAccelZ = int(data[5]);
        lpX.input (int(data[3]));
        lpY.input (int(data[4]));
        lpZ.input (int(data[5]));

        rawAccelX = lpX.output;
        rawAccelY = lpY.output;
        rawAccelZ = lpZ.output;

        gX.input (int(data[6]));
        gY.input (int(data[7]));
        gZ.input (int(data[8]));

        rawGyroX = gX.output;
        rawGyroY = gY.output;
        rawGyroZ = gZ.output;

        //        if (abs(rawAccelX) > maxAX)    maxAX = abs(rawAccelX) ;
        //        if (abs(rawAccelY) > maxAY)    maxAY = abs(rawAccelY) ;
        //        if (abs(rawAccelZ) > maxAZ)    maxAZ = abs(rawAccelZ) ;
        //        
        //        if (abs(rawGyroX) > maxGX)    maxGX = abs(rawGyroX) ;
        //        if (abs(rawGyroY) > maxGY)    maxGY = abs(rawGyroY) ;
        //        if (abs(rawGyroZ) > maxGZ)    maxGZ = abs(rawGyroZ) ;

        //println("A "+maxAX+" "+maxAY+" "+maxAZ+"   G "+maxGX+" "+maxGY+" "+maxGZ);
      }
    }
  } 
  catch (Exception e) {
    //    println("Caught Exception");
    //    println(dataIn);
    //  println(e);
  }
}

int debounce=0;

void draw() {

  long now = millis();

  if (debounce>0)
    debounce--;

  if (keyPressed && debounce <=0) 
  {
    debounce=20;

    lastPress = now + 800;

    //    if (key == 'r')  // reset Arduino!
    //    {
    //      arduinoPort.clear();
    //      arduinoPort.stop();
    //      delay(100);
    //      arduinoPort = new Serial(this, portName, 1200);
    //      arduinoPort.stop();
    //      
    //      arduinoPort = new Serial(this, portName,115200);
    //      arduinoPort.clear();
    //      delay(100);
    //
    //    }

    if (key == 'i' || key == 'I')  // into
    {
      //yawOffset = -DMPYaw;
      //pitchOffset = -DMPPitch;
      arduinoPort.write('I');
    }


    if (key == 'T' || key == 't')  // into
    {
      //yawOffset = -DMPYaw;
      //pitchOffset = -DMPPitch;
      arduinoPort.write('t');
    }

    if (key == '1') 
    {
      if (monitoring)
        arduinoPort.write('S');
      else
      {
        arduinoPort.write('V');
        arduinoPort.write('I');
      }
    }

    if (monitoring)
    {

      if (key == '0') 
      {
        arduinoPort.write('0');
      }

      if (key == 'a') 
      {
        arduinoPort.write('a');
      }

      if (key == 'x') 
      {
        arduinoPort.write('x');
      }

      if (key == 'y') 
      {
        arduinoPort.write('y');
      } 

      if (key == 'z') 
      {
        arduinoPort.write('z');
      } 


      if (key == '2') 
      {
        arduinoPort.write('I');
      }

      if (key == '3') 
      {
        arduinoPort.write('R');
      }

      if (key == '4') 
      {
        // ask ET to toggle response mode
        arduinoPort.write('t');
      }

      if (key=='5')
      {
        arduinoPort.write('P');
      }

     if (key=='6')
      {
       arduinoPort.write('t');//toggle linear/expo
     }

      if (key=='7')
      {
        driftScale *= 2.0;
        if (driftScale > 2000.0)
          driftScale = 1.0;
      }

      if (key=='8')
      {
        arduinoPort.write('D');
      }

      if (key=='9')
      {
        arduinoPort.write('B');
      }

      if (key=='f')
      {
        arduinoPort.write('F');
      }
    }

    //      
    if (key=='q' || key =='Q')
    {
      exit();
    }
  }

  background(30);
  fill(0, 0, 0);
  rect(0, height-100, width, height);


  fill(249, 250, 150);
  textSize(26); 
  text("ED Tracker Configuration and Calibration Utility", 10, 30);


  if (monitoring)
  {
    text(info + " - Monitoring", 10, 60);
  } else
  {
    fill(250, 50, 50);
    text(info +  " - Not Monitoring", 10, 60);
  }

  fill(249, 250, 150);
  textSize(18); 
  text("1 Toggle Monitoring", 10, 100);
  text("2 Get Info", 10, 120);


  if (!info.equals("Unknown Device"))
  {
    if (info.indexOf("Calib")<0)
    {
      text("3 Reset View/Drift Tracking", 10, 140);
      text("4 Toggle Response Mode", 10, 160);
      text("5 Rotate Mounting Axis", 10, 180);
      text("6 Toggle Linear/Exponential Response", 10, 200);
      text("7 Adjust Drift Graph Scale", 10, 220);
      text("8 Save Drift Compensation", 10, 240);
    } 
    else
    {
      text("9 Recalc Bias Values", 10, 280);
      text("x Set X Accel Bias Only", 10, 300);
      text("y Set Y Accel Bias Only", 10, 320);
      text("z Set Z Accel Bias Only", 10, 340);
      text("a Set All Accel Biases", 10, 360);

    }
  }
  text("Q Quit", 10, 540);

  fill(255, 255, 150);


  if (info.indexOf("Calib")<0)
  {
    text("DMP Yaw", (int)width-240, 120);
    text (nfp(DMPYaw*rad2deg, 0, 2), (int)width-100, 120);
    text("DMP Pitch", (int)width-240, 100); 
    text (nfp( DMPPitch*rad2deg, 0, 2), (int)width-100, 100);
    text("DMP Roll", (int)width-240, 140);
    text (nfp(DMPRoll*rad2deg, 0, 2), (int)width-100, 140);
  } else
  {
    if (adjustX == 1 && adjustY ==1 && adjustZ ==1)
      fill(255, 255, 250);   
   else
      fill(100, 100, 100);   

    text("Raw X Gyro", (int)width-240, 40);
    text (rawGyroX, (int)width-100, 40); 

    text("Raw Y Gyro", (int)width-240, 60);
    text (rawGyroY, (int)width-100, 60);

    text("Raw Z Gyro", (int)width-240, 80);
    text (rawGyroZ, (int)width-100, 80);


    fill(100+155*adjustX, 100+155*adjustX, 100+155*adjustX);      
    text("Raw X Accel", (int)width-240, 100);
    text (rawAccelX, (int)width-100, 100); 

    fill(100+155*adjustY, 100+155*adjustY, 100+155*adjustY);      
    text("Raw Y Accel", (int)width-240, 120);
    text (rawAccelY, (int)width-100, 120);

    fill(100+155*adjustZ, 100+155*adjustZ, 100+155*adjustZ);      
    text("Raw Z Accel", (int)width-240, 140);
    text (rawAccelZ, (int)width-100, 140);
  }


  fill(255, 255, 160);



  //text("Yaw Offset", (int)width-240, 440);
  //text (nfp(yawOffset*rad2deg, 0, 2), (int)width-100, 440);
  //text("Pitch Offset", (int)width-240, 460);
  //text (nfp(pitchOffset*rad2deg, 0, 2), (int)width-100, 460);


  text("Yaw Drift", (int)width-240, 400);
  text (nfp(yawDrift, 1, 2), (int)width-100, 400);

  text("Drift Comp", (int)width-240, 420); 
  text (nfp(yawDriftComp, 0, 2), width -100, 420);

  text("Temperature", (int)width-240, 440); 
  text (nfp(temperature, 0, 2), width -100, 440);
  
  
  if (info.indexOf("Calib")<0)
  {
   if (expScaleMode) 
     text("Response Mode : Exponential", (int)width-300, 540);
    else
     text("Response Mode : Linear", (int)width-300, 540);
  }
     
//  text("Response Scale Factor", (int)width-240, 440); 
//  text (nfp(scaleFactor, 0, 2), width -100, 440);

  text("x" + driftScale, 10, height-80);

  //draw the message
  stroke(255, 255, 255);
  fill(255, 255, 255);

  textSize(14); 
  for (int i=0;i<10; i++)
    text (messages[i], 10, height-240+i*14);

  // text(gyrStr, (int) (width/6.0) - 40, 50);

  stroke(255, 255, 255);
  line(0, height-50, width-1, height-50);

  stroke(10, 255, 10);
  for (int i=0; i<hs-1;i++)
  {
    line(i*2, height-50-yawHist[i], (i*2)+1, height-50-yawHist[i+1]); 
    yawHist[i] = yawHist[i+1];
  }
  yawHist[hs-1]=DMPYaw *rad2deg*0.55;


  stroke(250, 10, 10);
  for (int i=0; i<hs-1;i++)
  {
    line(i*2, height-50-pitchHist[i], (i*2)+1, height-50-pitchHist[i+1]); 
    pitchHist[i] = pitchHist[i+1];
  }
  pitchHist[hs-1]=(DMPPitch)*rad2deg*0.55;    

  stroke(255, 255, 0);
  for (int i=0; i<hs-1;i++)
  {
    line(i*2, height-50-constrain(yawDriftHist[i]*driftScale, -45, 45), (i*2)+1, 
    height-50-constrain(yawDriftHist[i+1]*driftScale, -45, 45));
  }



  stroke(255, 0, 0);
  for (int i=0; i<hs-1;i++)
  {
    line(i*2, height-100-10.0*(tempHist[i]-55.0), (i*2)+1, height-100-10.0*(tempHist[i+1]-55.0));
  }

  // sprit level
  fill(0, 0, 0);
  stroke(255, 255, 255);

  ellipse(width-130, 280, 180, 180);
  ellipse(width-130, 280, 45, 45);
  ellipse(width-130, 280, 90, 90);
  ellipse(width-130, 280, 45, 45);

  //  line(580, 280, 760, 280);
  //  line(670, 190, 670, 370);

  line(width-220, 280, width-40, 280);
  line(width-130, 190, width-130, 370);

  fill(255, 255, 0);
  stroke(255, 40, 40);

  //ellipse(670 + constrain (rawAccelX/10, -90, 90), 280 -constrain(rawAccelY/20, -90, 90), 5, 5);
  ellipse(width-130 + constrain (rawAccelX/10, -90, 90), 280 -constrain(rawAccelY/20, -90, 90), 5, 5);

  fill(0, 255, 255);
  stroke(25, 255, 40);
  ellipse(width-130  + constrain (rawGyroX, -90, 90), 280 -constrain(rawGyroY/2, -90, 90), 5, 5);



  fill(255, 255, 255);
  ambientLight(80, 80, 100);
  pointLight(255, 255, 255, 1000, -2000, 1000 );
  translate(width/2, height/2-20, 100);

  rotateY(DMPYaw   );// + yawOffset);
  rotateX(DMPPitch );// + pitchOffset);

  gfx.origin(new Vec3D(), 200);
  noStroke();
  noSmooth();
  gfx.mesh(mesh, false);
}



void exit() {
      println("exiting");
      arduinoPort.write('S');
      delay(200);  
      arduinoPort.clear();
      arduinoPort.stop();
  super.exit();
}






public class secondApplet extends PApplet {
  
  String mess  = "Startup";

  public void setup() {
    size(300, 100);
     noLoop();
  }
  
  public void setText(String s)
  {
    mess = s;
  }
  
  public void draw() {
    background(30);
    text(mess,10,10); 
  }
}

public class PFrame extends JFrame {
  
  public void setText(String mess)
  {
    s.setText(mess);
    s.redraw();

  }
  
  public PFrame() {

    setBounds(0, 0, 300, 100);
    s = new secondApplet();
    add(s);
    s.init();
    println("birh");
    show();
  }
}




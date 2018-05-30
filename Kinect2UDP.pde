String version="Kinect2UDP v 1.0.5 - cc 2018 Simon Biggs";
// this app will run on a PC (Processing 3.x) with a Kinect V2 connnected. It will send skeleton data over UDP to other machines running any OS and app that can read UDP
// if you want you can compile this code. It's been tested and runs fine.

import KinectPV2.KJoint;  //for app to run you need to have downloaded and installed the following libraries in your Processing library folder
import KinectPV2.*;
import hypermedia.net.*; //library for UDP

KinectPV2 kinect;
UDP udp1,udp2;  //initialise the UDP ports - the receiving applications will need to be set to receive this data with the same settings. The data will need to be parsed.
int myPort1=8000,myPort2=8001,skeletonNumber,numberOfSkeletons;
String myIP="10.0.0.1",targetIP1="10.0.0.2",targetIP2="10.0.0.3";  //addresses of hist and target computers - more target computers can be added by duplicating code here and elsewhere
float zVal=500,rotX=PI,myX,myY,myZ,minX,maxX,minY,maxY,minZ=10.0,maxZ,xDim,yDim,zDim,camX,camY,camZ,lookX,lookY,lookZ; //various variables for the world-space, camera and spatial limits
color col;
ArrayList<KSkeleton> skeletonArray; //initialise skeleton arrays to contain joint data
KSkeleton skeleton;

void setup() {
  size(960,540,P3D);
  textSize(24);
  kinect=new KinectPV2(this); //initialise Kinect V2
  kinect.enableColorImg(true); //just so we can see what the camera can see
  kinect.enableSkeleton3DMap(true); //turn on skeleton tracking
  kinect.init();
  udp1=new UDP(this,myPort1,myIP); // initialise a UDP port for a target machine
  udp1.listen(true);
  udp2=new UDP(this,myPort2,myIP); // initialise another UDP port
  udp2.listen(true);
  cameraSetUp();
  println(version);
}

void draw() {
  background(0);
  image(kinect.getColorImage(), width-384, 0, 384, 216); //get the colour camera image from the Kinect
  pushMatrix(); //translate the scene to the center
  translate(width/2, height/2, 0); //centre the world
  scale(zVal); //arrange for the camera position
  rotateX(rotX); //arrange for the camera position
  skeletonArray=kinect.getSkeleton3d(); //get the skeleton data from the Kinect
  numberOfSkeletons=skeletonArray.size(); //get the number of skeletons
  for(skeletonNumber=0;skeletonNumber<6;skeletonNumber++){ //iterate through the skeletons
    if(skeletonNumber<numberOfSkeletons) {
      skeleton=skeletonArray.get(skeletonNumber); //get data for a specific skeleton
      if(skeleton.isTracked()){ //if the skeleton is live do the following
        KJoint[] joints=skeleton.getJoints(); //get the joint data
        col=skeleton.getIndexColor(); //get the colour data
        stroke(col);
        drawBody(joints); // draw the skeleton
        sendBody(joints); //send the data over UDP
      }
    } else {
      udp1.send(skeletonNumber + " " + false + " " + "absent" + " " + 0 + " " + 0.0 + " " + 0.0 + " " + 0.0 + " " + 0, targetIP1, myPort1); //send some empty data so the receiving systems know there is nothing there
      udp2.send(skeletonNumber + " " + false + " " + "absent" + " " + 0 + " " + 0.0 + " " + 0.0 + " " + 0.0 + " " + 0, targetIP1, myPort2);
    }
  }
  popMatrix();
  printData(); //print data about how big the current world is, as defined by the activity of the Kinect skeletons
}

void sendBody(KJoint[] joints) { //extracts and sends the joint data over UDP, working through all 25 joints
  //torso
  sendJoint(joints, KinectPV2.JointType_Head, "Head");
  sendJoint(joints, KinectPV2.JointType_Neck, "Neck");
  sendJoint(joints, KinectPV2.JointType_SpineShoulder, "SpineShoulder");
  sendJoint(joints, KinectPV2.JointType_SpineMid, "SpineMid");
  sendJoint(joints, KinectPV2.JointType_SpineBase, "SpineBase");
  //right arm
  sendJoint(joints, KinectPV2.JointType_ShoulderRight, "ShoulderRight");
  sendJoint(joints, KinectPV2.JointType_ElbowRight, "ElbowRight");
  sendJoint(joints, KinectPV2.JointType_WristRight, "WristRight");
  sendJoint(joints, KinectPV2.JointType_HandRight, "HandRight");
  sendJoint(joints, KinectPV2.JointType_HandTipRight, "HandRightTip");
  sendJoint(joints, KinectPV2.JointType_ThumbRight, "ThumbRight");
  //left arm
  sendJoint(joints, KinectPV2.JointType_ShoulderLeft, "ShoulderLeft");
  sendJoint(joints, KinectPV2.JointType_ElbowLeft, "ElbowLeft");
  sendJoint(joints, KinectPV2.JointType_WristLeft, "WristLeft");
  sendJoint(joints, KinectPV2.JointType_HandLeft, "HandLeft");
  sendJoint(joints, KinectPV2.JointType_HandTipLeft,"HandLeftTip");
  sendJoint(joints, KinectPV2.JointType_ThumbLeft, "ThumbLeft");
  //right leg
  sendJoint(joints, KinectPV2.JointType_HipRight, "HipRight");
  sendJoint(joints, KinectPV2.JointType_KneeRight, "KneeRight");
  sendJoint(joints, KinectPV2.JointType_AnkleRight, "AnkleRight");
  sendJoint(joints, KinectPV2.JointType_FootRight, "FootRight");
  //left leg
  sendJoint(joints, KinectPV2.JointType_HipLeft, "HipLeft");
  sendJoint(joints, KinectPV2.JointType_KneeLeft, "KneeLeft");
  sendJoint(joints, KinectPV2.JointType_AnkleLeft, "AnkleLeft");
  sendJoint(joints, KinectPV2.JointType_FootLeft, "FootLeft");
}

void sendJoint(KJoint[] joints, int jointType, String jointName) { //sends the joint data over UDP, one at a time
  myX=joints[jointType].getX(); //extract data into simple float variable
  myY=joints[jointType].getY();
  myZ=joints[jointType].getZ();
  myY=myY*-1; //correct for destination world-space (eg: Kinect world is upside down)
  myZ=myZ*-1;
  udp1.send(skeletonNumber + " " + true + " " + jointName + " " + jointType + " " + myX + " " + myY + " " + myZ + " " + col, targetIP1, myPort1); //send the data
  udp2.send(skeletonNumber + " " + true + " " + jointName + " " + jointType + " " + myX + " " + myY + " " + myZ + " " + col, targetIP2, myPort2);
  if(myX<minX) minX=myX; //set the world-scale variables - useful for calibration on detsination machines
  if(myX>maxX) maxX=myX;
  if(myY<minY) minY=myY;
  if(myY>maxY) maxY=myY;
  if(myZ<minZ) minZ=myZ;
  if(myZ>maxZ) maxZ=myZ;
}

void drawBody(KJoint[] joints) { //draw the skeletons on the screen
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);
  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);
  // Right Arm    
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);
  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);
  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);
  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);
}

void drawJoint(KJoint[] joints, int jointType, String jointName) { // draw the individual joint
  strokeWeight(0.01f);
  point(joints[jointType].getX(), joints[jointType].getY(), joints[jointType].getZ());
}

void drawBone(KJoint[] joints, int jointType1, int jointType2) { // draw lines between the joints
  strokeWeight(0.01f);
  line(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ(),joints[jointType2].getX(), joints[jointType2].getY(), joints[jointType2].getZ());
}

void drawHandState(KJoint joint) { // indicate whether the hand is open or closed
  handState(joint.getState());
  strokeWeight(0.1f);
  point(joint.getX(), joint.getY(), joint.getZ());
}

void handState(int handState) { // find the state of the hands
  switch(handState) {
  case KinectPV2.HandState_Open:
    stroke(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    stroke(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    stroke(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    stroke(100, 100, 100);
    break;
  }
}

void printData() { // print world data - useful for calibration on destination machines
  fill(255, 0, 0);
  text("framerate " + frameRate, 0, 30);
  text("minX " + minX, 0, 60);
  text("maxX " + maxX, 0, 90);
  text("minY " + minY, 0, 120);
  text("maxY " + maxY, 0, 150);
  text("minZ " + minZ, 0, 180);
  text("maxZ " + maxZ, 0, 210);
  xDim=maxX-minX;
  yDim=maxY-minY;
  zDim=maxZ-minZ;
  text("dimX " + xDim, 0, 240);
  text("dimY " + yDim, 0, 270);
  text("dimZ " + zDim, 0, 300);
  text("number of skeletons " + numberOfSkeletons, 0, 330);
  text("my IP address " + myIP, 0, 360);
  text("sending to IP address " + targetIP1 + " " + targetIP2, 0, 390);
}

void cameraSetUp() { //place the camera in its initial position
  camX=width/6.2;
  camY=height/2.15;
  camZ=width*-1.0;
  lookX=width/6.2;
  lookY=height/2.15;
  lookZ=180;
}

void stop() { //clean up the UDP connections when the app stops running
  udp1.close();
  udp2.close();
}
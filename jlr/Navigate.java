package jlr;

import java.util.ArrayList;

import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.mss.MotionSubsystem;
import robotinterface.mss.MotionSubsystemListener;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;

import robotinterface.vss.VisionSubsystem;
import robotinterface.vss.VisionSubsystemListener3D;
import robotinterface.vss.KeyPointPackage3D;
import robotinterface.vss.ObservedKeyPoint3D;




/**
* Navigate skeleton
*/
public class Navigate extends RobotController implements MotionSubsystemListener,VisionSubsystemListener3D {

    private int radius=30;  // Spiralen-Radius - wird beim Fahren ständig erhöht

    private boolean anyCollision=false;
    private boolean tactile=false;
    private double us=Double.NaN;
    private double posX=Double.NaN;
    private double posY=Double.NaN;
    private double posAng=Double.NaN;
    
    private double destX = Double.NaN;
    private double destY = Double.NaN;

    int changeVorzeichen = 1;

    public Navigate() {
        Robot.motionSubsystem.registerMotionListener(this);
        Robot.visionSubsystem.setTiming(VisionSubsystem.TRIGGERED,-1);

        try {
            Robot.visionSubsystem.registerVisionListener3D(this);
        }
        catch (UnsupportedOperationException e) {
            Robot.debugOut.println("Vision Subsystem does not provide 3D keypoints");
        }
    }


/**
* Returns a short description of this robot controller, e.g. about the author and configuration.
* @return (multiline) string description
*/
    public String getDescription() {
        return "Navigate Skeleton "+getClass().getName()+":\n"+
               "Drive the robot to a given coordinate";
    }


/**
* Indicates, that this controller requires a configuration. 
* @return true if this controllers requires a configuration; here, always true
*/
    public boolean requiresConfiguration() {
        return true;
    }


/**
* Pass a configuration to the robot controller. This method is executed directly after instantiation and before calling the run() the first time.
* The controller has to memorize the configuration for further calls of run().
* The configuration is a controller-dependent string, usually passed as command-line argument. The entire parsing has to be performed by the controller.
* @param params a configuration string in controller-dependent format
* @throws IllegalArgumentException if either the controller does not accept a configuration or the string is malformed
*/
    public void configure(String params) throws IllegalArgumentException {
        Robot.debugOut.println("Passed configuration: "+params);
        
        String[] paramsArr = params.split(";");
        if(paramsArr.length >= 2) {
        	destX = Double.valueOf(paramsArr[0]);
        	destY = Double.valueOf(paramsArr[0]);
        }
    }


/**
* Stop navigating.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void run() throws Exception {
    	while(isRunning()) {
    		while(!anyCollision) {
    	        Robot.motionSubsystem.sendCommand("fore 50");
    	        Thread.sleep(1000);
    		}
    		
    		anyCollision = false;
    		Robot.motionSubsystem.sendCommand("back 50");
    		Thread.sleep(2000);
    		changeVorzeichen *= -1; 
    		Robot.motionSubsystem.sendCommand("rotate " + Math.random() * 45 * changeVorzeichen);
    		Thread.sleep(1000);
    	}
        
        
        Thread.sleep(2500);
        
        // ... perform navigation

    }


/**
* Pause navigating.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void pause() throws Exception {
        Robot.debugOut.println("pause not implemented yet");

        // ... pause navigation
    }


/**
* Stop driving a spiral. Reset the circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void stop() throws Exception {
        Robot.debugOut.println("stop not implemented yet");

        // ... stop navigation
    }


/**
* Required to implement the <i>MotionSubsystemListener</i>. A (synchronous) response message appears from the motion subsystem. Response messages are a result of a 
* command sent to the motion subsystem.
* @param messages list of strings indicating the response, usually only a single, e.g. 'OK'
* @param responseType one of RESPONSE_TYPE_OK, RESPONSE_TYPE_FAILURE, RESPONSE_TYPE_OTHERS
* @throws Exception if something happens
*/
    public void mssResponse(ArrayList<String> messages,int responseType) throws Exception {
        if (responseType==MotionSubsystemListener.RESPONSE_TYPE_FAILURE)
             Robot.debugOut.println("Failure response "+messages.get(0));
    }

/**
* Required to implement the <i>MotionSubsystemListener</i>. An asynchronous message appears from the motion subsystem. Asynchronous message are periodically issued
* and indicate the current state and position.
* @param messages list of strings indicating the current state
* @param bundle the parsed messages that easily provides access to key and values
* @throws Exception if something happens
*/
    public void mssAsyncMessages(ArrayList<String> messages,AsyncMotionMessageBundle bundle) throws Exception {
    
        if (bundle.containsPos()) { 
            posX=bundle.getDouble(AsyncMotionMessage.X);
            posY=bundle.getDouble(AsyncMotionMessage.Y);
            posAng=bundle.getDouble(AsyncMotionMessage.ANG);
        }   
        
        if (bundle.containsType(AsyncMotionMessage.TACTIL)) 
            tactile=bundle.getBoolean(AsyncMotionMessage.TACTIL);

        if (bundle.containsType(AsyncMotionMessage.US)) 
            us=bundle.getDouble(AsyncMotionMessage.US);      
        
        if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL) || 
            bundle.containsType(AsyncMotionMessage.COLLISION_US)) {
             anyCollision=true;
        }

        // ... do something intelligent with these information
    }


/**
* Required to implement the <i>VisionSubsystemListener3D</i>.  A list of keypoints (3D) is passed for further computation.
* Caution: for information only - this feature is not available on the real Carbot. None of the information passed via this call should be used for own computations!
* @param keypoints3D keypoints (3D)
* @throws Exception if something happens
*/
    public void observedKeypoints3D(KeyPointPackage3D keypoints3D) throws Exception {

        // Korrektur der MSS-Position anhand der SLAM-Position
        if (isRunning())
            Robot.motionSubsystem.sendCommand("offset "+(keypoints3D.observationPosX-keypoints3D.mssPosX)+" "+(keypoints3D.observationPosY-keypoints3D.mssPosY)+" "+(keypoints3D.observationAngle-keypoints3D.mssAngle));

        for(ObservedKeyPoint3D p : keypoints3D.observedKeyPoints) {
        	Robot.debugOut.println("p: " + p.toString());
        }
        // ... do something intelligent with the keypoints
    }


}
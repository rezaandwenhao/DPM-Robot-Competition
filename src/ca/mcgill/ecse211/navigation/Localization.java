/**
 * This is the localization class.
 * It contains the falling edge method which will orient the robot towards zero degrees, using the ultrasonic sensor
 * It also contains the light localization which ensures the robot is parallel to the grid lines using 2 light sensors
 * @author Eden Ovadia
 * @author Eliot Bourachot
 * 
 */

package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.main.RingRetriever;
import ca.mcgill.ecse211.main.RingRetriever.Tunnel;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.robotics.SampleProvider;

public class Localization {
	
	// Class Relations
	private Navigation nav;
	private Odometer odo;

	// Objects
	private SampleProvider lightMeanL;
	private SampleProvider lightMeanR;
	private SampleProvider usMean;
	private float[] lightDataL;
	private float[] lightDataR;
	private float[] usData;

	// Parameters 
	private static final int CORRECTION_ANGLE1 = 225;
	private static final int CORRECTION_ANGLE2 = 45;
	private static final int CORRECTION_TIMEOUT = 3000;
	private static final int VOID_THRESHOLD = 30;
	private static final int VOID_BAND = 3;
	private static final int LIGHT_THRESHOLD = 5; // %
	private static final int FORWARD_CORRECTION_SPEED = 80;
	private static final int ROTATE_CORRECTION_SPEED = 50;

	
	// Global variables
	private boolean pastView = false;
	private static float lightL;
	private static float nowLightL;
	private static float pastLightL = -1;
	private static float lightR;
	private static float nowLightR;
	private static float pastLightR = -1;
	
	
	public Localization(SampleProvider lightMeanL, SampleProvider lightMeanR, SampleProvider usMean, 
			float[] lightDataL, float[] lightDataR, float[] usData, Navigation nav, Odometer odo, TextLCD lcd) {
		this.lightMeanL = lightMeanL;
		this.lightMeanR = lightMeanR;
		this.usMean = usMean;
		this.lightDataL = lightDataL;
		this.lightDataR = lightDataR;
		this.usData = usData;
		this.nav = nav;
		this.odo = odo;
	}
	
	/**
	 * This method runs a falling edge localization procedure that follows these instructions:
	 * 
	 * When it starts, If the robot is facing a wall, turn clockwise towards void and a bit more.
	 * Then Rotate clockwise until robot detects  a falling edge, record angle at the falling edge.
	 * Turn counter clockwise until robot detects a second falling edge, record the angle.
	 * Perform calculations to find zero degree angle.
	 * Turn to zero degrees.
	 */
	public void fallingEdge() {
		
		if (seeingSomething()) {
			nav.rotate(true, 360, false); // rotate clockwise
			while (seeingSomething()); // if we're facing the wall, turn to face the void
			nav.stopMotors();
			
			nav.rotate(true, 10, true); // rotate a bit more
			odo.setTheta(odo.getXYT()[2]); // reset 0 angle
		}
		
		nav.rotate(true, 360, false); // rotate clockwise
		while(!seeingSomething());
		nav.stopMotors();
		
		Sound.beep();
		double angle1 = odo.getXYT()[2];

		nav.rotate(false, 360, false); // rotate counter-clockwise
		long snapshot = System.currentTimeMillis();
		while(seeingSomething() || (System.currentTimeMillis() - snapshot > 1000));
		nav.stopMotors();
		
		nav.rotate(false, 360, false); // rotate counter-clockwise
		while(!seeingSomething());
		nav.stopMotors();
		
		Sound.beep();
		double angle2 = odo.getXYT()[2];

		double deltaTheta = getHeading(angle1, angle2);
		
		odo.setTheta(deltaTheta+odo.getXYT()[2]-RingRetriever.ULTRASONIC_OFFSET);
		
		nav.turnTo(0);
	}
	/**
	 * This method ensures the robot stays on track, it uses 2 light sensors above each wheel to do so.
	 * When using this method, the robot will move until one of the sensors detects a line.
	 * The corresponding wheel will stop, until the other wheel crosses the line as well.
	 * This straightens the robot, and makes it exactly parallel to the grid lines.
	 * 
	 */
	public void lightCorrection() {
		nav.stopMotors();
	   
		// Move forwards until first sensor hits line
		nav.move(true, true, true, false, 30, FORWARD_CORRECTION_SPEED);
		
		boolean leftLightDetected = false;
		boolean rightLightDetected = false; 
		while (!leftLightDetected && !rightLightDetected) { // move forward until you hit a black band
			leftLightDetected = seeingLine(true);
			rightLightDetected = seeingLine(false);
		}
		Sound.beep();
		nav.stopMotors();
		
		// Move whichever sensor didn't hit line until it hits the line
		if (leftLightDetected && rightLightDetected) {
			nav.stopMotors();
		} else if (leftLightDetected) {
			nav.move(true, false, false, true, 3, FORWARD_CORRECTION_SPEED);
			nav.move(true, false, true, false, 10, ROTATE_CORRECTION_SPEED);
			while (!seeingLine(false));
			Sound.beep();
			nav.stopMotors();
		} else if (rightLightDetected) {
			nav.move(true, false, false, true, 3, FORWARD_CORRECTION_SPEED);
			nav.move(true, false, true, false, 10, ROTATE_CORRECTION_SPEED);
			while (!seeingLine(true));
			Sound.beep();
			nav.stopMotors();
		}
		Sound.beepSequence();
	}
	
	/**
	 * This method is used to tell if the robot is facing a wall or a void.
	 * @return true if the robot detects a wall, false if its not.
	 * This is called in the fallingEdge method
	 * 
	 */
	boolean seeingSomething() {
		usMean.fetchSample(usData, 0); // acquire data
		int distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
		
		if (distance > VOID_THRESHOLD + VOID_BAND) {
			this.pastView = false;
			return false;
		} else if (distance < VOID_THRESHOLD - VOID_BAND) {
			this.pastView = true;
			return true;
		} else {
			return pastView;
		}
	}
	
	boolean seeingLine(boolean left) {
		if (left) {
			lightMeanL.fetchSample(lightDataL,0);	// acquire data
			nowLightL=lightDataL[0]*100;
			if(100*Math.abs(nowLightL - pastLightL)/pastLightL > LIGHT_THRESHOLD){
				if (nowLightL < pastLightL){
					System.out.println("LEFT!");
					pastLightL = -1;
					return true;
				}
			}
			pastLightL = nowLightL;
			return false;
		} else {
			lightMeanR.fetchSample(lightDataR,0);	// acquire data
			nowLightR=lightDataR[0]*100;
			if(100*Math.abs(nowLightR - pastLightR)/pastLightR > LIGHT_THRESHOLD){
				if (nowLightR < pastLightR){
					System.out.println("RIGHT!");
					pastLightR = -1;
					return true;
				}
			}
			pastLightR = nowLightR;
			return false;
		}
	}
	
	/**
	 * This method takes the two angles detected during the falling edge and calculates zero degrees
	 * @return the correction needed to make the heading of the robot true North
	 * @param angle1
	 * @param angle2
	 */
	private double getHeading(double angle1, double angle2) {
		if (angle1 < angle2) {
			return CORRECTION_ANGLE1-((angle1+angle2)/2);
		} else {
			angle1 %= 360;
			return CORRECTION_ANGLE2-((angle1+angle2)/2);
		}
	}

	/**
	 * ATTENTION: REQUIRES ROBOT TO BE AT "ENTRANCE" OR "EXIT" OF TUNNEL!!!
	 * 1. rotate to known angle
	 * 2. correct angle by light correcting
	 * 3. go to middle of square
	 * 4. rotate to tunnel
	 * 5. correct angle by light correcting
	 * If entering, we now should be in front of the tunnel and aligned well enough to go through it.
	 * If exiting, we now should be facing opposite of the tunnel and aligned on a black line.
	 * @param side: side of tunnel on which you are
	 * @param entering: true if entering the tunnel, false if exiting
	 */
	public void tunnelLocalization(double[] directionInfo, boolean correctOdometer) {
		// localize to a black line perpendicular to tunnel
		nav.turnTo(directionInfo[2]-90);
		lightCorrection();
		
		
		nav.move(true, true, false, true, RingRetriever.HALF_TILE_SIZE-RingRetriever.LIGHT_SENSOR_Y_OFFSET, RingRetriever.FORWARD_SPEED); // move backwards
		nav.turnTo(directionInfo[2]);
		lightCorrection();

        if (correctOdometer) {
          nav.move(true, true, false, true, RingRetriever.HALF_TILE_SIZE-RingRetriever.LIGHT_SENSOR_Y_OFFSET, RingRetriever.FORWARD_SPEED); // move backwards
          odo.setTheta(directionInfo[2]);
          odo.setX(directionInfo[0]*RingRetriever.TILE_SIZE);
          odo.setY(directionInfo[1]*RingRetriever.TILE_SIZE);
        }

	}
	
	/**
	 * 
	 * @param x
	 * @return true if location is in a zone, false if in water or past wall
	 */
	private boolean isLocationAvailable(double x, double y) {
		boolean available = true;
		
		double zoneURx = RingRetriever.zoneURx*RingRetriever.TILE_SIZE;
		double zoneURy = RingRetriever.zoneURy*RingRetriever.TILE_SIZE;
		double zoneLLx = RingRetriever.zoneLLx*RingRetriever.TILE_SIZE;
		double zoneLLy = RingRetriever.zoneLLy*RingRetriever.TILE_SIZE;
		double islandURx = RingRetriever.islandLLx*RingRetriever.TILE_SIZE;
		double islandURy = RingRetriever.islandURy*RingRetriever.TILE_SIZE;
		double islandLLx = RingRetriever.islandLLx*RingRetriever.TILE_SIZE;
		double islandLLy = RingRetriever.islandLLy*RingRetriever.TILE_SIZE;

		if (x < zoneURx && y < zoneURy && x > zoneLLx && y > zoneLLy) return available;
		if (x < islandURx && y < islandURy && x > islandLLx && y > islandLLy) return available;
		
		return false;
	}	
}

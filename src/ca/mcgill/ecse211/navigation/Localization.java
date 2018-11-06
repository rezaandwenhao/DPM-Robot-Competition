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
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class Localization {
	
	// Class Relations
	private Navigation nav;

	// Objects
	private SampleProvider lightMeanL = null;
	private SampleProvider lightMeanR = null;
	private SampleProvider usMean = null;
	private float[] lightDataL = null;
	private float[] lightDataR = null;;
	private float[] usData = null;

	// Parameters 
	private static final int CORRECTION_ANGLE1 = 225;
	private static final int CORRECTION_ANGLE2 = 45;
	private static final int VOID_THRESHOLD = 30;
	private static final int VOID_BAND = 3;
	private static final int COLOR_THRESHOLD = 17;
	private static final int LIGHT_X_OFFSET = 5;
	private static final int LIGHT_Y_OFFSET = 5;
	
	// Global variables
	private boolean pastView = false;
	
	public Localization() {
		
	}
	
	public void setLightSensors(SampleProvider lightMeanL, SampleProvider lightMeanR, float[] lightDataL, float[] lightDataR) {
		this.lightMeanL = lightMeanL;
		this.lightMeanR = lightMeanR;
		this.lightDataL = lightDataL;
		this.lightDataR = lightDataR;
	}
	
	public void setUltrasonicSensor(SampleProvider usMean, float[] usData) {
		this.usMean = usMean;
		this.usData = usData;
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
			RingRetriever.odoData.setTheta(RingRetriever.odoData.getXYT()[2]); // reset 0 angle
		}
		
		nav.rotate(true, 360, false); // rotate clockwise
		while(!seeingSomething());
		nav.stopMotors();
		
		Sound.beep();
		double angle1 = RingRetriever.odoData.getXYT()[2];

		nav.rotate(false, 360, false); // rotate counter-clockwise
		long snapshot = System.currentTimeMillis();
		while(seeingSomething() || (System.currentTimeMillis() - snapshot > 1000));
		nav.stopMotors();
		
		nav.rotate(false, 360, false); // rotate counter-clockwise
		while(!seeingSomething());
		nav.stopMotors();
		
		Sound.beep();
		double angle2 = RingRetriever.odoData.getXYT()[2];

		double deltaTheta = getHeading(angle1, angle2);
		
		RingRetriever.odoData.setTheta(deltaTheta+RingRetriever.odoData.getXYT()[2]);
		
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
	   
	    // Read both sensors once at first
	    lightMeanL.fetchSample(lightDataL, 0); // acquire data
		int lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
		
		lightMeanR.fetchSample(lightDataR, 0); // acquire data
		int lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
		
		// Move forwards until first sensor hits line
		nav.move(true, true, true, false, 30, 30);
		while (lightL > COLOR_THRESHOLD && lightR > COLOR_THRESHOLD) { // move forward until you hit a black band
			lightMeanL.fetchSample(lightDataL, 0); // acquire data
			lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
			lightMeanR.fetchSample(lightDataR, 0); // acquire data
			lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
		}
		Sound.beep();
		nav.stopMotors();
		
		// Move whichever sensor didn't hit line until it hits the line
		if (lightL > COLOR_THRESHOLD) {
			nav.move(true, false, true, false, 10, 30);
			while (lightL > COLOR_THRESHOLD) {
				lightMeanL.fetchSample(lightDataL, 0); // acquire data
				lightL = (int) (lightDataL[0] * 100.0); // extract from buffer, cast to int
			}
			Sound.beep();
			nav.stopMotors();
		} else if (lightR > COLOR_THRESHOLD) {
			nav.move(false, true, true, false, 10, 30);
			while (lightR > COLOR_THRESHOLD) {
				lightMeanR.fetchSample(lightDataR, 0); // acquire data
				lightR = (int) (lightDataR[0] * 100.0); // extract from buffer, cast to int
			}
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
	
}

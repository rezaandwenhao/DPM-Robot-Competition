package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	private static final int VOID_THRESHOLD = 30;
	private static final int VOID_BAND = 3;
	private static final int COLOR_THRESHOLD = 17;
	private static final int LIGHT_X_OFFSET = 5;
	private static final int LIGHT_Y_OFFSET = 5;
	
	// Global variables
	private boolean pastView = false;
	
	public Localization(SampleProvider lightMeanL, SampleProvider lightMeanR, SampleProvider usMean, 
			float[] lightDataL, float[] lightDataR, float[] usData) {
		this.lightMeanL = lightMeanL;
		this.lightMeanR = lightMeanR;
		this.usMean = usMean;
		this.lightDataL = lightDataL;
		this.lightDataR = lightDataR;
		this.usData = usData;
	}
	
	/**
	 * runs a falling edge localization procedure that follows these instructions:
	 * 
	 * Start -> If looking at wall, turn clockwise towards void and a bit more
	 * -> Rotate clockwise until you see a falling edge -> Record angle
	 * Turn counter clockwise until you see a falling edge -> Record angle
	 * Set new angle
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
		
		odo.setTheta(deltaTheta+odo.getXYT()[2]);
		
		nav.turnTo(0);
	}
	
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
	 * @return true if the robot is seeing a wall, false if its seeing the void
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

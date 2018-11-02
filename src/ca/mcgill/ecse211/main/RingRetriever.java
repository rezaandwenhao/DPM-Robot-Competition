package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

public class RingRetriever {

	// Enumerations
	enum Ring {ORANGE, BLUE, GREEN, YELLOW};
	enum Coordinate {UR, LL};
	enum Location {ZONE, BRIDGE, TREE, STARTING_CORNER};
	enum Team {RED, GREEN};
	
	// Parameters
	public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
	public static final double TRACK = 9.0; // (cm) measured with caliper
	
	// Objects
	private static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lightPort = LocalEV3.get().getPort("S2");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	
	/**
	 * This method initializes the odometer class and starts the thread
	 * It also initializes the ultrasonic sensors and uses a filter to reduce the fluctuations
	 * @throws OdometerExceptions
	 */
	public RingRetriever() throws OdometerExceptions{
		
		// Odometer
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// Initializing Ultrasonic Sensor and runs it in this thread
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usSample = usSensor.getMode("Distance"); 
		SampleProvider usMean = new MeanFilter(usSample, 5); // use a mean filter to reduce fluctuations
	    float[] usData = new float[usMean.sampleSize()]; // usData is the buffer in which data are returned
	}
	
	public static void main(String args[]) {
		
	}
	
	/**
	 * This method finds the rings on the tree and detects their colors
	 * The colors are each associated with a value, with orange being worth the most
	 */
	public void findRings() {
		
	}
	 
	/**
	 * This method loads the rings into the basket section of the robot
	 * This is done by raising the arms of the robot using the medium motor and taking the rings off the tree
	 * The way of retrieving the rings is different depending if the ring is on the lower branch or the upper branch
	 * The angle that the arms move towards the tree differs depending on the position of the ring
	 */
	public void loadRing() {
		
	}
	/**
	 *This method unloads the rings from the robot
	 *It will only be called when the robot is back in its home area
	 *The unloading of the rings will be done using a medium motor
	 *The motor will lower the back wall of the basket, then the robot will move back and forth until the rings are unloaded
	 *
	 */
	public void unloadRing() {
		
	}
	
	
}

package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.navigation.Localization;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * <h1>ECSE 211 - Design Principles and Methods</h1>
 * @author Eliott Bourachot
 *
 */
public class RingRetriever {

	// Enumerations
	enum Ring {ORANGE, BLUE, GREEN, YELLOW};
	enum Coordinate {UR, LL};
	enum Location {ZONE, BRIDGE, TREE, STARTING_CORNER};
	enum Team {RED, GREEN};
	
	// Parameters
	public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
	public static final double TRACK = 12.0; // (cm) measured with caliper
	public static final int FORWARD_SPEED = 150;
	public static final int ROTATE_SPEED = 70;
	public static final double TILE_SIZE = 30.48;
	public static final double LEFT_RADIUS = 2.13;
	public static final double RIGHT_RADIUS = 2.13;
	
	// Objects
	public static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3MediumRegulatedMotor medMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3LargeRegulatedMotor backMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final OdometerData odoData = new OdometerData();
	public static final Odometer odo = new Odometer();
	public static final Localization loc = new Localization();
	public static final Navigation nav = new Navigation();
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static final Port lightLPort = LocalEV3.get().getPort("S1");
	private static final Port lightRPort = LocalEV3.get().getPort("S4");
	private static final Port colorPort = LocalEV3.get().getPort("S3");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	// Update periods
	public static final long ODOMETER_PERIOD = 25; // odometer update period in ms
	
	public static void main(String args[]) {

		// Odometer
		Thread odoThread = new Thread(odo);
		odoThread.start();

		// Initializing Ultrasonic Sensor and runs it in this thread
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usSample = usSensor.getMode("Distance"); 
		SampleProvider usMean = new MeanFilter(usSample, 5); // use a mean filter to reduce fluctuations
	    float[] usData = new float[usMean.sampleSize()]; // usData is the buffer in which data are returned
	    
	    // Ultrasonic Localization
	    loc.setUltrasonicSensor(usMean, usData);
	    (new Thread() {
	    	public void run() {
	    		RingRetriever.loc.fallingEdge();
	    	}
	    }).start();
	    
	    // Initializing Ultrasonic Sensor and runs it in this thread
 		@SuppressWarnings("resource") // Because we don't bother to close this resource
 		SensorModes lightLSensor = new EV3ColorSensor(lightLPort); // usSensor is the instance
 		SampleProvider lightLSample = lightLSensor.getMode("Red"); 
 		SampleProvider lightLMean = new MeanFilter(lightLSample, 5); // use a mean filter to reduce fluctuations
 	    float[] lightLData = new float[lightLMean.sampleSize()]; // usData is the buffer in which data are returned
	    
 	    // Initializing Ultrasonic Sensor and runs it in this thread
 		@SuppressWarnings("resource") // Because we don't bother to close this resource
 		SensorModes lightRSensor = new EV3ColorSensor(lightRPort); // usSensor is the instance
 		SampleProvider lightRSample = lightRSensor.getMode("Red"); 
 		SampleProvider lightRMean = new MeanFilter(lightRSample, 5); // use a mean filter to reduce fluctuations
 	    float[] lightRData = new float[lightRMean.sampleSize()]; // usData is the buffer in which data are returned
 	    
	    // Navigate to origin
	    loc.setLightSensors(lightLMean, lightRMean, lightLData, lightRData);
	    (new Thread() {
	    	public void run() {
	    		RingRetriever.loc.lightCorrection();
	    	}
	    }).start();
	    
	    odoData.setXYT(odoData.getXYT()[0], 0, 0); // reset Y and Theta
	    nav.travelTo(0, 0);
	    nav.turnTo(0);
	    
	    nav.travelTo(5, 5);
	}
	
	public void findRings() {
		
	}
	
	public void loadRing() {
		
	}
	
	public void unloadRing() {
		
	}
	
	
}

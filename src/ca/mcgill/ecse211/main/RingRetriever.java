package ca.mcgill.ecse211.main;

import java.util.HashMap;
import java.util.Map;

import ca.mcgill.ecse211.navigation.Localization;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	public static final double TRACK = 16.8; // (cm) measured with caliper
	private static final double LIGHT_SENSOR_X_OFFSET = 3.5;
	private static final double LIGHT_SENSOR_Y_OFFSET = 4.75;
	
	// Objects
	private static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static final Port lightLPort = LocalEV3.get().getPort("S1");
	private static final Port lightRPort = LocalEV3.get().getPort("S4");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
<<<<<<< HEAD
	
	/**
	 * This method initializes the odometer class and starts the thread
	 * It also initializes the ultrasonic sensors and uses a filter to reduce the fluctuations
	 * @throws OdometerExceptions
	 */
	public RingRetriever() throws OdometerExceptions{
=======
	public static void main(String args[])  throws OdometerExceptions {
		
		Wifi wifi = new Wifi();
		Map data = wifi.run();
		
		lcd.drawString("red team: "+((Long) data.get("RedTeam")).intValue(), 0, 0);

		Button.waitForAnyPress();
>>>>>>> 546372f883f5b988fd1a54b493961772469ca127
		
		// Odometer
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		// Navigation
	    Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer);

		// Initializing Ultrasonic Sensor and runs it in this thread
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usSample = usSensor.getMode("Distance"); 
		SampleProvider usMean = new MeanFilter(usSample, 5); // use a mean filter to reduce fluctuations
	    float[] usData = new float[usMean.sampleSize()]; // usData is the buffer in which data are returned
	    
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
	   
 	    Localization ll = new Localization(lightLMean, lightRMean, usMean, lightLData, lightRData, usData, nav, odometer, lcd);
	   
 	    ll.fallingEdge();
		ll.lightCorrection();
	    odometer.setXYT(odometer.getXYT()[0], 1-LIGHT_SENSOR_Y_OFFSET, 0); // reset Y and Theta
	    nav.move(true, true, false, true, 5, Navigation.FORWARD_SPEED);
	    nav.rotate(true, 90, true);
	    ll.lightCorrection();
	    odometer.setXYT(1-LIGHT_SENSOR_X_OFFSET, odometer.getXYT()[1], 90);
	    nav.travelTo(1, 1);
	    nav.turnTo(0);
	    nav.travelTo(2, 2.2);
	    nav.turnTo(90);
	    nav.travelTo(4, 2);
	    
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

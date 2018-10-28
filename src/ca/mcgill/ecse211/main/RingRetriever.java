package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.lab4.Odometer;
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
	
	public RingRetriever() {
		
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
	
	public void findRings() {
		
	}
	
	public void loadRing() {
		
	}
	
	public void unloadRing() {
		
	}
	
	
}

package ca.mcgill.ecse211.main;

import java.util.HashMap;
import java.util.Map;

import ca.mcgill.ecse211.navigation.Localization;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * <h1>ECSE 211 - Design Principles and Methods</h1>
 * @author Eliott Bourachot
 * @edits Wenhao Geng
 *
 */
public class RingRetriever {

	// Enumerations
	enum Ring {ORANGE, BLUE, GREEN, YELLOW};
	enum Coordinate {UR, LL};
	enum Location {ZONE, BRIDGE, TREE, STARTING_CORNER};
	enum Team {RED, GREEN};
	public enum Tunnel {LEFT, RIGHT, ABOVE, BELOW};
	
	public static int startingCorner;
	public static int zoneLLx;
	public static int zoneLLy;
	public static int zoneURx;
	public static int zoneURy;
	public static int islandLLx;
	public static int islandLLy;
	public static int islandURx;
	public static int islandURy;
	public static int tunnelLLx;
	public static int tunnelLLy;
	public static int tunnelURx;
	public static int tunnelURy;
	public static int ringsetx;
	public static int ringsety;
	public static Tunnel tunnelEntrance;
	public static Tunnel tunnelExit;
	public static double[] tunnelEntranceCoordinates;
	public static double[] tunnelExitCoordinates;
	
	// Parameters
	public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
	public static final double TRACK = 16.8; // (cm) measured with caliper
	public static final double LIGHT_SENSOR_X_OFFSET = 3.5;
	public static final double LIGHT_SENSOR_Y_OFFSET = 8;
	
	public static final int FORWARD_SPEED = 150;
	public static final int ROTATE_SPEED = 70;
	public static final double TILE_SIZE = 30.48;
	public static final double HALF_TILE_SIZE = 15.24;
	public static final int BOARD_WIDTH = 8;
	public static final int BOARD_HEIGHT = 8;
	public static final int ULTRASONIC_OFFSET = -90;
	public static final int FILTER_LIMIT = 10;
	
	// Objects
	private static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor backMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3MediumRegulatedMotor medMotor = 
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static final Port lightLPort = LocalEV3.get().getPort("S1");
	private static final Port lightRPort = LocalEV3.get().getPort("S4");
	private static final Port colorPort = LocalEV3.get().getPort("S3");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	
	/**
	 * This method initializes the odometer class and starts the thread
	 * It also initializes the ultrasonic sensors and uses a filter to reduce the fluctuations
	 * @throws OdometerExceptions
	 */
	public static void main(String args[])  throws OdometerExceptions {	  
   	    Wifi wifi = new Wifi();
		Map data = wifi.getData();
		
		fillGlobalData(data);
		
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
	    
	    // Initializing Left Light Sensor and runs it in this thread
 		@SuppressWarnings("resource") // Because we don't bother to close this resource
 		SensorModes lightLSensor = new EV3ColorSensor(lightLPort); // usSensor is the instance
 		SampleProvider lightLSample = lightLSensor.getMode("Red"); 
 		SampleProvider lightLMean = new MeanFilter(lightLSample, 5); // use a mean filter to reduce fluctuations
 	    float[] lightLData = new float[lightLMean.sampleSize()]; // usData is the buffer in which data are returned
	    
 	    // Initializing Right Light Sensor and runs it in this thread
 		@SuppressWarnings("resource") // Because we don't bother to close this resource
 		SensorModes lightRSensor = new EV3ColorSensor(lightRPort); // usSensor is the instance
 		SampleProvider lightRSample = lightRSensor.getMode("Red"); 
 		SampleProvider lightRMean = new MeanFilter(lightRSample, 5); // use a mean filter to reduce fluctuations
 	    float[] lightRData = new float[lightRMean.sampleSize()]; // usData is the buffer in which data are returned
	   
 	    Localization ll = new Localization(lightLMean, lightRMean, usMean, lightLData, lightRData, usData, nav, odometer, lcd);
	   
		// localize to the closest intersection
 	    ll.fallingEdge();
		ll.lightCorrection();
	    odometer.setXYT(odometer.getXYT()[0], 0-LIGHT_SENSOR_Y_OFFSET, 0); // reset Y and Theta
	    nav.move(true, true, false, true, 5, FORWARD_SPEED);
	    nav.rotate(true, 90, true);
	    ll.lightCorrection();
	    odometer.setXYT(0-LIGHT_SENSOR_X_OFFSET, odometer.getXYT()[1], 90);
	    nav.travelTo(0, 0, true);
	    nav.turnTo(0);
	    
	    // correct position depending on given start corner
		odometer.setStartingCoordinates(startingCorner);
		
		Sound.beep();Sound.beep();Sound.beep();
		
		// navigate to tunnel
		// entranceInfo is X,Y,THETA where THETA is the angle of the entrance to the tunnel
		double[] entranceInfo = getEntrance();
		nav.travelTo(entranceInfo[0], entranceInfo[1], true);
		
		// localize to "entrance" of tunnel
		ll.tunnelLocalization(entranceInfo, false);
		
		// move through tunnel
	    nav.move(true, true, true, true, TILE_SIZE, FORWARD_SPEED);        
	    nav.move(true, true, true, true, TILE_SIZE, ROTATE_SPEED);
		nav.move(false, true, true, true, 1, ROTATE_SPEED); // brute force offset, turn left a bit in the tunnel
		nav.move(true, true, true, true, TILE_SIZE*3-LIGHT_SENSOR_Y_OFFSET, FORWARD_SPEED);
		
		// localize to exit of tunnel
		double[] exitInfo = getExit();
		exitInfo[2] = exitInfo[2]-180; // reverse theta to have robot end point away from tunnel
		ll.tunnelLocalization(exitInfo, true);
		
		// determine which point around the tree is closest and determine a 'route'
		double[] startingPoint = {exitInfo[0], exitInfo[1]}; // this point should be where the robot is currently
		double[][] route = getFastestRoute(startingPoint);
		
		// navigate to tree
		nav.travelTo(route[0][0], route[0][1], true);
		
		// Initializing Color Sensor and runs it in this thread
 		@SuppressWarnings("resource") // Because we don't bother to close this resource
 		SensorModes colorSensor = new EV3ColorSensor(colorPort); // usSensor is the instance
 		SampleProvider colorSample = colorSensor.getMode("RGB"); 
 		SampleProvider colorMean = new MeanFilter(colorSample, 5); // use a mean filter to reduce fluctuations
 	    float[] colorData = new float[colorMean.sampleSize()]; // usData is the buffer in which data are returned
		
		// move light sensor to top row of rings height (should already be there ?)
		
 	    // initialize ringArray which will be 4x2 array
 	    // the first [] corresponds to the route on which the ring was found
 	    // the second [] corresponds to the height at which the ring was found (0 being top and 1 being bottom)
 	    // see diagram B
 	    int ringArray[][] = { { Color.NONE, Color.NONE }, { Color.NONE, Color.NONE}, { Color.NONE, Color.NONE}, { Color.NONE, Color.NONE } };
 	    
 	    int colourFilter = 0; int pastColour = Color.NONE;
 	    int sensorRotate = -60;
		// walk around tree to detect rings on top and bottom level
    	int path[] = {1,2,3,0};
    	for (int level = 0; level<2; level++) {
	    	for (int side=0; side<path.length; side++) {
	    		nav.travelTo(route[path[side]][0], route[path[side]][1], false);
		    	while(leftMotor.isMoving() && rightMotor.isMoving()) {
		    	  if (colourFilter < FILTER_LIMIT) {
		    	    ringArray[path[side]][level] = detectRing(colorMean, colorData);
		    	    if (ringArray[path[side]][level] == pastColour && pastColour != Color.NONE) {
		    	      colourFilter++;
		    	      if (colourFilter == FILTER_LIMIT) beepRing(ringArray[path[side]][level]);
		    	    } else {
		    	      colourFilter = 0;
		    	    }
		    	    pastColour = ringArray[path[side]][level];
		    	  }
		    	  
				}
		    	colourFilter = 0;
		    	
				usMean.fetchSample(usData, 0); // acquire data
				int distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
				if (distance > 30) {
					nav.move(true, true, false, true, LIGHT_SENSOR_Y_OFFSET+4, FORWARD_SPEED); // back up to be able to do localization
			    	ll.lightCorrection();
			    	nav.move(true, true, false, true, TILE_SIZE*0.4-LIGHT_SENSOR_Y_OFFSET, FORWARD_SPEED);
			        odometer.setXYT(route[path[side]][0]*TILE_SIZE, route[path[side]][1]*TILE_SIZE, getCorrectedTheta(odometer)); // update the x, y and theta
				}
	    	}
	    	backMotor.rotate(sensorRotate); // move light sensor to bottom row of rings height
	 		// TODO: Think of how to lower arm if there's a wall in the way
	    	sensorRotate = -sensorRotate;
      	}

    	// picking up rings
    	int currentSide= 0;
    	while (ringsRemaining(ringArray) > 0) {
    		// find most valuable ring
    		int[] sideAndLevel = getMostValuableRing(ringArray); // array is [side,level]
    		int side = sideAndLevel[0];
    		System.out.println("Traveling to side #: "+side);
    		int level = sideAndLevel[1];
        	ringArray = resetRing(sideAndLevel[0], sideAndLevel[1], ringArray); // reset ring value so we don't pick it up again
        	
        	while(!canTravelStraight(currentSide, side)) {
        		currentSide = (currentSide+1)%4;
        		System.out.println("Go through side #: "+currentSide);
        		nav.travelTo(route[currentSide][0], route[currentSide][1], true);
        	}
        	
        	double middle[] = {(route[side][0]+route[currentSide][0])/2, (route[side][1]+route[currentSide][1])/2 };
    		System.out.println("Middle is: ("+middle[0]+","+middle[1]+")");
        	nav.travelTo(middle[0]+0.11, middle[1], true); // travel to middle of square
        	
        	nav.rotate(true, 90, true); // rotate towards tree
        	nav.move(true, true, false, true, 11, FORWARD_SPEED); // TODO: test value // back off a bit to lower arm
        	int rotateAngle = (level == 1) ? 70 : 40; // TODO: test values // lower arm depending on level
    		medMotor.rotate(rotateAngle); //TODO: test value // lower arm
        	nav.move(true, true, true, true, 8, FORWARD_SPEED); // TODO: test value // move close enough to ring
        	medMotor.rotate(-rotateAngle); // bring arm back up
        	nav.move(true, true, false, true, 8, FORWARD_SPEED);
        	nav.travelTo(middle[0], middle[1], true); // travel back to middle
    		nav.travelTo(route[currentSide][0], route[currentSide][1], true); // travel back to starting corner
    	}

	}
	
	private static double getCorrectedTheta(Odometer odo) {
	  double theta = odo.getXYT()[2];
	  if (330 < theta || theta < 30) return 0;
	  else if (60 < theta && theta < 120) return 90;
	  else if (150 < theta && theta < 210) return 180;
	  else if (240 < theta && theta < 300) return 270;
	  else return theta;
	}

	private static boolean canTravelStraight(int from, int to) {
		if (from == to || from+1 == to) {
			return true;
		} else {
			return false;
		}
	}
	
	private static int ringsRemaining(int[][] ringArray) {
		int counter = 0;
		for (int level=0; level<2; level++) {
			for (int side=0; side<4; side++) {
				if(getValue(ringArray[side][level]) > 0) counter++;
			}
		}
		return counter;
	}
	
	/**
	 * 
	 * @param ringArray
	 * @return array of {side,level} of most valuable ring
	 */
	private static int[] getMostValuableRing(int[][] ringArray) {
		int maxRing_side = 0;
		int maxRing_level = 0;
		for (int level=0; level<2; level++) {
			for (int side=0; side<4; side++) {
				if (getValue(ringArray[maxRing_side][maxRing_side]) < getValue(ringArray[side][level])) {
					maxRing_level = level;
					maxRing_side = side;
				}
			}
		}
		int[] returnValue = {maxRing_side, maxRing_level};
		return returnValue;
	}
	
	private static int[][] resetRing(int side, int level, int[][] ringArray) {
		ringArray[side][level] = Color.NONE;
		return ringArray;
	}
	
	private static int getValue(int ring) {
		switch (ring) {
		case Color.ORANGE:
			return 4;
		case Color.YELLOW:
			return 3;
		case Color.GREEN:
			return 2;
		case Color.BLUE:
			return 1;
		default:
			return 0;
		}
	}

	/**
	 * Beeps a certain number of times per ring according to the following
	 * Orange: 4, Yellow: 3, Green: 2, Blue: 1, Others: 0
	 * @param ring
	 */
	private static void beepRing(int ring) {
		int numberOfBeeps = getValue(ring);
		for (int i=0; i<numberOfBeeps; i++) Sound.beep();
	}

	/**
	 * Samples the color sensor in RGB mode and returns the color detected
	 * @param colorMean
	 * @param colorData
	 * @return Color (as an int according to lejos library)
	 */
	private static int detectRing(SampleProvider colorMean, float[] colorData) {
		colorMean.fetchSample(colorData, 0); // acquire data
	
		//normalize the data
        float red = (float) (colorData[0]/(Math.sqrt(Math.pow(colorData[0], 2) + Math.pow(colorData[2], 2) + Math.pow(colorData[1], 2))));
//        float green = (float) (colorData[1]/(Math.sqrt(Math.pow(colorData[0], 2) + Math.pow(colorData[2], 2) + Math.pow(colorData[1], 2))));
        float blue = (float) (colorData[2]/(Math.sqrt(Math.pow(colorData[0], 2) + Math.pow(colorData[2], 2) + Math.pow(colorData[1], 2))));
        
        // classify color according to testing calibrations
        if (0.07 < red && red < 0.18 && 0.49 < blue && blue < 0.64) {
          return Color.BLUE;
        } else if (0.34 < red && red < 0.43 && 0.07 < blue && blue < 0.15) {
          return Color.GREEN;
        } else if (0.76 < red && red < 0.85 && 0.09 < blue && blue < 0.14) {
          return Color.YELLOW;
        } else if (0.92 < red && red < 0.98 && 0.06 < blue && blue < 0.1) {
          return Color.ORANGE;
        }         
		return Color.NONE;
	}
	
	/**
	 * 
	 * @param starting
	 * @param destinations
	 * @return list of coordinates in tiles (not cm) in clockwise rotation order. The 0th coordinate is the closest
	 * coordinate to the starting point. 
	 */
	private static double[][] getFastestRoute(double[] starting) {
		
	    //Note from Wenhao: I actully think this value does not really matter that much, what actually control the distance to the tree 
	    // is the call nav.move(true, true, false, true, TILE_SIZE*0.4-LIGHT_SENSOR_Y_OFFSET, FORWARD_SPEED); at line 207 and 233
	  
	    // I argue to lower it to 0.6-0.7 should work too, and is an more accurate odometer update
		double distanceFromTree=0.7;
		
		// determine 4 points around tree and their distances from the given starting point
		double[] bottomLeft = {ringsetx-distanceFromTree, ringsety-distanceFromTree};
		double distanceBL = Math.abs(starting[0]-bottomLeft[0]) + Math.abs(starting[1]-bottomLeft[1]);
		double[] topLeft = {ringsetx-distanceFromTree, ringsety+distanceFromTree};
		double distanceTL = Math.abs(starting[0]-topLeft[0]) + Math.abs(starting[1]-topLeft[1]);
		double[] topRight = {ringsetx+distanceFromTree, ringsety+distanceFromTree};
		double distanceTR = Math.abs(starting[0]-topRight[0]) + Math.abs(starting[1]-topRight[1]);
		double[] bottomRight = {ringsetx+distanceFromTree, ringsety-distanceFromTree};
		double distanceBR = Math.abs(starting[0]-bottomRight[0]) + Math.abs(starting[1]-bottomRight[1]);

		// find which point is the closest
		double min = Math.min (Math.min(distanceBL, distanceTL), Math.min(distanceTR, distanceBR));
		
		double[][] coordinates = {bottomLeft,topLeft,topRight,bottomRight};
		
		if (min == distanceTL) {
			coordinates[0] = topLeft; coordinates[1] = topRight; coordinates[2] = bottomRight; coordinates[3] = bottomLeft;
		}
		if (min == distanceTR) {
			coordinates[0] = topRight; coordinates[1] = bottomRight; coordinates[2] = bottomLeft; coordinates[3] = topLeft;
		}
		
		if (min == distanceBR) {
			coordinates[0] = bottomRight; coordinates[1] = bottomLeft; coordinates[2] = topLeft; coordinates[3] = topRight;
		}
		
		return coordinates;
	}


	public static void fillGlobalData(Map data) {
		if (((Long) data.get("RedTeam")).intValue()== Wifi.TEAM_NUMBER) {
			startingCorner = ((Long)data.get("RedCorner")).intValue();
			zoneLLx = ((Long)data.get("Red_LL_x")).intValue();
			zoneLLy = ((Long)data.get("Red_LL_y")).intValue();
			zoneURx = ((Long)data.get("Red_UR_x")).intValue();
			zoneURy = ((Long)data.get("Red_UR_y")).intValue();
			tunnelLLx = ((Long)data.get("TNR_LL_x")).intValue();
			tunnelLLy = ((Long)data.get("TNR_LL_y")).intValue();
			tunnelURx = ((Long)data.get("TNR_UR_x")).intValue();
			tunnelURy = ((Long)data.get("TNR_UR_y")).intValue();
			ringsetx = ((Long)data.get("TR_x")).intValue();
			ringsety = ((Long)data.get("TR_y")).intValue();
		} else if (((Long) data.get("GreenTeam")).intValue()== Wifi.TEAM_NUMBER) {
            startingCorner = ((Long)data.get("GreenCorner")).intValue();
            zoneLLx = ((Long)data.get("Green_LL_x")).intValue();
            zoneLLy = ((Long)data.get("Green_LL_y")).intValue();
            zoneURx = ((Long)data.get("Green_UR_x")).intValue();
            zoneURy = ((Long)data.get("Green_UR_y")).intValue();
            tunnelLLx = ((Long)data.get("TNG_LL_x")).intValue();
            tunnelLLy = ((Long)data.get("TNG_LL_y")).intValue();
            tunnelURx = ((Long)data.get("TNG_UR_x")).intValue();
            tunnelURy = ((Long)data.get("TNG_UR_y")).intValue();
            ringsetx = ((Long)data.get("TG_x")).intValue();
            ringsety = ((Long)data.get("TG_y")).intValue();
		}
		islandLLx = ((Long)data.get("Island_LL_x")).intValue();
		islandLLy = ((Long)data.get("Island_LL_y")).intValue();
		islandURx = ((Long)data.get("Island_UR_x")).intValue();
		islandURy = ((Long)data.get("Island_UR_y")).intValue();
	}
	
	/**
	 * 
	 * @return coordinates of entrance of tunnel
	 * Tested on 11/10/2018 by Eliott Bourachot
	 */
	public static double[] getEntrance() {
		double[] coordinates = {0,0,0};
		
		boolean rightSide = isTunnelAtRelativeRight(); // true if tunnel is determined to be on the right side of our zone
		
		switch (startingCorner) {
		case 0:
			if (rightSide) {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
				coordinates[2] = 90;
			} else {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
				coordinates[2] = 0;
			}
			break;
		case 1:
			if (rightSide) {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
				coordinates[2] = 0;
			} else {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
				coordinates[2] = -90;
			}
			break;
		case 2:
			if (rightSide) {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
				coordinates[2] = -90;
			} else {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
				coordinates[2] = 180;
			}
			break;
		case 3:
			if (rightSide) {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
				coordinates[2] = 180;
			} else {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
				coordinates[2] = 90;
			}
			break;
		}
		return coordinates;
	}
	
	/**
	 * 
	 * @return
	 * Tested on 11/10/2018 by Eliott Bourachot
	 */
	public static double[] getExit() {
		double[] coordinates = {0,0,0};
	
		boolean rightSide = isTunnelAtRelativeRight(); // true if tunnel is determined to be on the right (relative) side of our zone
		
		switch (startingCorner) {
		case 0:
			if (rightSide) {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
				coordinates[2] = -90;
			} else {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
				coordinates[2] = 180;
			}
			break;
		case 1:
			if (rightSide) {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
				coordinates[2] = 180;
			} else {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
				coordinates[2] = 90;
			}
			break;
		case 2:
			if (rightSide) {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
				coordinates[2] = 90;
			} else {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
				coordinates[2] = 0;
			}
			break;
		case 3:
			if (rightSide) {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
				coordinates[2] = 0;
			} else {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
				coordinates[2] = -90;
			}
			break;
		}
		return coordinates;
	}
	
	/**
	 * 
	 * @return
	 * Tested on 11/10/2018 by Eliott Bourachot
	 */
	private static boolean isTunnelAtRelativeRight() {
		boolean rightSide = false;
		
		int maxLen = getMaxTunnelLength();

		// determine coordinates of all zones which correspond to the coordinates for corner 0
		// i.e. for corner 0, all of these will be unchanged, for corner 1, x and y will be swapped, etc...
		int rel_tunnelLLx = tunnelLLx;
		int rel_tunnelLLy = tunnelLLy;
		int rel_tunnelURx = tunnelURx;
		int rel_zoneURx = zoneURx;
		int rel_zoneURy = zoneURy;
		
		switch (startingCorner) {
		case 0: // x -> x, y -> y
			rel_tunnelLLx = tunnelLLx;
			rel_tunnelLLy = tunnelLLy;
			rel_tunnelURx = tunnelURx;
			rel_zoneURx = zoneURx;
			rel_zoneURy = zoneURy;
			break;
		case 1: // x -> y and y -> -x
			rel_tunnelLLx = tunnelLLy; 
			rel_tunnelLLy = BOARD_WIDTH-tunnelURx;
			rel_tunnelURx = tunnelURy;
			rel_zoneURx = zoneURy;
			rel_zoneURy = BOARD_WIDTH-zoneLLx;
			break;
		case 2: // x -> -x, y -> -y
			rel_tunnelLLx = BOARD_WIDTH-tunnelURx;
			rel_tunnelLLy = BOARD_HEIGHT-tunnelURy;
			rel_tunnelURx = BOARD_WIDTH-tunnelLLx;
			rel_zoneURx = BOARD_WIDTH-zoneLLx;
			rel_zoneURy = BOARD_HEIGHT-zoneLLy;
			break;
		case 3: // x -> -y, y -> x
			rel_tunnelLLx = BOARD_HEIGHT-tunnelURy;
			rel_tunnelLLy = tunnelLLx;
			rel_tunnelURx = BOARD_HEIGHT-tunnelLLy;
			rel_zoneURx = BOARD_HEIGHT-zoneLLy;
			rel_zoneURy = zoneURx;
			break;
		}
		
		// see diagram A for cases
		if (rel_tunnelLLx < rel_zoneURx-1) rightSide = false; // case 1
		else if (rel_tunnelLLy < rel_zoneURy-1) rightSide = true; // case 2
		else if (rel_tunnelLLx == rel_zoneURx && rel_tunnelLLy > rel_zoneURy-2) rightSide = true; // case 3 
		else if (maxLen == 1) rightSide = false; // case 4
		else if (rel_tunnelURx - rel_tunnelLLx == 2) rightSide = true; // case 5.1
		else rightSide = false; // case 5.2
		
		return rightSide;
	}
	
	/**
	 * 
	 * @return
	 * Tested on 11/10/2018 by Eliott Bourachot
	 */
	private static int getMaxTunnelLength() {
		int x_len = Math.abs(tunnelURx - tunnelLLx);
		int y_len = Math.abs(tunnelURy - tunnelURy);
		return Math.max(x_len, y_len);
	}
	
	
	/**
	 * 
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
	 * @param x
	 * @param y
	 * @param theta
	 * @param bottom
	 */
	public static void loadRing(double x, double y, double theta, double bottom) {
		
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


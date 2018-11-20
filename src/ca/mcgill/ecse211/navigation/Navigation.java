
/**
 * This is the navigation class.
 * It contains the falling edge method which will orient the robot towards zero degrees, using the ultrasonic sensor
 * It also contains the light localization which ensures the robot is parallel to the grid lines using 2 light sensors
 * @author Eden Ovadia
 * @author Eliot Bourachot
 * 
 */package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.main.RingRetriever;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation {
	  
	private EV3LargeRegulatedMotor motorL;
	private EV3LargeRegulatedMotor motorR;
	private double leftRadius;
	private double rightRadius;
	private double track;
	private Odometer odo;
	    
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			double leftRadius, double rightRadius, double track, Odometer odo) {
		this.motorL = leftMotor;
		this.motorR = rightMotor;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.track = track;
		this.odo = odo;
	}
	
	  
	/**
	 * This method makes the robot travel to the coordinates it received as arguments.
	 * The distance that needs to be traveled is calculated.
	 * The angle that the robot needs to turn is calculated in the turnTo method which is called.
	 * Rotates the robot to face the direction in which it needs to travel to.
	 * Then rotates the wheels the exact distance from the current position to using the move method.
	 * @param x (tiles): x coordinate of destination
	 * @param y (tiles): y coordinate of destination
	 */
	public void travelTo(double x, double y, boolean wait) {  
		double currentPos[] = odo.getXYT();
		  
		//difference in position
		double deltaX = x*RingRetriever.TILE_SIZE-currentPos[0];
		double deltaY = y*RingRetriever.TILE_SIZE-currentPos[1];
	  
		// length of straight line from current position to desired position
		double hypotenuse = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
	  
		//angle from hypotenuse to X axis
		if (deltaY == 0) deltaY = 0.0001;
		double desiredAngle = Math.atan(deltaX/deltaY)*57.2957795131d;
	  
		// adjust desired angle to desired position
		if (deltaY < 0) {
			desiredAngle += 180.0d;
		}

		turnTo(desiredAngle);
	  
		//move forwards to reach the point x,y.
		move(true, true, true, wait, hypotenuse, RingRetriever.FORWARD_SPEED);
	}
	
	/**
	 * Enables the robot to travel through the tunnel. The speeds and distances have been determined through rigourous testing.
	 */
	public void travelThroughTunnel() {
		move(true, true, true, true, RingRetriever.TILE_SIZE, RingRetriever.FORWARD_SPEED);
		move(true, true, true, true, RingRetriever.TILE_SIZE, RingRetriever.ROTATE_SPEED);
		move(false, true, true, true, 1, RingRetriever.ROTATE_SPEED); // brute force offset, turn left a bit in the tunnel
		move(true, true, true, true, RingRetriever.TILE_SIZE * 3 - RingRetriever.LIGHT_SENSOR_Y_OFFSET, RingRetriever.FORWARD_SPEED);
	}
	
	/**
	 * Enables the robot to travel from one corner of the tree to another without colliding with the tree.
	 * @param route
	 * @param currentSide: side on which the robot is currently
	 * @param side: side to which the robot desires to navigate to.
	 * @return true if the robot should face clockwise to reach its desired target ring, false otherwise
	 */
	public boolean squareTravel(double[][] route, int currentSide, int side) {
		boolean clockwise = false;
		// Answers the question: If we're at "currentSide" and we want to go to "side"
	    // how do we get there efficiently
	    switch (currentSide) {
	    case 0:
		switch (side) {
		case 0:
		    turnTo(route[3][0], route[3][1]);
		    clockwise = false;
		    break;
		case 1:
		    turnTo(route[1][0], route[1][1]);
		    clockwise = true;
		    break;
		case 2:
		    travelTo(route[1][0], route[1][1], true);
		    turnTo(route[2][0], route[2][1]);
		    clockwise = true;
		    break;
		case 3:
		    travelTo(route[3][0], route[3][1], true);
		    turnTo(route[2][0], route[2][1]);
		    clockwise = false;
		    break;
		}
		break;
	    case 1:
		switch (side) {
		case 0:
		    travelTo(route[0][0], route[0][1], true);
		    turnTo(route[3][0], route[3][1]);
		    clockwise = false;
		    break;
		case 1:
		    turnTo(route[0][0], route[0][1]);
		    clockwise = false;
		    break;
		case 2:
		    turnTo(route[2][0], route[2][1]);
		    clockwise = true;
		    break;
		case 3:
		    travelTo(route[2][0], route[2][1], true);
		    turnTo(route[3][0], route[3][1]);
		    clockwise = true;
		    break;
		}
		break;
	    case 2:
		switch (side) {
		case 0:
		    travelTo(route[3][0], route[3][1], true);
		    turnTo(route[0][0], route[0][1]);
		    clockwise = true;
		    break;
		case 1:
		    travelTo(route[1][0], route[1][1], true);
		    turnTo(route[0][0], route[0][1]);
		    clockwise = false;
		    break;
		case 2:
		    turnTo(route[1][0], route[1][1]);
		    clockwise = false;
		    break;
		case 3:
		    turnTo(route[3][0], route[3][1]);
		    clockwise = true;
		    break;
		}
		break;
	    case 3:
		switch (side) {
		case 0:
		    turnTo(route[0][0], route[0][1]);
		    clockwise = true;
		    break;
		case 1:
		    travelTo(route[0][0], route[0][1], true);
		    turnTo(route[1][0], route[1][1]);
		    clockwise = true;
		    break;
		case 2:
		    travelTo(route[2][0], route[2][1], true);
		    turnTo(route[1][0], route[1][1]);
		    clockwise = false;
		    break;
		case 3:
		    turnTo(route[2][0], route[2][1]);
		    clockwise = false;
		    break;
		}
		break;
	    }
	    return clockwise;
	}
	
	/**
	 * Enables the robot to turn to a set of given coordinates
	 * @param x
	 * @param y
	 */
	public void turnTo(double x, double y) {
		double currentPos[] = odo.getXYT();
		  
		//difference in position
		double deltaX = x*RingRetriever.TILE_SIZE-currentPos[0];
		double deltaY = y*RingRetriever.TILE_SIZE-currentPos[1];
	  
		//angle from hypotenuse to X axis
		if (deltaY == 0) deltaY = 0.0001;
		double desiredAngle = Math.atan(deltaX/deltaY)*57.2957795131d;
	  
		// adjust desired angle to desired position
		if (deltaY < 0) {
			desiredAngle += 180.0d;
		}

		turnTo(desiredAngle);
	}
	
	/**
	 * This method turns the robot from currentAngle to angle theta in the most efficient direction.
	 * Calculations are performed to ensure the robot is turning in the smallest possible angle.
	 * @param theta: angle at which to turn clockwise
	 */
	public void turnTo(double theta) {
		double toRotate = theta - odo.getXYT()[2];
		  
		if (toRotate > 180.0d) {
			toRotate -= 360.0d;
		} else if (toRotate < -180.0d) {
			toRotate += 360.0d;
		}
		motorL.setSpeed(RingRetriever.ROTATE_SPEED);
	    motorR.setSpeed(RingRetriever.ROTATE_SPEED);
	      
		motorL.rotate(convertAngle(leftRadius, track, toRotate), true);
	    motorR.rotate(-convertAngle(rightRadius, track, toRotate), false);
    }
	  
	/**
	 * This method rotates the robot either clockwise or counter clockwise, for a specified angle. 
	 * @param clockwise: a Boolean that if true signifies clockwise and if false signifies counter clockwise
	 * @param angle: the angle that the robot should turn
	 * @param wait: a Boolean that if true the robot should wait for the rotation to terminate before doing anything else.
	 */
	public void rotate(boolean clockwise, int angle, boolean wait) {
		motorL.setSpeed(RingRetriever.ROTATE_SPEED);
	    motorR.setSpeed(RingRetriever.ROTATE_SPEED);
		if (clockwise) {
			motorL.rotate(convertAngle(leftRadius, track, angle), true);
		    motorR.rotate(-convertAngle(rightRadius, track, angle), !wait);
		} else {
		  	motorL.rotate(-convertAngle(leftRadius, track, angle), true);
		   	motorR.rotate(convertAngle(rightRadius, track, angle), !wait);
		}
	}
	  
	/**
	 * This method moves the robot forwards for a specified distance.
	 * It is called in the travelTo method.
	 * @param leftMotor: a Boolean that if true signifies that the left motor should be used
	 * @param rightMotor: a Boolean that if true signifies that the right motor should be used
	 * @param forward: a Boolean that if true signifies that the robot should move forward, and if false the robot should move backwards
	 * @param wait: if the robot should wait for the rotation to terminate before doing anything else.
	 * @param distance (cm)
	 * @param speed
	 */
	public void move(boolean leftMotor, boolean rightMotor, 
			boolean forward, boolean wait, 
			double distance, int speed) {
		// speed 
		if (leftMotor) {
			motorL.setSpeed(speed);
		}
		if (rightMotor) {
			motorR.setSpeed(speed);
		}
	
		// direction
		int direction = -1;
		if (forward) direction = 1;
		
		if (leftMotor && rightMotor) {
			motorL.rotate(direction*convertDistance(leftRadius, distance), true);
		    motorR.rotate(direction*convertDistance(rightRadius, distance), !wait);
		} else if (leftMotor) {
			motorL.rotate(direction*convertDistance(leftRadius, distance), !wait);
		} else if (rightMotor) {
			motorR.rotate(direction*convertDistance(leftRadius, distance), !wait);
		}
	    
	}
	
	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius: a double for the radius of the wheel
	 * @param distance: the calculated distance the robot must travel
	 * @return an integer that signifies the total rotations of each wheel
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * stops the motors at the same time.
	 */
	public void stopMotors() {
		motorL.stop(true);
		motorR.stop(false);
	}
	
	/**
	 * 
	 * @return true of any of the motors used for navigation are moving, false otherwise.
	 */
	public boolean anyMotorMoving() {
		return (motorL.isMoving() || motorR.isMoving());
	}
}

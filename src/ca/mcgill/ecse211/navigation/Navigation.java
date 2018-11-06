
/**
 * This is the navigationn class.
 * It contains the falling edge method which will orient the robot towards zero degrees, using the ultrasonic sensor
 * It also contains the light localization which ensures the robot is parallel to the grid lines using 2 light sensors
 * @author Eden Ovadia
 * @author Eliot Bourachot
 * 
 */package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.main.RingRetriever;

/*
 * Navigation.java
 */

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation {
	    
	public Navigation() {
		
	}
	
	/**
	 * This method makes the robot travel to the coordinates it received as arguments.
	 * The distance that needs to be traveled is calculated.
	 * The angle that the robot needs to turn is calculated in the turnTo method which is called.
	 * Rotates the robot to face the direction in which it needs to travel to.
	 * Then rotates the wheels the exact distance from the current position to using the move method.
	 * @param x: x coordinate of destination
	 * @param y: y coordinate of destination
	 */
	public void travelTo(double x, double y) {  
		double currentPos[] = RingRetriever.odoData.getXYT();
		  
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
		move(true, true, true, true, hypotenuse, RingRetriever.FORWARD_SPEED);
	}
	  
	/**
	 * This method turns the robot from currentAngle to angle theta in the most efficient direction.
	 * Calculations are performed to ensure the robot is turning in the smallest possible angle.
	 * @param theta: angle at which to turn clockwise
	 */
	public void turnTo(double theta) {
		double toRotate = theta - RingRetriever.odoData.getXYT()[2];
		  
		if (toRotate > 180.0d) {
			toRotate -= 360.0d;
		} else if (toRotate < -180.0d) {
			toRotate += 360.0d;
		}
		RingRetriever.leftMotor.setSpeed(RingRetriever.ROTATE_SPEED);
	    RingRetriever.rightMotor.setSpeed(RingRetriever.ROTATE_SPEED);
	      
		RingRetriever.leftMotor.rotate(convertAngle(RingRetriever.LEFT_RADIUS, RingRetriever.TRACK, toRotate), true);
	    RingRetriever.rightMotor.rotate(-convertAngle(RingRetriever.RIGHT_RADIUS, RingRetriever.TRACK, toRotate), false);
    }
	  
	/**
	 * This method rotates the robot either clockwise or counter clockwise, for a specified angle. 
	 * @param clockwise: a Boolean that if true signifies clockwise and if false signifies counter clockwise
	 * @param angle: the angle that the robot should turn
	 * @param wait: a Boolean that if true the robot should wait for the rotation to terminate before doing anything else.
	 */
	void rotate(boolean clockwise, int angle, boolean wait) {
		RingRetriever.leftMotor.setSpeed(RingRetriever.ROTATE_SPEED);
	    RingRetriever.rightMotor.setSpeed(RingRetriever.ROTATE_SPEED);
		if (clockwise) {
			RingRetriever.leftMotor.rotate(convertAngle(RingRetriever.LEFT_RADIUS, RingRetriever.TRACK, angle), true);
			RingRetriever.rightMotor.rotate(-convertAngle(RingRetriever.RIGHT_RADIUS, RingRetriever.TRACK, angle), !wait);
		} else {
			RingRetriever.leftMotor.rotate(-convertAngle(RingRetriever.LEFT_RADIUS, RingRetriever.TRACK, angle), true);
			RingRetriever.rightMotor.rotate(convertAngle(RingRetriever.RIGHT_RADIUS, RingRetriever.TRACK, angle), !wait);
		}
	}
	  
	/**
	 * This method moves the robot forwards for a specified distance.
	 * It is called in the travelTo method.
	 * @param leftMotor: a Boolean that if true signifies that the left motor should be used
	 * @param rightMotor: a Boolean that if true signifies that the right motor should be used
	 * @param forward: a Boolean that if true signifies that the robot should move forward, and if false the robot should move backwards
	 * @param wait: if the robot should wait for the rotation to terminate before doing anything else.
	 */
	void move(boolean leftMotor, boolean rightMotor, boolean forward, boolean wait, double distance, int speed) {
		// speed 
		if (leftMotor) {
			RingRetriever.leftMotor.setSpeed(speed);
		}
		if (rightMotor) {
			RingRetriever.rightMotor.setSpeed(speed);
		}
	
		// direction
		int direction = -1;
		if (forward) direction = 1;
		
		if (leftMotor && rightMotor) {
			RingRetriever.leftMotor.rotate(direction*convertDistance(RingRetriever.LEFT_RADIUS, distance), true);
			RingRetriever.rightMotor.rotate(direction*convertDistance(RingRetriever.RIGHT_RADIUS, distance), !wait);
		} else if (leftMotor) {
			RingRetriever.leftMotor.rotate(direction*convertDistance(RingRetriever.LEFT_RADIUS, distance), !wait);
		} else if (rightMotor) {
			RingRetriever.rightMotor.rotate(direction*convertDistance(RingRetriever.LEFT_RADIUS, distance), !wait);
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
		RingRetriever.leftMotor.stop(true);
		RingRetriever.rightMotor.stop(false);
	}
}

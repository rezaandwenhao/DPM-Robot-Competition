package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.main.RingRetriever;

/**
 * Class which updates the OdometerData's position by using the robot's motor's tachometer
 * @author Eliott Bourachot
 *
 */
public class Odometer implements Runnable {

	private int nowTachoL;
	private int nowTachoR;
	private int lastTachoL;
	private int lastTachoR;

	private double[] position; //[0] is X, [1] is Y, [2] is Theta

	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.It
	 * cannot be accessed externally.
	 * 
	 * @param motorL
	 * @param motorR
	 * @throws OdometerExceptions
	 */
	public Odometer() {
	  
		RingRetriever.odoData.setXYT(0, 0, 0); 	  // Reset the values of x, y and z to 0

		this.nowTachoL = 0;
		this.nowTachoR = 0;
    
		RingRetriever.leftMotor.resetTachoCount();
		RingRetriever.rightMotor.resetTachoCount();
		this.lastTachoL = RingRetriever.leftMotor.getTachoCount();
		this.lastTachoR = RingRetriever.rightMotor.getTachoCount();
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods provided from the
	 * OdometerData class to implement the odometer.
	 */
	public void run() {
		long updateStart, updateEnd;
    
		while (true) {
			updateStart = System.currentTimeMillis();

			nowTachoL = RingRetriever.leftMotor.getTachoCount();
			nowTachoR = RingRetriever.rightMotor.getTachoCount();

			// Calculate new robot position based on tachometer counts
			double distanceL = 3.14159d*RingRetriever.WHEEL_RAD*(nowTachoL-lastTachoL)/180.0d;
			double distanceR = 3.14159d*RingRetriever.WHEEL_RAD*(nowTachoR-lastTachoR)/180.0d;
      
			lastTachoL = nowTachoL;
			lastTachoR = nowTachoR;
      
			double deltaD = 0.5d*(distanceL+distanceR);
			double deltaT = (distanceL-distanceR)/RingRetriever.TRACK*57.29577;
      
			position = RingRetriever.odoData.getXYT();
			double theta = position[2]+=deltaT; // keeps the updates

			double dX = deltaD*Math.sin(theta*0.01745); //convert theta from degress to radians
			double dY = deltaD*Math.cos(theta*0.01745);
      
			// Update odometer values with new calculated values
			RingRetriever.odoData.update(dX, dY, deltaT);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < RingRetriever.ODOMETER_PERIOD) {
				try {
					Thread.sleep(RingRetriever.ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}

/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.main.RingRetriever;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Class which updates the OdometerData's position by using the robot's motor's tachometer
 * @author Eliott Bourachot
 *
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int nowTachoL;
  private int nowTachoR;
  private int lastTachoL;
  private int lastTachoR;
  private EV3LargeRegulatedMotor motorL;
  private EV3LargeRegulatedMotor motorR;

  private final double TRACK; // (cm) measured with caliper
  private final double WHEEL_RAD; // (cm) measured with caliper

  private double[] position; //[0] is X in cm, [1] is Y in cm, [2] is Theta


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param motorL
   * @param motorR
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor motorL, EV3LargeRegulatedMotor motorR,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.motorL = motorL;
    this.motorR = motorR;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.nowTachoL = 0;
    this.nowTachoR = 0;
    
    this.motorL.resetTachoCount();
    this.motorR.resetTachoCount();
    this.lastTachoL = this.motorL.getTachoCount();
    this.lastTachoR = this.motorR.getTachoCount();

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param motorL- the left motor
   * @param motorR - the right motor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor motorL,
      EV3LargeRegulatedMotor motorR, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(motorL, motorR, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    
    while (true) {
      updateStart = System.currentTimeMillis();

      nowTachoL = motorL.getTachoCount();
      nowTachoR = motorR.getTachoCount();

      // Calculate new robot position based on tachometer counts
      double distanceL = 3.14159d*WHEEL_RAD*(nowTachoL-lastTachoL)/180.0d;
      double distanceR = 3.14159d*WHEEL_RAD*(nowTachoR-lastTachoR)/180.0d;
      
      lastTachoL = nowTachoL;
      lastTachoR = nowTachoR;
      
      double deltaD = 0.5d*(distanceL+distanceR);
      double deltaT = (distanceL-distanceR)/this.TRACK*57.29577;
      
      position = getXYT();
      double theta = position[2]+=deltaT; // keeps the updates

      double dX = deltaD*Math.sin(theta*0.01745); //convert theta from degress to radians
      double dY = deltaD*Math.cos(theta*0.01745);
      
      // Update odometer values with new calculated values
      odo.update(dX, dY, deltaT);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * Corrects the odometer depending on the given starting corner.
   * Should be used directly after having localized to the nearest intersection. 
   * @param startingCorner
   */
  public void setStartingCoordinates(int startingCorner) {
    switch (startingCorner) {
    case 0:
    	odo.update(RingRetriever.TILE_SIZE, RingRetriever.TILE_SIZE, 0);
    	break;
    case 1:
    	odo.update((RingRetriever.BOARD_WIDTH-1)*RingRetriever.TILE_SIZE, RingRetriever.TILE_SIZE, -90);
    	break;
    case 2:
    	odo.update((RingRetriever.BOARD_WIDTH-1)*RingRetriever.TILE_SIZE, (RingRetriever.BOARD_HEIGHT-1)*RingRetriever.TILE_SIZE, 180);
    	break;
    case 3:
    	odo.update(RingRetriever.TILE_SIZE, (RingRetriever.BOARD_HEIGHT-1)*RingRetriever.TILE_SIZE, 90);
    	break;
    default:
    	break;
    }
  }
}

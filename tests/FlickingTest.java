/**
 * Class for testing the ring flicking algorithm and mechanism 
 * 
 * @author Wenhao Geng
 */
public class FlickingTest {

  // Motor Objects
  static final EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  static final EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  static final EV3MediumRegulatedMotor mediumMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  
  // Robot related parameters
  public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
  public static double TRACK = 9.0; // (cm) measured with caliper
 
  public static void main(String[] args) throws OdometerExceptions {
    
    while (Button.waitForAnyPress() != Button.ID_ENTER);
    
    //Lower level ring flicking
/*    mediumMotor.setSpeed(30);
    mediumMotor.rotate(105, false);
    mediumMotor.stop();
    
    motorL.setSpeed(50);
    motorR.setSpeed(50);
    motorL.rotate(convertDistance(WHEEL_RAD, 4.5), true);
    motorR.rotate(convertDistance(WHEEL_RAD, 4.5), false); //here the robot front is 18cm to the tree base
    
    mediumMotor.rotate(-105, false);
    mediumMotor.stop();
    
    motorL.rotate(-convertDistance(WHEEL_RAD, 4.5), true);
    motorR.rotate(-convertDistance(WHEEL_RAD, 4.5), false);*/ 
    
    //upper level ring flicking
    mediumMotor.setSpeed(30);
    mediumMotor.rotate(65, false);
    mediumMotor.stop();
    
    motorL.setSpeed(50);
    motorR.setSpeed(50);
    motorL.rotate(convertDistance(WHEEL_RAD, 3), true);
    motorR.rotate(convertDistance(WHEEL_RAD, 3), false); //here the robot front is 9.5cm to the tree base
    
    mediumMotor.rotate(-65, false);
    mediumMotor.stop();
    
    motorL.rotate(-convertDistance(WHEEL_RAD, 3), true);
    motorR.rotate(-convertDistance(WHEEL_RAD, 3), false);  

    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
  
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

}
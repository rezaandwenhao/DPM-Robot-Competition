public class TunnelPassing {
  
  // Parameters
  public static final double WHEEL_RAD = 2.13; // (cm) measured with caliper
  public static final double TRACK = 16.8; // (cm) measured with caliper
  public static final double LIGHT_SENSOR_X_OFFSET = 3.5;
  public static final double LIGHT_SENSOR_Y_OFFSET = 8;
  
  public static final int FORWARD_SPEED = 150;
  public static final int ROTATE_SPEED = 70;
  public static final double TILE_SIZE = 30.48;
  public static final double HALF_TILE_SIZE = 15.24;
  
  // Objects
  private static final EV3LargeRegulatedMotor leftMotor = 
          new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final EV3LargeRegulatedMotor rightMotor =
          new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final Port usPort = LocalEV3.get().getPort("S2");
  private static final Port lightLPort = LocalEV3.get().getPort("S1");
  private static final Port lightRPort = LocalEV3.get().getPort("S4");
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();


  public static void main(String[] args) throws OdometerExceptions {
    
    while (Button.waitForAnyPress() != Button.ID_ENTER);
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

    // Navigation
    Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer);
    Localization ll = new Localization(lightLMean, lightRMean, usMean, lightLData, lightRData, usData, nav, odometer, lcd);

    nav.rotate(false, 90, true);
    ll.lightCorrection();
    nav.move(true, true, false, true, RingRetriever.HALF_TILE_SIZE-LIGHT_SENSOR_Y_OFFSET, RingRetriever.FORWARD_SPEED);
    nav.rotate(true, 90, true);
    ll.lightCorrection();


    
    // move through tunnel
    nav.move(true, true, true, true, TILE_SIZE, 200);        
    nav.move(true, true, true, true, TILE_SIZE, 80);  
    nav.move(true, true, true, true, TILE_SIZE*3-LIGHT_SENSOR_Y_OFFSET, 200); 
  }

}

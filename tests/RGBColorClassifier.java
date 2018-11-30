/**
 * This class is used to figure out the RGB range for each color
 * 
 * @author Wenhao Geng
 */
public class RGBColorClassifier extends Thread {

  private static int COLOR_CORRECTION_PERIOD = 100;
  private static RGBColorClassifier cc;
  public static String detectedColor;
  public static float red, green, blue;
  
    // initializing color sensor, color sample provider, and an array to hold samples
    private static Port colorSensor = LocalEV3.get().getPort("S3");

    //for RGB     
    SensorModes mode = new EV3ColorSensor(colorSensor); // lightSensor is the instance
    SampleProvider colorSampleP = mode.getMode("RGB"); // init RGB mode
    SampleProvider colorMean = new MeanFilter(colorSampleP, 5); // use a mean filter to reduce fluctuations
    float[] sampleColor = new float[3]; // usData is the buffer in which data are returned
    
  public RGBColorClassifier(SampleProvider colorMean, float[] colorData) {
      RGBColorClassifier.detectedColor = null;
  }
  
  /**
     * This class is meant to return the existing Color Classifier Object. It is meant to be used only if an
     * Color Classifier object has been created
     * @return error if no previous Color Classifier exists
     */
  public synchronized static RGBColorClassifier getColorClassifier() throws OdometerExceptions {
  
    if (cc == null) {
      throw new OdometerExceptions("No previous Color Classifier exits.");
    }
    return cc;
  }
  
  public synchronized static RGBColorClassifier getColorClassifier(SampleProvider colorMean, float[] colorData)
        throws OdometerExceptions {
      if (cc != null) { // Return existing object
        return cc;
      } else { // create object and return it
        cc = new RGBColorClassifier(colorMean, colorData);
        return cc;
      }
    } 
   
  public void run() {
      long updateStart, updateEnd;

      while(true) {
          updateStart = System.currentTimeMillis();
           
          colorSampleP.fetchSample(sampleColor, 0);
          
         //normalize the data
         red = (float) (sampleColor[0]/(Math.sqrt(Math.pow(sampleColor[0], 2) + Math.pow(sampleColor[2], 2) + Math.pow(sampleColor[1], 2))));
         green = (float) (sampleColor[1]/(Math.sqrt(Math.pow(sampleColor[0], 2) + Math.pow(sampleColor[2], 2) + Math.pow(sampleColor[1], 2))));
         blue = (float) (sampleColor[2]/(Math.sqrt(Math.pow(sampleColor[0], 2) + Math.pow(sampleColor[2], 2) + Math.pow(sampleColor[1], 2))));
          
         if (0.070 < red && red < 0.180 && 0.490 < blue && blue < 0.640) {
           detectedColor = "blue";
         } else if (0.34 < red && red < 0.43 && 0.070 < blue && blue < 0.150) {
           detectedColor = "green";
         } else if (0.760 < red && red < 0.86 && 0.090 < blue && blue < 0.140) {
           detectedColor = "yellow";
         } else if (0.92 < red && red < 0.98 && 0.06 < blue && blue < 0.1) {
           detectedColor = "orange";
         } 
         else {
           detectedColor = "nothing";
         }
         
         
          // this ensures that the odometer only runs once every period
            updateEnd = System.currentTimeMillis();
            if (updateEnd - updateStart < COLOR_CORRECTION_PERIOD) {
              try {
                Thread.sleep(COLOR_CORRECTION_PERIOD - (updateEnd - updateStart));
              } catch (InterruptedException e) {
                // there is nothing to be done
              }
            }
      }
  }      

  public String getDetectedColor() {
      return detectedColor;
  }
}

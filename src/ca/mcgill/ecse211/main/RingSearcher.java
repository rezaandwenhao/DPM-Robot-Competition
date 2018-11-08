/**The RingSearcher class is responsible for making the robot move to the tree, circle it and locate the rings
 * 
 * @author Eden Ovadia
 * @author Eliot Bourachot
 * 
 */

package ca.mcgill.ecse211.main;
import ca.mcgill.ecse211.main.RingRetriever.Ring;

/**
 * Constructor method for RingSearcher
 * 
 *
 */
public class RingSearcher {

	public int ringsFound;
	public int ringsLeft;
	
	
	public RingSearcher() {
		
	}

	/**
	 * This method takes in the coordinates of the tree and travels to it
	 * @param x - the x coordinate of the tree
	 * @param y- the y coordinate of the tree
	 */
	public void travelToTree(int x, int y) {
		
	}
	
	/**
	 * This method will make the robot circle the tree and detects rings on it
	 * The robot will circle once for the rings on the top level and another time for the rings on the bottom level
	 * This method calls the evaluateRing method to get the value of each detected ring
	 */
	public void findRings() {
		
	}
	/**
	 * This method takes in an object of type ring as an argument and returns its value according to a point system provided
	 * @param ring- the ring detected in the findRings method
	 * @return an int- which is the value of the ring 
	 */
	public int evaluateRing(Ring ring) {
		return 0;
	}
}

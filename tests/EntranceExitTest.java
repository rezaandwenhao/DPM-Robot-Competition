public class EntranceExitTest {

	public static final int BOARD_WIDTH = 13;
	public static final int BOARD_HEIGHT = 9;
	
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
	public static double[] tunnelEntranceCoordinates;
	public static double[] tunnelExitCoordinates;
	
	public static void main(String args[]) {
		System.out.println("Test SC=0, Right");
		startingCorner = 0;
		tunnelLLx = 5; tunnelLLy = 2; tunnelURx = 7; tunnelURy = 3;
		zoneLLx = 0; zoneLLy = 0; zoneURx = 5; zoneURy = 4;
		
		double[] entrance = getEntrance();
		double[] exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=1 Right");
		startingCorner = 1;
		tunnelLLx = 11; tunnelLLy = 4; tunnelURx = 12; tunnelURy = 6;
		zoneLLx = 7; zoneLLy = 0; zoneURx = 13; zoneURy = 4;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=2 Right");
		startingCorner = 2;
		tunnelLLx = 7; tunnelLLy = 7; tunnelURx = 8; tunnelURy = 8;
		zoneLLx = 8; zoneLLy = 5; zoneURx = 13; zoneURy = 9;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=3 Right");
		startingCorner = 3;
		tunnelLLx = 2; tunnelLLy = 4; tunnelURx = 3; tunnelURy = 6;
		zoneLLx = 0; zoneLLy = 6; zoneURx = 7; zoneURy = 9;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=0 Left");
		startingCorner = 0;
		tunnelLLx = 2; tunnelLLy = 4; tunnelURx = 3; tunnelURy = 6;
		zoneLLx = 0; zoneLLy = 4; zoneURx = 5; zoneURy = 4;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=1 Left");
		startingCorner = 1;
		tunnelLLx = 5; tunnelLLy = 2; tunnelURx = 7; tunnelURy = 3;
		zoneLLx = 7; zoneLLy = 0; zoneURx = 13; zoneURy = 4;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=2 Left");
		startingCorner = 2;
		tunnelLLx = 11; tunnelLLy = 4; tunnelURx = 12; tunnelURy = 6;
		zoneLLx = 8; zoneLLy = 5; zoneURx = 13; zoneURy = 9;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");
		
		System.out.println("Test SC=3 Left");
		startingCorner = 3;
		tunnelLLx = 7; tunnelLLy = 7; tunnelURx = 8; tunnelURy = 8;
		zoneLLx = 0; zoneLLy = 6; zoneURx = 7; zoneURy = 9;
		
		entrance = getEntrance();
		exit = getExit();
		
		System.out.println("Entrance: ("+entrance[0]+","+entrance[1]+")");
		System.out.println("Exit: ("+exit[0]+","+exit[1]+")");

	}
	
	public static double[] getEntrance() {
		double[] coordinates = {0,0};
		
		boolean rightSide = isTunnelAtRelativeRight(); // true if tunnel is determined to be on the right side of our zone
		
		switch (startingCorner) {
		case 0:
			if (rightSide) {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
			} else {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
			}
			break;
		case 1:
			if (rightSide) {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
			} else {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
			}
			break;
		case 2:
			if (rightSide) {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
			} else {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
			}
			break;
		case 3:
			if (rightSide) {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
			} else {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
			}
			break;
		}
		return coordinates;
	}
	
	public static double[] getExit() {
		double[] coordinates = {0,0};
	
		boolean rightSide = isTunnelAtRelativeRight(); // true if tunnel is determined to be on the right (relative) side of our zone
		
		switch (startingCorner) {
		case 0:
			if (rightSide) {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
			} else {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
			}
			break;
		case 1:
			if (rightSide) {
				coordinates[0] = tunnelURx-0.5;
				coordinates[1] = tunnelURy+1.5;
			} else {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
			}
			break;
		case 2:
			if (rightSide) {
				coordinates[0] = tunnelLLx-1.5;
				coordinates[1] = tunnelLLy+0.5;
			} else {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
			}
			break;
		case 3:
			if (rightSide) {
				coordinates[0] = tunnelLLx+0.5;
				coordinates[1] = tunnelLLy-1.5;
			} else {
				coordinates[0] = tunnelURx+1.5;
				coordinates[1] = tunnelURy-0.5;
			}
			break;
		}
		return coordinates;
	}
	
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
	
	private static int getMaxTunnelLength() {
		int x_len = Math.abs(tunnelURx - tunnelLLx);
		int y_len = Math.abs(tunnelURy - tunnelURy);
		return Math.max(x_len, y_len);
	}
	
}

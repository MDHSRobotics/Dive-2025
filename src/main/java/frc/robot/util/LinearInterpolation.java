package frc.robot.util;

public class LinearInterpolation {
    // Method to perform linear interpolation
    public static double interpolate(double x, double[] xTable, double[] yTable) {
        // If x is out of range, return the closest value
        if (x <= xTable[0]) return yTable[0];
        if (x >= xTable[xTable.length - 1]) return yTable[yTable.length - 1];

        // Find the interval that contains x
        for (int i = 0; i < xTable.length - 1; i++) {
            if (x >= xTable[i] && x <= xTable[i + 1]) {
                // Apply linear interpolation formula
                double x1 = xTable[i];
                double x2 = xTable[i + 1];
                double y1 = yTable[i];
                double y2 = yTable[i + 1];

                // Linear interpolation
                return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
            }
        }

        // Should never reach here if input is within range
        throw new IllegalArgumentException("x value out of range.");
    }
}

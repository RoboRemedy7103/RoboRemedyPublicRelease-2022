// RobotMath.java - common math functions
package frc.robot;

public class RobotMath {
    public static double angleCenteredOnTarget(double angle, double goalAngle) {
        while (angle < (goalAngle - 180)) {
            angle += 360;
        }
        while (angle > (goalAngle + 180)) {
            angle -= 360;
        }

        return angle;
    }

    // Round a double to 1 decimal place
    static double round1(double valueToRound) {
        return Math.round(valueToRound * 10.0) / 10.0;
    }

    // Round a double to 2 decimal places
    public static double round2(double valueToRound) {
        return Math.round(valueToRound * 100.00) / 100.00;
    }
}
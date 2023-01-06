// OI.java - Operator Interface. Reads values from joysticks and buttons
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class OI {

    OI() {
    }

    public Joystick driver = new Joystick(0);

    double getDriveMagnitude() {
        return driver.getMagnitude();
    }

    double getDriveDirectionDegrees() {
        return driver.getDirectionDegrees();
    }

    double getFacingJoystickMagnitude() {
        return Math.sqrt(Math.pow(driver.getRawAxis(2), 2) + Math.pow(driver.getRawAxis(3), 2));
    }

    double getFacingJoystickDegrees() {
        double x = driver.getRawAxis(2);
        double y = driver.getRawAxis(3);
        double angleRad = Math.atan2(y, x);
        double angleDeg = Math.toDegrees(angleRad);
        double finalAng = angleDeg + 90;
        return finalAng;
    }

    public boolean isJoystickButtonPressed(int stick, int button) {
        if (DriverStation.isJoystickConnected(stick)) {
            return DriverStation.getStickButton(stick, button);
        } else {
            return false;
        }
    }
    
    boolean getLockButtonPressed() {
        return isJoystickButtonPressed(0, 1);
    }

    boolean getDriveSlowButtonPressed() {
        return isJoystickButtonPressed(0, 6);
    }

    boolean getDriveFastButtonPressed() {
        return driver.getRawButton(8);
    }

    boolean getAllignWheelsButtonPressed() {
        return driver.getRawButton(2);
    }

    boolean robotOrientedDriving() {
        return driver.getRawButton(5);
    }

    boolean getDriverResetGyroButtonPressed() {
        return driver.getRawButton(4);
    }

    public boolean getIntakeButtonPressed() {
        return driver.getRawButton(3);
    }

    public double getRotationSpeed() {
        return driver.getRawAxis(2) * 270;
    }

    public boolean getIndexerButtonPressed() {
        return driver.getRawButton(7);
    }

    public boolean getRiseHookButtonPressed() {
        return driver.getRawButton(10);
    }

    public boolean getLowerHookButtonPressed() {
        return driver.getRawButton(9);
    }

    public boolean getOperatorIntakePressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButton(1, 8);
        } else {
            return false;
        }
    }

    public boolean getOperatorUnjambPressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButton(1, 6);
        } else {
            return false;
        }
    }

    public boolean getOperatorAutoShootPressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButton(1, 7);
        } else {
            return false;
        }
    }

    public boolean getOperatorAutoIndexMissPressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButton(1, 5);
        } else {
            return false;
        }
    }

    public double getOperatorMagnitude() {
        if (DriverStation.isJoystickConnected(1)) {
            return Math.sqrt(
                    Math.pow(DriverStation.getStickAxis(1, 0), 2) + Math.pow(DriverStation.getStickAxis(1, 1), 2));
        } else {
            return 0.0;
        }
    }

    public double getOperatorDirectionDegrees() {
        if (DriverStation.isJoystickConnected(1)) {
            return Math.toDegrees(Math.atan2(DriverStation.getStickAxis(1, 0), -DriverStation.getStickAxis(1, 1)));
        } else {
            return 0.0;
        }
    }

    public boolean getOperatorManualIndexForwardPressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButton(1, 1);
        } else {
            return false;
        }
    }

    public boolean getOperatorManualIndexBackwardPressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButton(1, 2);
        } else {
            return false;
        }
    }

    public boolean getOperatorResetGyroButtonPressed() {
        if (DriverStation.isJoystickConnected(1)) {
            return DriverStation.getStickButtonPressed(1, 4);
        } else {
            return false;
        }
    }

    public boolean getOperatorNoVisionPressed() {
        if(DriverStation.isJoystickConnected(1)){
            return DriverStation.getStickButton(1, 3);
        } else {
            return false;
        }
    }

    public boolean getOperatorDistanceAdjustmentPlus() {
        if(DriverStation.isJoystickConnected(1)){
            return DriverStation.getStickButtonPressed(1, 9);
        } else {
            return false;
        }
    }

    public boolean getOperatorDistenceAdjustmentMinus() {
        if(DriverStation.isJoystickConnected(1)){
            return DriverStation.getStickButtonPressed(1, 10);
        } else {
            return false;
        }
    }

    
}
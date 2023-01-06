// TeleopRobot.java - Code for teleop mode
package frc.robot;

public class TeleopRobot {
    Electronics e;
    RoboLog rLog;
    Action act;
    OI oi;
    RobotState robotState;
    Dashboard dash;

    private double facing = 0;
    boolean lastAlignWheels = false;
    boolean lastLockWheels = false;
    boolean lastOpeResetGyro = false;
    boolean lastDriveResetGyro = false;
    boolean lastRamping = false;
    double lastDriveSpeed = 0.0;
    double lastDriveDirection = 0.0;

    public TeleopRobot(Electronics e, RoboLog rLog, Action act, OI oi,
        RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;
    }

    public void teleopInit() {
        e.resetMaxCurrents();
        act.actionReset();
        lastDriveSpeed = 0.0;
        lastDriveDirection = 0.0;
    }

    
    private void logButtonChanges() {
        boolean alignWheels = oi.getAllignWheelsButtonPressed();
        boolean lockWheels = oi.getLockButtonPressed();
        boolean opeResetGyro = oi.getOperatorResetGyroButtonPressed();
        boolean driverResetGyro = oi.getDriverResetGyroButtonPressed();

        if(alignWheels != lastAlignWheels) {
            if(alignWheels)
                rLog.print("Align Wheels Button pressed");
            else
                rLog.print("Align Wheels Button released");
        }

        if(lockWheels != lastLockWheels) {
            if(lockWheels)
                rLog.print("Lock Wheels Button pressed");
            else
                rLog.print("Lock Wheels Button released");
        }

        if(opeResetGyro != lastOpeResetGyro) {
            if (opeResetGyro)
                rLog.print("Operater reset Gyro button presed");
            else
                rLog.print("Operater reset Gyro button released");
        }

        if(driverResetGyro != lastDriveResetGyro) {
            if (driverResetGyro)
            rLog.print("Driver reset Gyro button presed");
        else
            rLog.print("Driver reset Gyro button released");
        }

        lastAlignWheels = alignWheels;
        lastLockWheels = lockWheels;
        lastOpeResetGyro = opeResetGyro;
        lastDriveResetGyro = driverResetGyro;
    }

    public void teleopPeriodic() {
        e.checkMaxCurrents();
        logButtonChanges();
        double joyMag = Math.pow(oi.getDriveMagnitude(), 2);
        double driveSpeed = 0.0;
        double driveDirection = lastDriveDirection;
        boolean isStopped;
        boolean isRamping = false;
        boolean isDemoMode = dash.isDemoMode();

        // driving
        if (oi.getDriveFastButtonPressed() || oi.getDriveSlowButtonPressed()) { // 6 & 8

            if (joyMag < 0.2) {
                driveSpeed = 0;
            } else {
                double maxSpeed = (oi.getDriveSlowButtonPressed() ? 60 : e.getMaxTeleopInchesPerSecond()); //was 37
                if (isDemoMode) {
                    maxSpeed = (oi.getDriveSlowButtonPressed() ? 20 : 35);
                }
                driveSpeed = (joyMag - 0.2) * (maxSpeed / 0.8);
            }

            driveDirection = oi.getDriveDirectionDegrees();
            double facingMagnitude = oi.getFacingJoystickMagnitude();
            double facingAngle = oi.getFacingJoystickDegrees();

            if (facingMagnitude > 0.5) {
                facing = facingAngle;
            }

            isStopped = false;
        } else {
            facing = e.getGyro();
            driveSpeed = 0.0;
            driveDirection = lastDriveDirection;
            isStopped = true;
        }

        // Ramp down to prevent tipping robot
        if (lastDriveSpeed > 50) {
            double angleDiff = Math.abs(RobotMath.angleCenteredOnTarget(driveDirection, lastDriveDirection) - lastDriveDirection);
            if ((driveSpeed < lastDriveSpeed - 7.5) || (angleDiff > 35.0)) {
                isRamping = true;
                driveSpeed = lastDriveSpeed - 7.5;
                driveDirection = lastDriveDirection;
                isStopped = false;
            }
        }

        lastDriveSpeed = driveSpeed;
        lastDriveDirection = driveDirection;
        lastRamping = isRamping;

        if (!isStopped) {
            e.assignRobotMotionAndHeadingField(driveDirection, driveSpeed, facing);
        } else if (oi.getAllignWheelsButtonPressed()) {
            e.alignSwerveMotorsForward();
        } else if (oi.getLockButtonPressed()) {
            e.lockWheels();
        } else {
            e.stopSwerveMotors();
        }

    }
}
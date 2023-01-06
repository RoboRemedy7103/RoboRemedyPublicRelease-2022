// Electronics.java - control the motors, gyro, encoders, solenoids, etc.
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.File;

import frc.robot.RobotMotor.RobotEncoderType;
import frc.robot.RobotMotor.RobotMotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class Electronics {
    public enum RobotName {
        LIVE2023, JerryJr, Invisibot
    }

    public static final double DISTANCE_SENSOR_OFFSET = 5.0;

    private RobotName robotName;

    private RoboLog rLog;
    RobotState robotState;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;
    private SwerveModule backLeft;
    private SwerveModule[] swerveDrives;
    private SwerveState currentState;
    private SwerveState centerPivotState;
    public RobotGyro gyro = new RobotGyro();
    private double lastTravelVelocity = 0;
    private Timer commandTimer = new Timer();
    private Timer batteryTimer = new Timer();
    private double batteryVoltage = 0;
    private double lastCommandTime = 0;
    private double maxTeleopInchesPerSecond = 0;
    TimeOfFlight distSensor;
    private double[] maxCurrent = new double[16];
    private PowerDistribution pdp;

    private double lastTravelAngleDegrees = 0.0;
    private double lastTravelRotationDegreesPerSecond = 0.0;
    private boolean lastYellowButtonsValue = false;

    //Prox Sensor
    private DigitalInput proxSensor1 = new DigitalInput(0);

    //Button
    private DigitalInput yellowButton1;
    private DigitalInput yellowButton2;
    private DigitalInput testLEDButton;

    //LED
    private DigitalOutput gyroLed = new DigitalOutput(6);

    Electronics(boolean fullSwerve, RoboLog rLog, RobotState robotState) {
        File jr = new File("/home/lvuser/jr.txt");
        File nexu = new File("/home/lvuser/invisi.txt");
        this.robotState = robotState;

        if (jr.isFile()) {
            rLog.print("Robot = Jerry Junior");
            robotName = RobotName.JerryJr;
        } else if (nexu.isFile()) {
            rLog.print("Robot = Invisibot");
            robotName = RobotName.Invisibot;
        } else {
            rLog.print("Robot = Live 2023");
            robotName = RobotName.LIVE2023;
        }

        this.rLog = rLog;
        if (robotName == RobotName.JerryJr) {
            centerPivotState = new SwerveState(13.5, 13.5);

            currentState = centerPivotState;

            frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.Falcon, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);

            // frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
            //         RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            // frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
            //         RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            // backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
            //         RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            // backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
            //         RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);

            maxTeleopInchesPerSecond = 77.0;

            pdp = null;
            distSensor = null;
            yellowButton1 = null;
            yellowButton2 = null;
            testLEDButton = null;
        } else if (robotName == RobotName.Invisibot) {
            centerPivotState = new SwerveState(17.5, 17.5);

            currentState = centerPivotState;

            // Swerves
            frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);

            maxTeleopInchesPerSecond = 120.0;

            pdp = new PowerDistribution();
            distSensor = new TimeOfFlight(1);
            yellowButton1 = new DigitalInput(3);
            yellowButton2 = new DigitalInput(4);
            testLEDButton = new DigitalInput(5);
        } else {
            centerPivotState = new SwerveState(17.5, 17.5);

            currentState = centerPivotState;

            //Swerves
            frontLeft = new SwerveModule(41, 51, 01, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.Falcon, RobotMotorType.Falcon,
                    RobotEncoderType.Cancoder, 3.77, 6.12, 28.14);

            maxTeleopInchesPerSecond = 120.0;

            pdp = new PowerDistribution();
            distSensor = new TimeOfFlight(2);
            yellowButton1 = new DigitalInput(3);
            yellowButton2 = new DigitalInput(4);
            testLEDButton = new DigitalInput(5);
        }

        if (fullSwerve) {
            swerveDrives = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };
        } else {
            swerveDrives = new SwerveModule[] { backLeft };
        }

        commandTimer.start();
        if (distSensor != null)
            distSensor.setRangingMode(RangingMode.Short, 24);

        if(pdp != null){
            batteryVoltage = pdp.getVoltage();
        }

        batteryTimer.start();
    }

    public void stopSwerveMotors() {
        lastTravelVelocity = 0;
        lastTravelRotationDegreesPerSecond = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDriveSpeed(0);
            m.setTurnPower(0);
            if (Math.abs(m.getDriveVelocity()) <= 0.25) {
                // If we are trying to stop the motor and the motor
                // is still not moving, then use Coast Mode
                m.setDriveAndTurnNeutralMode(false);
            } else {
                // If we are trying to stop the motor and the motor
                // is still spinning, then use Break Mode
                m.setDriveAndTurnNeutralMode(false);
            }
        }
    }

    public void alignSwerveMotorsForward() {
        lastTravelVelocity = 0;
        lastTravelAngleDegrees = 0;
        lastTravelRotationDegreesPerSecond = 0;
        lastCommandTime = commandTimer.get();
        for (SwerveModule m : swerveDrives) {
            m.setDrivePercent(0);
            m.setTurnHeading(0);
        }
    }

    public void stopAllMotors() { // Stops other motors, too
        stopSwerveMotors();
    }

    private SwerveModule getModule(int module) {
        if (module >= swerveDrives.length || module < 0) {
            return null;
        } else {
            return swerveDrives[module];
        }
    }

    public void setDrivePercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDrivePercent(power);
        }
    }

    public void setDriveSpeed(int moduleNumber, double velocity) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDriveSpeed(velocity);
        }
    }

    public RobotMotor getSwerveDriveMotor(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return null;
        } else {
            return swerveDriveModule.getDriveMotor();
        }
    }

    public void setTurnPercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnPower(power);
        }
    }

    public void setTurnHeading(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnHeading(angle);
        }
    }

    public void setAllHeadings(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnHeading(i, angle);
        }
    }

    public void setTurnEncoder(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnEncoderValue(angle);
        }
    }

    public void assignAllEncoderValues() {
        for (int i = 0; i < 4; i++) {
            setTurnEncoder(i, getAbsoluteTurnEncoderPosition(i));
        }
        rLog.print("Swerve Drive Turn Encoders Reset");
    }

    public void setAbsoluteEncoderValue(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setAbsoluteTurnEncoderPosition(angle);
        }
    }

    public void zeroAllAbsoluteEncoderValues(){
        for (int i = 0; i < 4; i++) {
            setAbsoluteEncoderValue(i, 0);
        }
    }

    public void setAllTurnEncoders(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnEncoder(i, angle);
        }
    }

    public double[] getAllTurnEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getTurnEncoderPosition();
        }
        return positions;
    }

    public double[] getAllDriveEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getDriveEncoderPosition();
        }
        return positions;
    }

    public double getDriveVelocity(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveVelocity();
        }
    }

    public boolean areWheelsStopped() {
        for (SwerveModule m : swerveDrives) {
            if (Math.abs(m.getDriveVelocity()) > 0.5) {
                return false;
            }
        }
        return true;
    }

    public double getAbsoluteTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getAbsoluteTurnEncoderPosition();
        }
    }

    public double getTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getTurnEncoderPosition();
        }
    }

    public void assignRobotMotionField(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastTravelAngleDegrees = travelAngle;
        lastTravelRotationDegreesPerSecond = degreesPerSecond;
        lastCommandTime = commandTimer.get();
        currentState.assignSwerveModulesField(travelAngle, travelInchesPerSecond, degreesPerSecond, getGyro(),
                swerveDrives[0].getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(currentState.getMagnitude(i), currentState.getAngle(i));
        }
    }

    public void assignRobotMotionRobot(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastTravelAngleDegrees = travelAngle + gyro.getAngle();
        lastTravelRotationDegreesPerSecond = degreesPerSecond;
        lastCommandTime = commandTimer.get();
        currentState.assignSwerveModules(travelAngle, travelInchesPerSecond, degreesPerSecond, swerveDrives[0].getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(currentState.getMagnitude(i), currentState.getAngle(i));
        }
    }

    public void assignRobotMotionAndHeadingField(double travelAngle, double travelInchesPerSecond, double facingAngle) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 1.0), 0.6);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 6.0;
            if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-225, calcDPS), -10);
            } else {
                degreesPerSecond = Math.max(Math.min(225, calcDPS), 10);
            }
        }
        assignRobotMotionField(travelAngle, travelInchesPerSecond, degreesPerSecond);
    }

    public void assignRobotMotionAndHeadingFieldAccel(double travelAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 2.5), 1);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 2.0;
            if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-120, calcDPS), -25);
            } else {
                degreesPerSecond = Math.max(Math.min(120, calcDPS), 25);
            }
        }
        assignRobotMotionField(travelAngle, limitAccel(travelInchesPerSecond, maxTravelAcceleration), degreesPerSecond);
    }

    public void lockWheels() {
        lastTravelVelocity = 0;
        lastTravelRotationDegreesPerSecond = 0;
        lastCommandTime = commandTimer.get();
        currentState.lockWheels();
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setDrivePercent(currentState.getMagnitude(i));
            swerveDrives[i].setTurnHeading(currentState.getAngle(i));
        }
    }

    // swerveModuleNumber: 0 - frontleft; 1 - frontright; 2 - backright; 3 - backleft
    // Type: 0 - Not Applicable; 1 - Turn; 2 - Drive
    // OtherMotorID: 0 - Not Applicable; Use motor IDs
    public boolean isMotorAttached(boolean isSwerve, int swerveModuleNumber, int swerveType, int notSwerveMotorID) {
        if (isSwerve == true) {
            SwerveModule swerveDriveModule = getModule(swerveModuleNumber);
            if (swerveDriveModule == null) {
                rLog.print("Not Valid Swerve Module Selected, Module: " + swerveModuleNumber);
                return false;
            } else {
                if (swerveType == 1)
                    return swerveDriveModule.isTurnAttached();
                else if (swerveType == 2)
                    return swerveDriveModule.isDriveAttached();
                else
                    return false;
            }
        } else if (isSwerve == false) {
            return false;
        } else {
            return false;
        }
    }
        
    public double getLastTravelVelocityCommand() {
        return lastTravelVelocity;
    }

    public double getLastTravelAngleDegrees() {
        return lastTravelAngleDegrees;
    }

    public double getLastTravelRotationDegreesPerSecond() {
        return lastTravelRotationDegreesPerSecond;
    }

    public double getCalculatedTravelSinceLastCommand() {
        return lastTravelVelocity * (commandTimer.get() - lastCommandTime);
    }

    public double getMaxSpeedChange(double maxTravelAcceleration) {
        return maxTravelAcceleration * (commandTimer.get() - lastCommandTime);
    }

    public double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        if (goalInPerSec >= getLastTravelVelocityCommand()) {
            calcInPerSec = Math.min(goalInPerSec,
                    getLastTravelVelocityCommand() + getMaxSpeedChange(maxTravelAcceleration));
        } else {
            calcInPerSec = Math.max(goalInPerSec,
                    getLastTravelVelocityCommand() - getMaxSpeedChange(maxTravelAcceleration));
        }
        return calcInPerSec;
    }

    public double getMaxTeleopInchesPerSecond() {
        return maxTeleopInchesPerSecond;
    }

    public void setSwerveCenterPoint(double x, double y) {
        currentState = new SwerveState(centerPivotState, x, y);
    }

    public void setSwerveCoast() {
        for (SwerveModule m : swerveDrives) {
            m.setDriveAndTurnNeutralMode(true);
        }
    }

    public void setSwerveBrake() {
        for (SwerveModule m : swerveDrives) {
            m.setDriveAndTurnNeutralMode(false);
        }
    }

    public double getGyro() {
        if (gyro != null)
            return gyro.getAngle();
        else
            return 0.0;
    }

    public double getGyroCenteredOnGoal(double goalAngle) {
        double gyroValue = getGyro();

        return RobotMath.angleCenteredOnTarget(gyroValue, goalAngle);
    }

    public void setGyro(double gyroAngle) {
        if (gyro != null)
            gyro.setYaw(gyroAngle);
    }

    public void resetGyro() {
        setGyro(0);
    }

    public boolean isBatteryGood() {
        if (batteryVoltage >= 12.4) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isDistanceSensorRangeValid() {
        if (distSensor != null)
            return distSensor.isRangeValid();
        else
            return false;
    }

    public double getDistanceInInches() {
        double distance;
        if (distSensor != null)
        {
            rLog.print("Good sensor");
            distance = distSensor.getRange();
        }
        else
        {
          rLog.print("Null sensor");
            distance = 0;
        }
        distance = distance * 0.0393701; // Convert mm to inches
        double robotCenterDistance = distance + DISTANCE_SENSOR_OFFSET;
        return robotCenterDistance;
    }

    public boolean getProxSensor() {
        return proxSensor1.get();
    }

    public boolean getYellowButtonValue() {
        if (yellowButton1 == null || yellowButton2 == null) {
            return false;
        }
        boolean bothPressed = yellowButton1.get() && yellowButton2.get();
        boolean yellowButtonValue = bothPressed && !lastYellowButtonsValue;

        lastYellowButtonsValue = bothPressed;

        return yellowButtonValue;
    }

    public boolean isPhyscalResetGyroButtonPressed() {
        if (DriverStation.isDisabled()) {
            return getYellowButtonValue();
        } else {
            return false;
        }
    }

    public boolean isTestLEDButtonPressed() {
        if (testLEDButton == null || yellowButton1 == null) {
            return false;
        }

        return (!testLEDButton.get() && !yellowButton1.get());
    }

    public String getRobotName() {
        String name;
        if (robotName == RobotName.JerryJr) {
            name = "Jerry Jr";
        } else if (robotName == RobotName.Invisibot) {
            name = "Invisibot";
        } else {
            name = "Live 2023";
        }
        return name;
    }

    public void resetMaxCurrents() {
        for (int i = 0; i < 16; i++) {
            maxCurrent[i] = 0;
        }
    }

    public void checkMaxCurrents() {
        // if (pdp != null) {
        //     for (int i = 0; i < 16; i++) {
        //         maxCurrent[i] = Math.max(maxCurrent[i], pdp.getCurrent(i));
        //     }
        // }
    }

    public boolean isRedAlliance(){
        if (DriverStation.getAlliance() == Alliance.Red){
            return true;
        } else
            return false;
    }

    public void reportMaxCurrents() {
        boolean foundOne = false;
        if (pdp != null) {
            for (int i = 0; i < 16; i++){
                if (maxCurrent[i] > 0)
                    foundOne = true;
            }
            if (foundOne) {
                String logMsg = "Max Currents:";
                for (int i = 0; i < 16; i++){
                    logMsg += " " + (RobotMath.round1(maxCurrent[i]));
                }
                rLog.print(logMsg);
            }
        }
    }

    public void ledDisplay(boolean isOn){
        gyroLed.set(isOn);
    }

    public void robotPeriodic(){
        if(batteryTimer.get() >= 10 && pdp != null){
            batteryVoltage = pdp.getVoltage();
            batteryTimer.reset();
            batteryTimer.start();    
        }
    }
}
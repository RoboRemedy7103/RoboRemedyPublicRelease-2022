// SwerveModule.java - used to control one corner of a swerve drive (one drive and one turn motor)
package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.RobotMotor.*;

public class SwerveModule {
    private RobotMotor driveMotor;
    private RobotMotor turnMotor;
    private PWMInput pWMInput = null;
    private CANCoder turnEncoder = null;
    private RoboLog rLog;
    private double lastTurnDifference;
    private RobotEncoderType turnEncoderType;
    private double offsetDegrees;
    private LinearMapper velocityMapper = new LinearMapper();

    SwerveModule(int driveID, int turnID,int absoluteEncoderID, double offsetDegrees, RoboLog rLog,
            RobotMotor.RobotMotorType driveMotorType, RobotMotor.RobotMotorType turnMotorType,
            RobotEncoderType turnEncoderType, double driveWheelDiameter, double driveGearRatio, double turnEncoderRatio) {
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: rLog is null in Action");
        }
        
        if (driveMotorType == RobotMotorType.SparkMax) {
            velocityMapper.add(3.6, 0.03);
            velocityMapper.add(9.45, 0.07);
            velocityMapper.add(16.6, 0.12);
            velocityMapper.add(28.2, 0.20);
            velocityMapper.add(56.1, 0.40);
            velocityMapper.add(70.1, 0.50);
            velocityMapper.add(104.4, 0.75);
            velocityMapper.add(139.0, 1.00);
        } else {
            // nexu
            // mapper.add(1.5, 0.02);
            // mapper.add(12.5, 0.067);
            // mapper.add(23.2, 0.119);
            // mapper.add(38.4, 0.198);
            // mapper.add(76.9, 0.399);
            // mapper.add(95.9, 0.499);
            // mapper.add(143.8, 0.748);
            // mapper.add(191.0, 1.0);

            //2022 live
            velocityMapper.add(1.8, 0.02);
            velocityMapper.add(14.5, 0.067);
            velocityMapper.add(25.5, 0.119);
            velocityMapper.add(42.7, 0.198);
            velocityMapper.add(83.5, 0.399);
            velocityMapper.add(104.0, 0.499);
            velocityMapper.add(157.0, 0.748);
            velocityMapper.add(207.0, 1.0);
        }

        driveMotor = new RobotMotor(driveMotorType, driveID, false, true, rLog, (driveWheelDiameter * Math.PI) / driveGearRatio, 11,
                RobotEncoderType.Internal, 0);
        driveMotor.setPID(0, 0, 0, 0, false);
        driveMotor.setMaxOutput(1.0);
        driveMotor.setPercentVelocityLinearMapper(velocityMapper);
        driveMotor.burnFlash();

        turnMotor = new RobotMotor(turnMotorType, turnID, true, true, rLog, turnEncoderRatio, 11,
            RobotEncoderType.Internal, 0);
        turnMotor.setPID(0.04, 0.0001, 0, 3.0, true);
        turnMotor.setMaxOutput(0.8);
        turnMotor.burnFlash();

        this.turnEncoderType = turnEncoderType;
        if (turnEncoderType == RobotEncoderType.Cancoder) {
            turnEncoder = new CANCoder(absoluteEncoderID);
            turnEncoder.configSensorDirection(true);
        } else {
            pWMInput = new PWMInput(absoluteEncoderID, 1.0E-6, 1.0E-6, 1024);
        }

        this.offsetDegrees = offsetDegrees;
    }

    public boolean isDriveAttached() {
        return driveMotor.isAttached();
    }

    public boolean isTurnAttached() {
        return turnMotor.isAttached();
    }

    public void setDrivePercent(double percent) {
        driveMotor.setPercent(percent);
    }

    public void setTurnPower(double percent) {
        turnMotor.setPercent(percent);
    }

    public void resetDriveEncoderValue() {
        driveMotor.setEncoderValue(0);
    }

    public void setDriveEncoderValue(double position) {
        driveMotor.setEncoderValue(position);
    }

    public void setTurnEncoderValue(double position) {
        turnMotor.setEncoderValue(position);
    }

    public double getTurnEncoderPosition() {
        return turnMotor.getEncoderPosition();
    }

    public double getDriveEncoderPosition() {
        return driveMotor.getEncoderPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoderVelocity();
    }

    public void setAbsoluteTurnEncoderPosition(double position){
        if (turnEncoderType == RobotEncoderType.Cancoder) {
            double magnetOffset = turnEncoder.configGetMagnetOffset();
            double currentEncoderValue = turnEncoder.getAbsolutePosition();
            turnEncoder.configMagnetOffset(magnetOffset - position + currentEncoderValue);
        }
    }

    public double getAbsoluteTurnEncoderPosition() {
        if (turnEncoderType == RobotEncoderType.Cancoder) {
            return (turnEncoder.getAbsolutePosition() - offsetDegrees);
        } else {
            return ((pWMInput.getLastPulse() / 1024.0) * 360.0) - offsetDegrees;
        }
    }

    public double getMaxVelocity() {
        return velocityMapper.getMaxInputValue();
    }

    public RobotMotor getDriveMotor() {
        return driveMotor;
    }

    /*
     * 
     * Methods to use when we are using feeback
     * 
     */

    public void setDriveSpeed(double velocity) {
        driveMotor.setVelocity(velocity);
    }

    public void setTurnHeading(double angle) {
        double currentHeading = getTurnEncoderPosition();
        while (angle < (currentHeading - 180)) {
            angle = angle + 360;
        }
        while (angle > (currentHeading + 180)) {
            angle = angle - 360;
        }
        double difference = (angle - currentHeading);
        if (((difference <= 0 && lastTurnDifference >= 0) || (difference >= 0 && lastTurnDifference <= 0))
                && Math.abs(difference) < 0.55) {
            setTurnPower(0);
        } else {
            lastTurnDifference = difference;
            turnMotor.setPosition(angle);
        }
    }

    public void setSpeedAndHeading(double velocity, double angle) {
        if (velocity < 0.01) {
            setDriveSpeed(0);
            setTurnPower(0);
        } else {
            double currentHeading = getTurnEncoderPosition();
            double currentVelocity = getDriveVelocity();
            while (angle < (currentHeading - 180)) {
                angle = angle + 360;
            }
            while (angle > (currentHeading + 180)) {
                angle = angle - 360;
            }
            double angleDifference = Math.abs(angle - currentHeading);
            if (angleDifference > 90.0 && currentVelocity < 20.0) {
                angle = angle + 180;
                velocity = -velocity;
                angleDifference = 180.0 - angleDifference;
            }
            if (angleDifference > 10.0 && currentVelocity < 20.0) {
                setDriveSpeed(0);
            } else {
                setDriveSpeed(velocity);
            }
            setTurnHeading(angle);
        }
    }

    /*
     * 
     * Methods to use when tuning new modules
     * 
     */

    public void setDriveFPID(double F, double P, double I, double D, double iZone, boolean isVelocity) {
        driveMotor.setPID(P, I, D, iZone, isVelocity);
    }

    public void setTurnFPID(double F, double P, double I, double D, double iZone, boolean isVelocity) {
        turnMotor.setPID(P, I, D, iZone, isVelocity);
    }

    public void setDriveOpenLoopRamp(double rate) {
        driveMotor.setDriveOpenLoopRamp(rate);
    }

    public void setTurnOpenLoopRamp(double rate) {
        turnMotor.setDriveOpenLoopRamp(rate);
    }

    public void setDriveAndTurnNeutralMode(boolean isCoast) {
        driveMotor.setNeutralMode(isCoast);
        turnMotor.setNeutralMode(isCoast);
    }
}
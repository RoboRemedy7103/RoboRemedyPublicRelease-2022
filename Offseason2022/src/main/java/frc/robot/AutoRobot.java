// AutoRobot.java - Code for autonomous programs
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class AutoRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private Dashboard dash;
    private RobotState robotState;

    private int stepNumber;
    private Timer timer1 = new Timer();
    private String autoSelection;

    public AutoRobot(Electronics e, RoboLog rLog, Action act, Dashboard d, RobotState robotState) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.dash = d;
        this.robotState = robotState;

        if (this.rLog == null) {
            System.out.println("Warning: AutoRobot.rLog is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: AutoRobot.dash is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: AutoRobot.robotState is null");
        }
    }

    public void autonomousInit(String autoSelection) {
        e.resetMaxCurrents();
        this.autoSelection = autoSelection;
        act.actionReset();
        e.stopSwerveMotors();
        setStepNumber(1);
        rLog.print("Start Autonomous Program: " + autoSelection);
        e.stopAllMotors();

        if (autoSelection.equals("Sample 1")) {
        }

        if (autoSelection.equals("Sample 2")) {
        }
    }

    public void autonomousPeriodic() {
        e.checkMaxCurrents();
        if (autoSelection.equals("na"))
            e.alignSwerveMotorsForward();
        else if (autoSelection.equals("Sample 1"))
            sample1();
        else if (autoSelection.equals("Sample 2"))
            sample2();
        else {
            rLog.print("Your selection for the Autonomus program is incorrect. Selected: " + autoSelection);
        }
    }

    void setStepNumber(int number) {
        stepNumber = number;
        act.actionReset();
        timer1.reset();
        timer1.start();
        rLog.print("New Auto Step Number: " + stepNumber + " Gyro: " + RobotMath.round1(e.getGyro()));
    }

    void setNextStepNumber() {
        setStepNumber(stepNumber + 1);
    }

    void sample1() {
        switch (stepNumber) {
            case 1: // Sample 1: drive forward
                if (act.driveStraightWithFacing(0, 30, 0, 50, 51, 25)) {
                    setNextStepNumber();
                }
                break;

            case 2: // Sample 1: drive backward
                if (act.driveStraightWithFacing(180, 30, 0, 50, 51, 25)) {
                    setNextStepNumber();
                }
                break;

            case 3: // Sample 1: stop
                e.stopSwerveMotors();
                break;
        }
    }

    void sample2() {
        switch (stepNumber) {
            case 1: // Sample 2: drive curved to right
                if (act.driveCurveWithFacing(0, 180, 30, 0, 51, 90)) {
                    setNextStepNumber();
                }
                break;

            case 2: // Sample 2: drive curved to right
                if (act.driveCurveWithFacing(180, 0, 30, 0, 51, 90)) {
                    setNextStepNumber();
                }
                break;

            case 3: // Sample 2: stop
                e.stopSwerveMotors();
                break;
        }
    }
}
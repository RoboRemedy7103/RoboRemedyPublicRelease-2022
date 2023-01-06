// Robot.java - main robot code for 2023
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    private final String projectName = "2023 Competition";
    private RoboLog rLog = new RoboLog();
    private RobotState robotState = new RobotState(rLog);
    private Electronics e = new Electronics(true, rLog, robotState);
    private OI oi = new OI();
    private Action act = new Action(e, rLog, robotState);;
    private Dashboard dash = new Dashboard(e, rLog, robotState, act);
    private AutoRobot auto = new AutoRobot(e, rLog, act, dash, robotState);
    private TeleopRobot teleop = new TeleopRobot(e, rLog, act, oi, robotState, dash);
    private TestRobot test = new TestRobot(e, rLog, act, oi.driver, dash, robotState, oi);
    private Timer autoTimer = new Timer();
    private Timer autoDisplayTimer = new Timer();
    private String lastAuto = "";
    private boolean lastLEDTest = false;

    /* Called once when robot starts */
    @Override
    public void robotInit() {
        rLog.print(projectName + " robotInit");
        e.assignAllEncoderValues();
        autoDisplayTimer.reset();
        autoDisplayTimer.start();
        LiveWindow.disableAllTelemetry(); // Improve performance
        SmartDashboard.updateValues(); // Improve performance
        PhotonVision.PhotonVisionInit();
        PhotonVision.turnOffLED();
        e.setSwerveCoast();
    }

    /* Called periodically in all modes */
    @Override
    public void robotPeriodic() {
        act.logChanges();

        if (DriverStation.isDSAttached()) {
            dash.dashboardPeriodic();
            if (oi.getOperatorResetGyroButtonPressed() || oi.getDriverResetGyroButtonPressed()) {
                e.resetGyro();
            }
        }

        e.robotPeriodic();
    }

    /* Called once whenever robot is disabled */
    @Override
    public void disabledInit() {
        rLog.setRobotMode("DISA", "Disabled");
        e.reportMaxCurrents();
    }

    /* Called periodically while robot is disabled */
    @Override
    public void disabledPeriodic() {
        if (autoDisplayTimer.get() > 30 || !lastAuto.equals(dash.getAutoMode())) {
            autoDisplayTimer.reset();
            autoDisplayTimer.start();
            lastAuto = dash.getAutoMode();
            rLog.print("Auto Selection = " + lastAuto);
        }

        if (e.isPhyscalResetGyroButtonPressed()) {
            rLog.print("Physical Gyro reset button pressed: reset Gyro and Encoders");
            e.resetGyro();
            e.assignAllEncoderValues();
        }

        if (e.getGyroCenteredOnGoal(0) > -1.5 && e.getGyroCenteredOnGoal(0) < 1.5) {
            e.ledDisplay(true);
        } else
            e.ledDisplay(false);

        if (e.isTestLEDButtonPressed()) {
            PhotonVision.turnOnLED();
            if (lastLEDTest == false)
                rLog.print("LED Test Button Pressed");
        } else {
            PhotonVision.turnOffLED();
            if (lastLEDTest == true)
                rLog.print("LED Test Button Released");
        }
        lastLEDTest = e.isTestLEDButtonPressed();
    }

    /* Called once when autonomous is started */
    @Override
    public void autonomousInit() {
        rLog.setRobotMode("AUTO", "Autonomus");
        String autoSelected = dash.getAutoMode();
        rLog.print(projectName + " autonomousInit, auto: " + autoSelected + " PV Attached: " +
                robotState.getPhotonVisionAttached() + " gyro: " + Math.round(e.getGyro()));
        auto.autonomousInit(autoSelected);
    }

    /* Called periodically during autonomous */
    @Override
    public void autonomousPeriodic() {
        auto.autonomousPeriodic();
        autoTimer.reset();
        autoTimer.start();
    }

    /* Called once when operator control is started */
    @Override
    public void teleopInit() {
        rLog.setRobotMode("TELE", "Teleop");
        rLog.print(projectName + " teleopInit");
        teleop.teleopInit();
        PhotonVision.turnOnLED();
    }

    /* Called periodically during operator control */
    @Override
    public void teleopPeriodic() {
        teleop.teleopPeriodic();
    }

    /* Called once when test mode is started */
    @Override
    public void testInit() {
        rLog.setRobotMode("TEST", "Test");
        rLog.print(projectName + " testInit");
        test.testInit();
    }

    /* Called periodically during test mode */
    @Override
    public void testPeriodic() {
        test.testPeriodic();
    }
}
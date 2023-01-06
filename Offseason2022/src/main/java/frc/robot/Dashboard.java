// Dashboard.java - Controls the Shuffleboard dashboard
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;

import java.util.Map;

public class Dashboard {
    private RoboLog rLog;
    private Electronics e;
    RobotState robotState;
    Action act;

    // Teleop Tab
    GenericEntry alliance;

    // Test Tab
    private GenericEntry testString = null;
    private SendableChooser<String> testChooser = null;

    GenericEntry demoMode = null;

    // Autonomous Tab
    private SendableChooser<String> autoChooser = null;
    GenericEntry goToTerminal = null;
    GenericEntry autoValue = null;

    // PreGame Tab
    GenericEntry frontLeftDrive;
    GenericEntry frontLeftTurn;
    GenericEntry frontRightDrive;
    GenericEntry frontRightTurn;
    GenericEntry backLeftDrive;
    GenericEntry backLeftTurn;
    GenericEntry backRightDrive;
    GenericEntry backRightTurn;
    GenericEntry isBothJoystick;
    GenericEntry isBattery;

    Dashboard(Electronics e, RoboLog rLog, RobotState robotState, Action act) {
        this.e = e;
        this.rLog = rLog;
        this.robotState = robotState;
        this.act = act;

        if (this.rLog == null) {
                System.out.println("Warning: rLog is null in Action");
            }
            
        InitTelop();
        InitTest();
        InitAutonomous();
        InitPreGame();
    }

    public void selectTab(String selectTab) {
        rLog.print("Selecting new tab: " + selectTab);
        Shuffleboard.selectTab(selectTab);
    }

    public void InitTelop() {
        Shuffleboard.getTab("Teleop")
                .add("Robot Name", e.getRobotName())
                .withPosition(6, 0)
                .getEntry();

        Shuffleboard.getTab("Teleop")
                .add("Gyro", e.gyro)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(7, 0)
                .withSize(2, 2);

        alliance = Shuffleboard.getTab("Teleop")
                .add("Alliance", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "red", "color when false", "blue"))
                .withPosition(5, 2)
                .withSize(4, 1)
                .getEntry();
    }

    public void InitTest() {

        testChooser = new SendableChooser<String>();
        testChooser.setDefaultOption("Quick Test", "Quick Test");
        testChooser.addOption("Swerve Test", "Swerve Test");
        testChooser.addOption("Test Angle", "Test Angle");
        testChooser.addOption("Test Distance Sensor", "Test Distance Sensor");
        testChooser.addOption("Test Motors", "Test Motors");
        testChooser.addOption("Yellow Button", "Yellow Button");
        testChooser.addOption("Tune Motor", "Tune Motor");

        Shuffleboard.getTab("Test")
                .add("Test Mode", testChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        Shuffleboard.getTab("Test")
                .add("Gyro Deg", e.gyro)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(7, 0)
                .withSize(2, 2);

        Shuffleboard.getTab("Test")
                .add("Robot Name", e.getRobotName())
                .withPosition(6, 0)
                .withSize(1, 1)
                .getEntry();

        testString = Shuffleboard.getTab("Test")
                .add("Test String", "")
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();
    }

    public void InitAutonomous() {
        autoChooser = new SendableChooser<String>();
        autoChooser.setDefaultOption("None", "na");
        autoChooser.addOption("Sample 1", "Sample 1");
        autoChooser.addOption("Sample 2", "Sample 2");

        Shuffleboard.getTab("Autonomous")
                .add("Autonomous Mode", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        autoValue = Shuffleboard.getTab("Autonomous")
                .add("Auto Value", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();

        goToTerminal = Shuffleboard.getTab("Autonomous")
                .add("Go To Terminal", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();

        Shuffleboard.getTab("Autonomous")
                .add("Robot Name", e.getRobotName())
                .withPosition(6, 0)
                .withSize(1, 1)
                .getEntry();

        Shuffleboard.getTab("Autonomous")
                .add("Gyro", e.gyro)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(7, 0)
                .withSize(2, 2);
    }

    public void InitPreGame() {

        frontLeftDrive = Shuffleboard.getTab("PreGame")
                .add("fL Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 0)
                .getEntry();

        frontLeftTurn = Shuffleboard.getTab("PreGame")
                .add("fL Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 1)
                .getEntry();

        frontRightDrive = Shuffleboard.getTab("PreGame")
                .add("fR Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(1, 0)
                .getEntry();

        frontRightTurn = Shuffleboard.getTab("PreGame")
                .add("fR Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(1, 1)
                .getEntry();

        backLeftDrive = Shuffleboard.getTab("PreGame")
                .add("bL Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(2, 0)
                .getEntry();

        backLeftTurn = Shuffleboard.getTab("PreGame")
                .add("bL Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(2, 1)
                .getEntry();

        backRightDrive = Shuffleboard.getTab("PreGame")
                .add("bR Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(3, 0)
                .getEntry();

        backRightTurn = Shuffleboard.getTab("PreGame")
                .add("bR Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(3, 1)
                .getEntry();

        Shuffleboard.getTab("PreGame")
                .add("Robot Name", e.getRobotName())
                .withPosition(6, 0)
                .getEntry();

        Shuffleboard.getTab("PreGame")
                .add("Gyro Deg", e.gyro)
                .withPosition(7, 0)
                .withWidget(BuiltInWidgets.kGyro)// kDial
                .withSize(2, 2);

        isBothJoystick = Shuffleboard.getTab("PreGame")
                .add("Both Joystick", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(8, 2)
                .getEntry();

        isBattery = Shuffleboard.getTab("PreGame")
                .add("Battery OK", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(6, 2)
                .getEntry();
    }

    public void dashboardPeriodic() {
        // Test Entries

        // Teleop Entries
        alliance.setBoolean(e.isRedAlliance());

        // Autonomous Entries

        // Pre-Game Entries
        // Swerve
        frontLeftDrive.setBoolean(e.isMotorAttached(true, 0, 2, 0));
        frontLeftTurn.setBoolean(e.isMotorAttached(true, 0, 1, 0));
        frontRightDrive.setBoolean(e.isMotorAttached(true, 1, 2, 0));
        frontRightTurn.setBoolean(e.isMotorAttached(true, 1, 1, 0));
        backLeftDrive.setBoolean(e.isMotorAttached(true, 3, 2, 0));
        backLeftTurn.setBoolean(e.isMotorAttached(true, 3, 1, 0));
        backRightDrive.setBoolean(e.isMotorAttached(true, 2, 2, 0));
        backRightTurn.setBoolean(e.isMotorAttached(true, 2, 1, 0));
        // others
        isBothJoystick.setBoolean(DriverStation.isJoystickConnected(1) && DriverStation.isJoystickConnected(0));
        isBattery.setBoolean(e.isBatteryGood());
    }

    public String getTestMode() {
        if (testChooser != null)
            return testChooser.getSelected();
        else
            return "";
    }

    public String getAutoMode() {
        if (autoChooser != null)
            return autoChooser.getSelected();
        else
            return "";
    }

    public double getAutoValue() {
        if (autoValue != null)
            return autoValue.getDouble(0.0);
        else
            return 0.0;
    }

    public void setTestString(String string) {
        if (testString != null)
            testString.setString(string);
    }

    public boolean isDemoMode() {
        if (demoMode != null)
            return demoMode.getBoolean(false);
        else
            return false;
    }
}
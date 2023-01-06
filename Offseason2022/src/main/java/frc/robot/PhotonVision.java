// PhotonVision.java - Photonvision vision control
package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class PhotonVision {

    private static LinearMapper mapper = new LinearMapper();

    public enum CargoColor {
        red, blue
    }

    public enum VisionPipeline {
        target, blueCargo, redCargo
    }

    // http://hoodvision.local:5800/#/dashboard
    static PhotonCamera target = null; // new PhotonCamera("target2022");
    static PhotonCamera cargo = null; //new PhotonCamera("drivervision");

    public static void PhotonVisionInit() {
        // Map Pitch values to Target Distance values in feet, using linear mapper
        mapper.add(-22.28, 24);
        mapper.add(-21.89, 23);
        mapper.add(-20.94, 22);
        mapper.add(-20.21, 21);
        mapper.add(-19.38, 20);
        mapper.add(-18.42, 19);
        mapper.add(-17.06, 18);
        mapper.add(-15.76, 17);
        mapper.add(-13.79, 16);
        mapper.add(-11.79, 15);
        mapper.add(-10.15, 14);
        mapper.add(-7.93, 13);
        mapper.add(-5.04, 12);
        mapper.add(-1.82, 11);
        mapper.add(1.78, 10);
        mapper.add(6.43, 9);
        mapper.add(11.76, 8);
        mapper.add(15.05, 7.5);
        mapper.add(17.81, 7);   
        mapper.add(21.28, 6.5);    
    }

    // Set the green LED mode for target camera
    // -1: Default; 0: Off; 1: On; 2: Blink
    public static void setLEDMode(VisionLEDMode mode) {
        if(isAttached()){
            target.setLED(mode);
        }
    }

    public static void turnOnLED() {
        if (target != null)
            target.setLED(VisionLEDMode.kOn);
    }

    public static void turnOffLED() {
        if (target != null)
            target.setLED(VisionLEDMode.kOff);
    }

    // Set Pipeline
    public static void setPipeline(VisionPipeline object) {
        if(isAttached()){
            if (object == VisionPipeline.target) {
                target.setPipelineIndex(0);
            } else if (object == VisionPipeline.blueCargo) {
                cargo.setPipelineIndex(1);
            } else if (object == VisionPipeline.redCargo) {
                cargo.setPipelineIndex(2);
            }
        }
    }

    public static boolean isAttached() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/target2022");
        double a = table.getEntry("pipelineIndex").getDouble(-2);
        return (a != -2);
    }

    public static boolean isDriverVisionAttached() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/drivervision");
        double a = table.getEntry("pipelineIndex").getDouble(-2);
        return (a != -2);
    }

    // Return true if the target is found
    public static boolean isTargetFound() {
        if (target == null)
            return false;

        var result = target.getLatestResult();

        boolean isTargetFound = false;
        isTargetFound = result.hasTargets();
        return isTargetFound;
    }

    // Return true if a cargo is found on current pipeline
    public static boolean isCargoFound() {
        if (cargo == null)
            return false;

        var result = cargo.getLatestResult();

        boolean isCargoFound = false;
        isCargoFound = result.hasTargets();
        return isCargoFound;
    }

    // Return true if the desired color cargo is found
    public static boolean isColoredCargoFound(CargoColor desiredCargoColor) {
        if (desiredCargoColor == CargoColor.red) {
            setPipeline(VisionPipeline.redCargo);
        } else if (desiredCargoColor == CargoColor.blue) {
            setPipeline(VisionPipeline.blueCargo);
        }

        return isCargoFound();
    }

    // Calculate distance from known pitch based on the data collected
    public static double calculateDistanceInInchesFromPitchDegrees(double pitch) {
        double calcDist = 0.0;
        calcDist = mapper.calculate(pitch);
        return calcDist * 12; // from feet to inches
    }

    public static double getTargetPitchDegrees() {
        if (target == null)
            return 0.0;

        var result = target.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                return targetResult.getPitch();
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }

    public static double calculateDistanceInInches() {
        return calculateDistanceInInchesFromPitchDegrees(getTargetPitchDegrees());
    }

    // Get target pitch and radians
    public static double getTargetPitchRadians() {
        double pitch = Units.degreesToRadians(getTargetPitchDegrees());
        return pitch;
    }

    public static double getTargetYawDegrees() {
        if (target == null)
            return 0.0;

        var result = target.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                double yaw = targetResult.getYaw();
                return yaw - 0;//6.5
            }
            return 0;
        }
        return 0;
    }

    public static double getBallYawDegrees() {
        if (cargo == null)
            return 0.0;

        var result = cargo.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                double yaw = targetResult.getYaw();
                return yaw - 0;//6.5
            }
            return -100;
        }
        return -200;
    }
}
// RobotState.java
package frc.robot;

public class RobotState {
    private RoboLog rLog;
 
    private boolean isTargetFound = false;
    private boolean isPhotonVisionAttached = false;
    private boolean isDriverVisionAttached = false;

    public RobotState(RoboLog rLog) {
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: rLog is null in RobotState");
        }
    }

    public void setTargetFound(boolean found) {
        isTargetFound = found;
    }

    public boolean getTargetFound() {
        return isTargetFound;
    }

    public void setPhotonVisionAttached(boolean attached) {
        isPhotonVisionAttached = attached;
    }

    public boolean getPhotonVisionAttached() {
        return isPhotonVisionAttached;
    }

    public void setDriverVisionAttached(boolean attached) {
        isDriverVisionAttached = attached;
    }

    public boolean getDriverVisionAttached() {
        return isDriverVisionAttached;
    }
}
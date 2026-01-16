// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Made with Claude, i ain't boutta type allat 

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for interacting with Limelight camera
 * Provides easy access to Limelight NetworkTables values
 */
public class LimeLightHelpers {
    
    private final NetworkTable limelightTable;
    private final String limelightName;
    
    /**
     * Create LimelightHelpers for default limelight
     */
    public LimeLightHelpers() {
        this("limelight");
    }
    
    /**
     * Create LimelightHelpers for specific limelight
     * @param limelightName Name of the limelight (e.g., "limelight-front")
     */
    public LimeLightHelpers(String limelightName) {
        this.limelightName = limelightName;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    }
    
    // ===== BASIC VISION DATA =====
    
    /**
     * Check if Limelight has a valid target
     * @return True if target is detected (Changed to return 1)
     */
    public double hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0);  // slightly modified to return a double
    }
    
    /**
     * Get horizontal offset from crosshair to target
     * @return Horizontal offset in degrees (-29.8 to 29.8)
     */
    public double getTargetX() {
        return limelightTable.getEntry("tx").getDouble(0);
    }
    
    /**
     * Get vertical offset from crosshair to target
     * @return Vertical offset in degrees (-24.85 to 24.85)
     */
    public double getTargetY() {
        return limelightTable.getEntry("ty").getDouble(0);
    }
    
    /**
     * Get target area (percentage of image)
     * @return Target area (0% to 100%)
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }
    
    /**
     * Get AprilTag ID
     * @return AprilTag ID, or -1 if not an AprilTag
     */
    public int getAprilTagID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }
    
    /**
     * Get target skew/rotation
     * @return Skew in degrees (-90 to 0)
     */
    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }
    
    // ===== MEGATAG2 POSE ESTIMATION =====
    // _orb means use megatag2
    /**
     * Get robot pose from MegaTag2 (blue alliance origin)
     * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
     */
    public double[] getBotPoseBlue() {
        return limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
    }
    
    /**
     * Get robot pose from MegaTag2 (red alliance origin)
     * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
     */
    public double[] getBotPoseRed() {
        return limelightTable.getEntry("botpose_orb_wpired").getDoubleArray(new double[6]);
    }
    
    /**
     * Get robot pose as Pose2d (automatically uses correct alliance)
     * @return Robot pose on field
     */
    public Pose2d getRobotPose2d() {
        var alliance = DriverStation.getAlliance();
        
        double[] pose;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            pose = getBotPoseRed();
        } else {
            pose = getBotPoseBlue();
        }
        
        if (pose.length >= 6) {
            return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
        }
        
        return new Pose2d();
    }
    
    /**
     * Get full MegaTag2 data including quality metrics
     * @return Array [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea] (Only for Blue)
     * @return Don't use for position since it will not account for alliance diffrence
     */
    public double[] getBotMT2Data() {
        return limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
    }

    /**
     * Get number of AprilTags used in MegaTag2 solution
     * @return Number of tags (0 if no solution)
     */
    public int getMT2TagCount() {
        double[] pose = getBotMT2Data();
        return pose.length >= 8 ? (int) pose[7] : 0;
    }
    
    /**
     * Get tag span (coverage area of tags in image)
     * Higher span = more reliable pose
     * @return Tag span (0.0 to 1.0)
     */
    public double getMT2TagSpan() {
        double[] pose = getBotMT2Data();
        return pose.length >= 9 ? pose[8] : 0.0;
    }
    
    /**
     * Get average distance to tags in MegaTag2 solution
     * @return Average distance in meters
     */
    public double getMT2AvgTagDistance() {
        double[] pose = getBotMT2Data();
        return pose.length >= 10 ? pose[9] : 0.0;
    }
    
    /**
     * Check if MegaTag2 pose is reliable
     * Requires multiple tags with good coverage
     * @return true if pose is reliable
     */
    public boolean isMT2PoseReliable() {
        int tagCount = getMT2TagCount();
        double tagSpan = getMT2TagSpan();
        
        return tagCount >= 2 && tagSpan > 0.4;
    }
    
    // ===== LATENCY =====
    
    /**
     * Get pipeline latency
     * @return Latency in milliseconds
     */
    public double getPipelineLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }
    
    /**
     * Get capture latency
     * @return Latency in milliseconds
     */
    public double getCaptureLatency() {
        return limelightTable.getEntry("cl").getDouble(0);
    }
    
    /**
     * Get total latency (pipeline + capture)
     * @return Total latency in seconds
     */
    public double getTotalLatencySeconds() {
        return (getPipelineLatency() + getCaptureLatency()) / 1000.0;
    }
    
    // ===== CAMERA CONTROL =====
    
    /**
     * Set LED mode
     * @param mode 0=pipeline default, 1=off, 2=blink, 3=on
     */
    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Set camera mode
     * @param mode 0=vision processor, 1=driver camera
     */
    public void setCameraMode(int mode) {
        limelightTable.getEntry("camMode").setNumber(mode);
    }
    
    /**
     * Set pipeline number
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Get current pipeline number
     * @return Current pipeline index
     */
    public int getPipeline() {
        return (int) limelightTable.getEntry("getpipe").getDouble(0);
    }
    
    /**
     * Take snapshot
     * @param mode 0=stop snapshots, 1=take two snapshots per second
     */
    public void setSnapshot(int mode) {
        limelightTable.getEntry("snapshot").setNumber(mode);
    }
    
    /**
     * Set driver mode (convenience method)
     * @param enabled true to enable driver mode, false for vision mode
     */
    public void setDriverMode(boolean enabled) {
        setCameraMode(enabled ? 1 : 0);
        setLEDMode(enabled ? 1 : 3);
    }
    
    // ===== DISTANCE CALCULATION =====
    
    /**
     * Calculate distance to target using trigonometry
     * @param targetHeightMeters Height of target above ground
     * @param cameraHeightMeters Height of camera above ground
     * @param cameraMountAngleDegrees Angle of camera from horizontal
     * @return Distance to target in meters
     */
    public double getDistanceToTarget(double targetHeightMeters, double cameraHeightMeters, 
                                     double cameraMountAngleDegrees) {
        double ty = getTargetY();
        double angleToTargetDegrees = cameraMountAngleDegrees + ty;
        double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);
        
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToTargetRadians);
    }
    
    // ===== UTILITY =====
    
    /**
     * Get the name of this Limelight
     * @return Limelight name
     */
    public String getLimelightName() {
        return limelightName;
    }
    
    /**
     * Get the NetworkTable for this Limelight
     * @return NetworkTable instance
     */
    public NetworkTable getTable() {
        return limelightTable;
    }
}

// ===== USAGE EXAMPLES =====
//
// In your subsystem:
// private final LimelightHelpers limelight = new LimelightHelpers();
//
// Or for specific camera:
// private final LimelightHelpers frontCamera = new LimelightHelpers("limelight-front");
//
// Then use it:
// if (limelight.hasTarget()) {
//     double tx = limelight.getTargetX();
//     double distance = limelight.getDistanceToTarget(2.08, 0.4, 20.0);
// }
//
// For MegaTag2:
// if (limelight.isMT2PoseReliable()) {
//     Pose2d pose = limelight.getRobotPose2d();
// }


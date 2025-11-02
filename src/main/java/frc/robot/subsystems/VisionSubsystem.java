package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.MechConstants;
import frc.robot.constants.TunerConstants;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Camera and Target Constants
    private static final double CAMERA_ANGLE = 25.0; // Adjust based on mounting angle
    private static final double TARGET_HEIGHT = 2.64; // Target height in meters
    private static final double CAMERA_HEIGHT = 0.90; // Camera height in meters

    // AprilTag tracking constants
    private static final double TARGET_DISTANCE_METERS = 2.0; // Target distance: 2 meters
    private static final double DISTANCE_TOLERANCE = 0.1; // Stop within 10cm of target
    private static final double ANGLE_TOLERANCE = 2.0; // Stop within 2 degrees of target
    private static final double MIN_DRIVE_SPEED = 0.1; // Minimum speed to prevent stalling

    double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        // Set pipeline 0 for AprilTag detection (configure this in Limelight web interface)
        setPipeline(0);
        // Turn on LEDs for better detection
        setLedMode(3);
    }

    /**
     * Checks if the Limelight has a valid target.
     * 
     * @return true if a target is detected, false otherwise.
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Gets the horizontal offset (tx) from the crosshair to the target.
     * 
     * @return Horizontal offset in degrees.
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * Gets the vertical offset (ty) from the crosshair to the target.
     * 
     * @return Vertical offset in degrees.
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    /**
     * Estimates distance to the target using the camera angle and target height.
     *
     * @return Estimated distance in meters.
     */
    public double getEstimatedDistance() {
        if (!hasTarget()) {
            return -1.0;
        }
        double angleToTarget = CAMERA_ANGLE + getVerticalOffset();
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(angleToTarget));
    }

    /**
     * Checks if the robot is aimed at the target within tolerance.
     *
     * @return true if horizontal offset is within ANGLE_TOLERANCE degrees.
     */
    public boolean isAimedAtTarget() {
        return hasTarget() && Math.abs(getHorizontalOffset()) < ANGLE_TOLERANCE;
    }

    /**
     * Checks if the robot is at the target distance within tolerance.
     *
     * @return true if distance is within DISTANCE_TOLERANCE of TARGET_DISTANCE_METERS.
     */
    public boolean isAtTargetDistance() {
        double distance = getEstimatedDistance();
        return distance > 0 && Math.abs(distance - TARGET_DISTANCE_METERS) < DISTANCE_TOLERANCE;
    }

    /**
     * Checks if the robot is both aimed at the target and at the correct distance.
     *
     * @return true if both aimed and at target distance.
     */
    public boolean isAlignedWithTarget() {
        return isAimedAtTarget() && isAtTargetDistance();
    }

    /**
     * Sets the LED mode of the Limelight.
     * 0 = Pipeline default, 1 = Force off, 2 = Force blink, 3 = Force on
     * 
     * @param mode LED mode value.
     */
    public void setLedMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the Limelight pipeline.
     * 
     * @param pipeline Pipeline index (0-9).
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard values for debugging
        SmartDashboard.putBoolean("Vision/Target Detected", hasTarget());
        SmartDashboard.putNumber("Vision/Horizontal Offset (tx)", getHorizontalOffset());
        SmartDashboard.putNumber("Vision/Vertical Offset (ty)", getVerticalOffset());
        SmartDashboard.putNumber("Vision/Estimated Distance", getEstimatedDistance());
        SmartDashboard.putBoolean("Vision/Aimed at Target", isAimedAtTarget());
        SmartDashboard.putBoolean("Vision/At Target Distance", isAtTargetDistance());
        SmartDashboard.putBoolean("Vision/Fully Aligned", isAlignedWithTarget());
    }

    // "proportional control" is a control algorithm in which the output is
    // proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional
    // to the
    // "tx" value from the Limelight.
    double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = getHorizontalOffset() * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // Proportional ranging control to maintain TARGET_DISTANCE_METERS from the target
    // Uses estimated distance to calculate error from desired distance
    double limelight_range_proportional() {
        double kP = 0.5; // Proportional gain for distance control
        double currentDistance = getEstimatedDistance();

        if (currentDistance < 0) {
            return 0.0; // No target detected
        }

        // Calculate distance error (positive = too far, negative = too close)
        double distanceError = currentDistance - TARGET_DISTANCE_METERS;

        // Calculate forward speed proportional to distance error
        double targetingForwardSpeed = distanceError * kP;

        // Clamp speed to max speed
        targetingForwardSpeed = Math.max(-MaxSpeed, Math.min(MaxSpeed, targetingForwardSpeed));

        // Stop if within tolerance
        if (Math.abs(distanceError) < DISTANCE_TOLERANCE) {
            return 0.0;
        }

        // Apply minimum speed to prevent stalling
        if (Math.abs(targetingForwardSpeed) < MIN_DRIVE_SPEED && Math.abs(targetingForwardSpeed) > 0.01) {
            targetingForwardSpeed = Math.copySign(MIN_DRIVE_SPEED, targetingForwardSpeed);
        }

        return targetingForwardSpeed;
    }

    /**
     * Aligns the robot to an AprilTag target.
     * Rotates to aim at the target and drives to maintain 2 meters distance.
     *
     * @param drivetrain The swerve drivetrain subsystem
     */
    public void align(Swerve drivetrain) {
        // If no target detected, stop the robot
        if (!hasTarget()) {
            double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
            final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // Calculate rotational speed to aim at target
        // Stop rotation if aimed at target
        final double rotationalSpeed = isAimedAtTarget() ? 0.0 : limelight_aim_proportional();

        // Calculate forward speed to reach target distance (2 meters)
        final double forwardSpeed = limelight_range_proportional();

        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        // Use robot-centric drive for vision tracking (field-relative is off)
        final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Apply the calculated speeds to the drivetrain
        // VelocityX is forward/backward (positive = forward)
        // VelocityY is left/right (0 = no strafing)
        // RotationalRate is rotation speed
        drivetrain.applyRequest(() -> drive
                .withVelocityX(forwardSpeed)
                .withVelocityY(0)
                .withRotationalRate(rotationalSpeed));
    }
    
    public Command alignCommand(Swerve drive) {
        return run(() -> align(drive));
    }
}

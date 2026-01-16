// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

<<<<<<< HEAD
import java.util.function.Supplier;

import javax.sound.sampled.LineEvent;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
=======
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

>>>>>>> e229d2be188447f824b8819b37f7a8333dccdacf
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignment extends Command {
  /** Creates a new AutoAlignment. */

<<<<<<< HEAD
  private final CommandSwerveDrivetrain drivetrain;
  private final String limelightname = "limelight";
  private final int targetID;
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;

  private final PIDController rotController;
  private final SwerveRequest.FieldCentric fieldCentricdrive;

  private static final double ROTATION_TOLERANCE = 2.0;


  public AutoAlignment(CommandSwerveDrivetrain drivetrain,
                                int targetID,
                                Supplier<Double> xSpeed,
                                Supplier<Double> ySpeed) {
    this.drivetrain = drivetrain;
    this.targetID = targetID;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;

    this.rotController = new PIDController(3.0, 0.0, 0.15);
    rotController.enableContinuousInput(-180, 180);
    rotController.setTolerance(ROTATION_TOLERANCE);
        
    this.fieldCentricdrive = new SwerveRequest.FieldCentric();
        
    addRequirements(drivetrain);

=======
  public AutoAlignment(CommandSwerveDrivetrain drivetrain,LimeLightHelpers limelight,
  int target, double offsetX, double offsetY, double targetRotation  ) {
    // Use addRequirements() here to declare subsystem dependencies.
>>>>>>> e229d2be188447f824b8819b37f7a8333dccdacf

  }

  // Called when the command is initially scheduled.
  @Override
<<<<<<< HEAD
  public void initialize() {
    System.out.print("Bomj");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.SetRobotOrientation(limelightname, drivetrain.getState().Pose.getRotation().getDegrees(),0 , 0, 0, 0, 0);

    Pose2d currentPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightname);

    LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightname);


    if(currentPose != null && rawFiducials.length >= 2 && isPoseReliable(rawFiducials)){

      Pose2d targetPose = getAprilTagPose(targetID);

      if(targetPose != null){
        //Rotation 

      }


    }
  }


      private boolean isPoseReliable(LimelightHelpers.RawFiducial[] rawFiducials) {
        double avgDist = 0;
        for (LimelightHelpers.RawFiducial fid : rawFiducials) {
            avgDist += fid.distToRobot;
        }
        avgDist /= rawFiducials.length;
        
        if (avgDist > 6.0) {
            return false;
        }
        
        for (LimelightHelpers.RawFiducial fid : rawFiducials) {
            if (fid.ambiguity > 0.5) {
                return false;
            }
        }
        
        return true;
    }

        private Pose2d getAprilTagPose(int tagID) {
        return switch (tagID) {
            // Blue Alliance Wall Tags
            case 1 -> new Pose2d(0.0, 6.0, new Rotation2d(0));
            case 2 -> new Pose2d(0.0, 5.0, new Rotation2d(0));
            case 3 -> new Pose2d(0.0, 4.0, new Rotation2d(0));
            case 4 -> new Pose2d(0.0, 3.0, new Rotation2d(0));
            
            // Red Alliance Wall Tags
            case 5 -> new Pose2d(16.54, 6.0, new Rotation2d(Math.PI));
            case 6 -> new Pose2d(16.54, 5.0, new Rotation2d(Math.PI));
            case 7 -> new Pose2d(16.54, 4.0, new Rotation2d(Math.PI));
            case 8 -> new Pose2d(16.54, 3.0, new Rotation2d(Math.PI));
            
            // Bump & Trench Tags
            case 9 -> new Pose2d(2.5, 0.5, new Rotation2d(0));
            case 10 -> new Pose2d(5.0, 0.5, new Rotation2d(0));
            case 11 -> new Pose2d(11.54, 0.5, new Rotation2d(Math.PI));
            case 12 -> new Pose2d(14.04, 0.5, new Rotation2d(Math.PI));
            
            // Tower Tags
            case 13 -> new Pose2d(1.0, 3.0, new Rotation2d(0));
            case 14 -> new Pose2d(4.0, 3.0, new Rotation2d(0));
            case 15 -> new Pose2d(12.54, 3.0, new Rotation2d(Math.PI));
            case 16 -> new Pose2d(15.54, 3.0, new Rotation2d(Math.PI));
            
            // Hub Tags
            case 17 -> new Pose2d(8.27, 4.1, new Rotation2d(0));
            case 18 -> new Pose2d(8.27, 2.0, new Rotation2d(0));
            case 19 -> new Pose2d(8.27, 4.1, new Rotation2d(Math.PI));
            case 20 -> new Pose2d(8.27, 2.0, new Rotation2d(Math.PI));
            
            default -> null;
        };
    }
=======
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
>>>>>>> e229d2be188447f824b8819b37f7a8333dccdacf

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

<<<<<<< HEAD
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class AutoAlignSubSytem extends SubsystemBase {
//   /** Creates a new AutoAlignSubSytem. */
//   //private final LimelightHelpers frontCamara;
  
//   private final PIDController xController;
//   private final PIDController yController;
//   private final PIDController rotController;

//   private final CommandSwerveDrivetrain drivetrain;

//   final double POSITION_TOLERANCE = Units.inchesToMeters(.5);
//   final double ROTATION_TOLERANCE = Units.degreesToRadians(1);

//   private final SwerveRequest.FieldCentric fieldCentricDrive =
//     new SwerveRequest.FieldCentric();


//   public AutoAlignSubSytem(CommandSwerveDrivetrain drivetrain){
//     this.drivetrain = drivetrain;
//     //this.frontCamara = new LimeLightHelpers("limelight");

//      this.xController = new PIDController(2, 0, 1);
//      this.yController = new PIDController(2, 0, 1);
//     this.rotController = new PIDController(2, 0, 1);

//     this.rotController.enableContinuousInput(-180, 180);

//      // Set tolerances
//      this.xController.setTolerance(POSITION_TOLERANCE);
//      this.yController.setTolerance(POSITION_TOLERANCE);
//      this.rotController.setTolerance(ROTATION_TOLERANCE);


//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public Command align_TO_M2Tag(Pose2d targetpose, int target){
//     return run(() -> {
//       //double tv = frontCamara.hasTarget();

//       //if (tv == 1.0 && frontCamara.isMT2PoseReliable()) { // is MT2 pose is not defiened yet
//           Pose2d currentPose = drivetrain.getState().Pose;
          
//           double xVel = xController.calculate(currentPose.getX(), targetpose.getX());
//           double yVel = yController.calculate(currentPose.getY(), targetpose.getY());
//           double rotVel = rotController.calculate(
//               currentPose.getRotation().getDegrees(), 
//               targetpose.getRotation().getDegrees()
//           );

//           drivetrain.setControl(fieldCentricDrive
//               .withVelocityX(xVel)
//               .withVelocityY(yVel)
//               .withRotationalRate(Math.toRadians(rotVel))
//           );
//       //} else {
//           drivetrain.setControl(fieldCentricDrive
//               .withVelocityX(0)
//               .withVelocityY(0)
//               .withRotationalRate(0)
//           );
//       }
//   //}).until(() -> isAlignedMegaTag(targetpose));
// }

// public boolean isMT2PoseReliable() {
//   //int tagCount = frontCamara.getMT2TagCount();
//   //double tagSpan = frontCamara.getMT2TagSpan(); // bigger span = more reliable
  
//   //return tagCount >= 2 && tagSpan > 0.4;
// }

  

//   private boolean isAlignedMegaTag(Pose2d targetPose){
//     double tv = frontCamara.hasTarget();

//       if (tv != 1.0) { // if target is not located
//         return false;
//     }

//         Pose2d currentPose = frontCamara.getRobotPose2d(); //automatically accounts for alliance diffrence
     
            
//             double xError = Math.abs(targetPose.getX() - currentPose.getX());
//             double yError = Math.abs(targetPose.getY() - currentPose.getY());
//             double rotError = Math.abs(targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());
            
//             return xError < POSITION_TOLERANCE && // this checks to see the if robot camara is within target 
//                    yError < POSITION_TOLERANCE && 
//                    rotError < ROTATION_TOLERANCE;
        
        
//     }


// }
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;

public class AutoAlignSubSytem extends SubsystemBase {
  /** Creates a new AutoAlignSubSytem. */
  private final LimeLightHelpers frontCamara;
  
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  private final CommandSwerveDrivetrain drivetrain;

  final double POSITION_TOLERANCE = Units.inchesToMeters(.5);
  final double ROTATION_TOLERANCE = Units.degreesToRadians(1);

  private final SwerveRequest.FieldCentric fieldCentricDrive =
    new SwerveRequest.FieldCentric();


  public AutoAlignSubSytem(CommandSwerveDrivetrain drivetrain){
    this.drivetrain = drivetrain;
    this.frontCamara = new LimeLightHelpers("limelight");

     this.xController = new PIDController(2, 0, 1);
     this.yController = new PIDController(2, 0, 1);
    this.rotController = new PIDController(2, 0, 1);

    this.rotController.enableContinuousInput(-180, 180);

     // Set tolerances
     this.xController.setTolerance(POSITION_TOLERANCE);
     this.yController.setTolerance(POSITION_TOLERANCE);
     this.rotController.setTolerance(ROTATION_TOLERANCE);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command align_TO_M2Tag(Pose2d targetpose, int target){
    return run(() -> {
      double tv = frontCamara.hasTarget();

      if (tv == 1.0 && frontCamara.isMT2PoseReliable()) { // is MT2 pose is not defiened yet
          Pose2d currentPose = drivetrain.getState().Pose;
          
          double xVel = xController.calculate(currentPose.getX(), targetpose.getX());
          double yVel = yController.calculate(currentPose.getY(), targetpose.getY());
          double rotVel = rotController.calculate(
              currentPose.getRotation().getDegrees(), 
              targetpose.getRotation().getDegrees()
          );

          drivetrain.setControl(fieldCentricDrive
              .withVelocityX(xVel)
              .withVelocityY(yVel)
              .withRotationalRate(Math.toRadians(rotVel))
          );
      } else {
          drivetrain.setControl(fieldCentricDrive
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0)
          );
      }
  }).until(() -> isAlignedMegaTag(targetpose));
}

public boolean isMT2PoseReliable() {
  int tagCount = frontCamara.getMT2TagCount();
  double tagSpan = frontCamara.getMT2TagSpan(); // bigger span = more reliable
  
  return tagCount >= 2 && tagSpan > 0.4;
}

  

  private boolean isAlignedMegaTag(Pose2d targetPose){
    double tv = frontCamara.hasTarget();

      if (tv != 1.0) { // if target is not located
        return false;
    }

        Pose2d currentPose = frontCamara.getRobotPose2d(); //automatically accounts for alliance diffrence
     
            
            double xError = Math.abs(targetPose.getX() - currentPose.getX());
            double yError = Math.abs(targetPose.getY() - currentPose.getY());
            double rotError = Math.abs(targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());
            
            return xError < POSITION_TOLERANCE && // this checks to see the if robot camara is within target 
                   yError < POSITION_TOLERANCE && 
                   rotError < ROTATION_TOLERANCE;
        
        
    }


}
>>>>>>> e229d2be188447f824b8819b37f7a8333dccdacf
  



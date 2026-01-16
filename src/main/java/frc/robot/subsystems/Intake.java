// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Intake extends SubsystemBase {

//   private static final TalonFX m_intake = new TalonFX(0);
//   private static final DutyCycleOut DUTY_CYCLE_OUT = new DutyCycleOut(2);
//   public StatusSignal<Angle> position;
//   public StatusSignal<Current> current;





//   /** Creates a new Intake. */
//   public Intake() {

//   }

//   public double getMotorPosition() {
//       return position.refresh().getValueAsDouble();
//     }
      

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }

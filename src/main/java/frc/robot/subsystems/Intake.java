// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake intake = null;

    public static final synchronized Intake get(){
      
      if(intake == null){
        intake = new Intake();
    }
    return intake;
  }

  private static final TalonFX m_intake = new TalonFX(12);
  private static final DutyCycleOut DUTY_CYCLE_OUT = new DutyCycleOut(0);
  public StatusSignal<Angle> position;
  public StatusSignal<Current> current;


  /** Creates a new Intake. */
  public Intake() {
    position = m_intake.getPosition();
    current = m_intake.getStatorCurrent();


  }

  public Command INTAKE(double POWER){
    return runEnd(()->m_intake.setControl(DUTY_CYCLE_OUT.withOutput(POWER)), this::stop);
  }

  public void stop() {
    m_intake.setControl(DUTY_CYCLE_OUT.withOutput(0));
  }
  

  public double getMotorPosition() {
      return position.refresh().getValueAsDouble();
    }
      

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

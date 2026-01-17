// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter shooter = null;
  
    public static final synchronized Shooter get(){
      
      if(shooter == null){
        shooter = new Shooter();
    }
    return shooter;
  }

  private TalonFX mShooter = new TalonFX(11);
  private DutyCycleOut DUTY_CYCLE_OUT = new DutyCycleOut(0);
  public StatusSignal<Angle> position;
  public StatusSignal<Current> current;



  /** Creates a new Shooter. */
  public Shooter() {
      position = mShooter.getPosition();
      current = mShooter.getStatorCurrent();

  }

  public void stop() {
    mShooter.setControl(DUTY_CYCLE_OUT.withOutput(0));
  }

  public Command SHOOT(double POWER){
      return runEnd(()-> mShooter.setControl(DUTY_CYCLE_OUT.withOutput(POWER)), this::stop);
  }



  public double getMotorPosition() {
      return position.refresh().getValueAsDouble();
    }
      
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Motor Current", current.refresh().getValueAsDouble());
  }
}

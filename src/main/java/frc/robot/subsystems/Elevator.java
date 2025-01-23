// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX m_LeftMotor = new TalonFX(14);
  private TalonFX m_RightMotor = new TalonFX(15);


  /** Creates a new Elevator. */
  public Elevator() {
    m_LeftMotor.setControl(new Follower(m_RightMotor.getDeviceID(), false));
    
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    m_RightMotor.set(speed);
  }
}

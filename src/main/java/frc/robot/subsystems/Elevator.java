// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX m_LeftMotor = new TalonFX(14);
  private TalonFX m_RightMotor = new TalonFX(15);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final double elevatorDriveRatio = Math.PI * 1.88900 * (1/12) * 2; // pi * sproket * gear ratio * doubled for pulley, # of inches per motor rotation

  private int currentLevel = 1;

  private final double elevatorBaseHeight = 0; // in inches, the height of the lowest position of the elevator's lowest position
  private final double l1Height = 2 - elevatorBaseHeight; // height above the floor to set elevator carridge to, base height is subtracted to calculate the actual distance the elevator must lift the carridge
  private final double l2Height = 3 - elevatorBaseHeight; // ^
  private final double l3Height = 4 - elevatorBaseHeight; // ^
  private final double l4Height = 5 - elevatorBaseHeight; // ^
  /**
  private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.25).withKV(0.12).withKA(0.01).withKP(4.8).withKI(0).withKD(0.1);
  MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
**/
  

  /** Creates a new Elevator. */
  public Elevator() {

    
    m_LeftMotor.setControl(new Follower(m_RightMotor.getDeviceID(), false));

     TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_RightMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLevel(int level){
    if (level > 0 && level < 5) currentLevel = level;

    switch(currentLevel){
      case 1:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
      case 2:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
      case 3:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
      case 4:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
    }

  }

  public void setLevel(){
    switch(currentLevel){
      case 1:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
      case 2:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
      case 3:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
      case 4:
        m_RightMotor.setControl(m_mmReq.withPosition(-inchesToRotations(l1Height)).withSlot(0));
        break;
    }
  }

  public double inchesToRotations(double inches) {
    return inches / elevatorDriveRatio;
  }
}

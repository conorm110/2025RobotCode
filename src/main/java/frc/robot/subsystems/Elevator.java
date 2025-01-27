// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX m_LeftMotorFollower = new TalonFX(Constants.Elevator.leftMotorID);
  private TalonFX m_RightMotorDriver = new TalonFX(Constants.Elevator.rightMotorID);

  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0); 

  private int currentLevel = 1; // Default level if setLevel(void) is called without previously calling setLevel(int level)
  private double currentVoltage = 0.0; // Default voltage is setVoltage(void) is called without previously calling setVoltage()
                                       // for debugging only, bad way to control elevator height !!
  
  ///////////////////////////////////////////////////     Constructor    //////////////////////////////////////////////////////// 
  public Elevator() {
    m_LeftMotorFollower.setControl(new Follower(m_RightMotorDriver.getDeviceID(), false));

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1.0; // gear ratio calculated with inchesToRotations()

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(Constants.Elevator.mm_cruiseVelocity))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(Constants.Elevator.mm_acceleration))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(Constants.Elevator.mm_jerk)); 

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = Constants.Elevator.elevatorkS; 
    slot0.kV = Constants.Elevator.elevatorkV;
    slot0.kA = Constants.Elevator.elevatorkA;
    slot0.kG = Constants.Elevator.elevatorkG;
    slot0.kP = Constants.Elevator.elevatorkP;
    slot0.kI = Constants.Elevator.elevatorkI;
    slot0.kD = Constants.Elevator.elevatorkD; 

    // apply motor configs
    StatusCode statusLeft = StatusCode.StatusCodeNotInitialized;
    StatusCode statusRight = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusRight = m_RightMotorDriver.getConfigurator().apply(cfg);
      statusLeft = m_LeftMotorFollower.getConfigurator().apply(cfg);
      if (statusLeft.isOK() && statusRight.isOK()) break;
    }
    if (!statusRight.isOK()) {
      System.out.println("Could not configure device. Error: " + statusLeft.toString());
    }
    if (!statusLeft.isOK()) {
      System.out.println("Could not configure device. Error: " + statusLeft.toString());
    }
  }

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
          Volts.of(Constants.Elevator.sysID_rampRate).div(Seconds.one()),
          Volts.of(Constants.Elevator.sysID_stepVoltage),
          null,                                                       // Use default timeout (10 s)
          (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
        ),
        new SysIdRoutine.Mechanism(
          (volts) -> m_RightMotorDriver.setControl(m_voltReq.withOutput(volts.in(Volts))),
          null,
          this
        )
    );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.quasistatic(direction);
}

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /////////////////////////////////////////////////// End of Constructor ////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////////      Methods       ////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    debugElevatorHeightSmartDashboard();
    debugElevatorVoltageSmartDashboard();
  }

  /* setLevel(int level) - Sets level that the elevator will try to target. Does not control the motor directly, 
     needs setLevel(void) to drive the motor */
  public void setLevel(int level){
    if (level > 0 && level < 5) currentLevel = level;
  }


  /* setLevel(void) - Use motion magic to target position set as currentLevel. */
  public void setLevel(){
    // positive rotations will make the elevator carridge travel up. 
    // negative rotations will make the elevator carridge travel down.
    switch(currentLevel){
      case 1:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(inchesToRotations(Constants.Elevator.l1Height)).withSlot(0));
        break;
      case 2:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(inchesToRotations(Constants.Elevator.l2Height)).withSlot(0));
        break;
      case 3:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(inchesToRotations(Constants.Elevator.l3Height)).withSlot(0));
        break;
      case 4:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(inchesToRotations(Constants.Elevator.l4Height)).withSlot(0));
        break;
    }
  }

  /* inchesToRotations(double inches) Converts inches to rotations of motor shaft for elevator. */
  public double inchesToRotations(double inches) {
    return  inches / Constants.Elevator.inchesPerRot;
  }

  /* setManualSpeed(double m_speed) - Sets speed without using PID controls. */
  public void setManualSpeed(double m_speed) {
    m_RightMotorDriver.set(m_speed);
  }

  public void setVoltage(double voltage) {
    currentVoltage = voltage;
  }

  public double getVoltage() {
    return currentVoltage;
  }

  public void setVoltage() {
    m_RightMotorDriver.setVoltage(currentVoltage);
  }
/* maintainElevatorPosition() - Runs motor to maintain position as recorded by the internal encoder */
  public void maintainElevatorPosition() {
    double position = m_RightMotorDriver.getPosition().getValueAsDouble();
    if (position < 0.05) position = 0.05; // prevents the elevator from trying to go further down than possible. slightly more than zero to keep a safe position even if the relative encoder slips
    m_RightMotorDriver.setControl(m_mmReq.withPosition(position).withSlot(0));
  }

  ///////////////////////////////////////////    Debug Methods    ///////////////////////////////////////////
  private void debugElevatorHeightSmartDashboard() {
    double currentInches;
    if (currentLevel == 1) {
      currentInches = Constants.Elevator.l1Height;
    }
    else if(currentLevel == 2){
      currentInches = Constants.Elevator.l2Height;
    }
    else if(currentLevel == 3){
      currentInches = Constants.Elevator.l3Height;
    }
    else {
      currentInches = Constants.Elevator.l4Height;
    }
    SmartDashboard.putNumber("Actual Rotations", m_RightMotorDriver.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Rotations", inchesToRotations(currentInches));
    SmartDashboard.putNumber("Actual Inches", m_RightMotorDriver.getPosition().getValueAsDouble() * Constants.Elevator.inchesPerRot);
    SmartDashboard.putNumber("CurrentInches", currentInches);
  }

  public void debugElevatorVoltageSmartDashboard() {
    SmartDashboard.putNumber("Current Voltage: ", currentVoltage);
    SmartDashboard.putNumber("Rotations per Second",m_RightMotorDriver.getVelocity().getValueAsDouble());
    
  }
}
 
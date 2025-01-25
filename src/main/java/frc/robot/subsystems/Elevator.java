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

public class Elevator extends SubsystemBase {
  private TalonFX m_LeftMotorFollower = new TalonFX(14);   // TODO: declare in constants
  private TalonFX m_RightMotorDriver = new TalonFX(15);    // TODO: declare in constants

  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0); 

  private final double inchesPerRot = Math.PI * 1.88900 * (1.0/12.0) * 2; // pi*sproket*gear ratio*doubled for pulley

  private int currentLevel = 1; // Default level if setLevel(void) is called without previously calling setLevel(int level)

  //////////////////////////////////////////   Constants for Elevator Target Heights   //////////////////////////////////////////
  // TODO: find actual values                                                                                                  //
  // elevatorBaseHeight is the distance from the base of the elevator carridge to the floor in inches                          //
  private final double elevatorBaseHeight = 0;                                                                                 //
  //                                                                                                                           //
  // the constant value in each variable below is the height the base of the carridge should go to for that level.             //
  // elevatorBaseHeight is subtracted to find the actual distance the elevator must travel. only edit the constant !!          //
  private final double l1Height = 10 - elevatorBaseHeight;                                                                     //
  private final double l2Height = 35 - elevatorBaseHeight;                                                                     //
  private final double l3Height = 40 - elevatorBaseHeight;                                                                     //
  private final double l4Height = 45 - elevatorBaseHeight;                                                                     //
  //                                                                                                                           //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////////     Constructor    //////////////////////////////////////////////////////// 
  public Elevator() {
    m_LeftMotorFollower.setControl(new Follower(m_RightMotorDriver.getDeviceID(), false));

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1.0; // gear ratio calculated with inchesToRotations()

    MotionMagicConfigs mm = cfg.MotionMagic;
    // TODO: move magnitudes to constants
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(56))                // 56 output shaft rotations per second cruise TODO: Find higher max speed
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1568))       // TODO: Find appropriate value that isnt a random high number
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(5000));  // TODO: Find appropriate value that isnt a random high number

    Slot0Configs slot0 = cfg.Slot0;
    // TODO: move to constants
    slot0.kS = 0.25; // TODO: tune PID
    slot0.kV = 0.08; // found with online calculation
    slot0.kA = 0.00; // 
    slot0.kP = 10;   // 
    slot0.kI = 0;    // No output for integrated error
    slot0.kD = 0.0;  // 

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
          null,        // Use default ramp rate (1 V/s)
          Volts.of(2), // Super low step voltage because im scared of the elevator
          null,          // Use default timeout (10 s)
                                 // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("state", state.toString())
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
  }

  /* setLevel(int level) - Sets level that the elevator will try to target. Does not control the motor directly, 
     needs setLevel(void) to drive the motor */
  public void setLevel(int level){
    if (level > 0 && level < 5) currentLevel = level;
  }

  /* setLevel(void) - Use motion magic to target position set as currentLevel. */
  public void setLevel(){
    // negative rotations will make the elevator carridge travel up. 
    // positive rotations will make the elevator carridge travel down.
    switch(currentLevel){
      case 1:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(-1 * inchesToRotations(l1Height)).withSlot(0));
        break;
      case 2:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(-1 * inchesToRotations(l2Height)).withSlot(0));
        break;
      case 3:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(-1 * inchesToRotations(l3Height)).withSlot(0));
        break;
      case 4:
        m_RightMotorDriver.setControl(m_mmReq.withPosition(-1 * inchesToRotations(l4Height)).withSlot(0));
        break;
    }
  }

  /* inchesToRotations(double inches) Converts inches to rotations of motor shaft for elevator. */
  public double inchesToRotations(double inches) {
    return  inches / inchesPerRot;
  }

  /* setManualSpeed(double m_speed) - Sets speed without using PID controls. */
  public void setManualSpeed(double m_speed) {
    m_RightMotorDriver.set(m_speed);
  }

  ///////////////////////////////////////////    Debug Methods    ///////////////////////////////////////////
  private void debugElevatorHeightSmartDashboard() {
    double currentInches;
    if (currentLevel == 1) {
      currentInches = l1Height;
    }
    else if(currentLevel == 2){
      currentInches = l2Height;
    }
    else if(currentLevel == 3){
      currentInches = l3Height;
    }
    else {
      currentInches = l4Height;
    }
    SmartDashboard.putNumber("Right Motor Encoder", m_RightMotorDriver.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Rotations", inchesToRotations(currentInches));
    SmartDashboard.putNumber("Inches", currentInches);
  }
}
 
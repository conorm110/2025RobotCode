package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class Manipulator extends SubsystemBase {
    private SparkMax shooterMotor = new SparkMax(Constants.Manipulator.shooterMotorID, MotorType.kBrushless);
    private SparkMax algaeArmMotor = new SparkMax(Constants.Manipulator.algaeMotorID, MotorType.kBrushless);
    // algaeFlywheelMotor
    // revthroughbore encoder
    private LaserCan laserDistance = new LaserCan(Constants.Manipulator.laserCanID);

    private double algaeDistanceThreshold = 30;

    public Manipulator() {
        try {
            laserDistance.setRangingMode(LaserCan.RangingMode.SHORT);
            laserDistance.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserDistance.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    // spin the motor slow for intake
    public void spinIntakeMotor() {
        shooterMotor.set(0.3);
    }

    // spin the motor fast for shooting
    public void spinOuttakeMotor() {
        shooterMotor.set(0.6);
    }

    // stop motor from spinning
    public void stopShooterMotor() {
        shooterMotor.stopMotor();
    }

    // return 1 if there is algae, 0 if there is not algae, and -1 if there is a sensor error
    public int algaeChecker(){
        LaserCan.Measurement measurement = laserDistance.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < algaeDistanceThreshold){
                return 1;  
            } else {
                return 0;
            }
        }
        else {
            return -1;
        }
    }

}
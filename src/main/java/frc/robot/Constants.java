package frc.robot;

public class Constants {
    public static final class Swerve {
        // steer motor gains
        public static final double steerKP = 100.0;
        public static final double steerKI = 0.000;
        public static final double steerKD = 0.500;
        public static final double steerKS = 0.100;
        public static final double steerKV = 2.660;
        public static final double steerKA = 0.000;

        // drive motor gains
        public static final double driveKP = 0.100;
        public static final double driveKI = 0.000;
        public static final double driveKD = 0.000;
        public static final double driveKS = 0.100;
        public static final double driveKV = 0.124;
    }

    public static final class Elevator {
        public static final int leftMotorID = 14;
        public static final int rightMotorID = 15;

        public static final int elevatorMode = 0;  // 0 - Standard Mode;                     1 - Debugging with Set Speed Mode
                                                   // 2 - Debugging with Set Voltage Mode    3 - SysID Mode
                                                   // defaults to standard mode

        public static final double inchesPerRot = Math.PI * 1.88900 * (1.0/12.0) * 2; // pi * sproket * gear ratio * doubled for pulley

        public static final double elevatorkS = 0.45; // DECREASE IF OSCILLATING AND NOTHING ELSE WORKS - CHANGES SIGN FOR CLOSED LOOP AND MIGHT BE CAUSING OVEROSCILATION WITH LOW LOAD found by determining the output necessary to overcome static friction
        public static final double elevatorkV = 0.00; // keep at zero
        public static final double elevatorkA = 0.00; // INCREASE IF ODD ACCELERATION BEHAVIOR
        public static final double elevatorkG = 0.15; // found by determining the output necessary to hold at constant height
        public static final double elevatorkP = 1.90; // 
        public static final double elevatorkI = 0.00; // No output for integrated error
        public static final double elevatorkD = 0.00; //

        public static final double mm_cruiseVelocity = 80; // rotations per second
        public static final double mm_acceleration = 160;  // rotations per second, per second
        public static final double mm_jerk = 1600;         // rotations per second, per second, per second 

        public static final double sysID_rampRate = 0.5;      // volts per second
        public static final double sysID_stepVoltage = 1.2;   // volts

        // elevatorBaseHeight is the distance from the base of the elevator carridge to the floor in inches
        public static final double elevatorBaseHeight = 0;
  
        // the constant value in each variable below is the height the base of the carridge should go to for that level.
        // elevatorBaseHeight is subtracted to find the actual distance the elevator must travel. only edit the constant !!
        public static final double l1Height = 10 - elevatorBaseHeight;
        public static final double l2Height = 35 - elevatorBaseHeight;
        public static final double l3Height = 40 - elevatorBaseHeight;
        public static final double l4Height = 45 - elevatorBaseHeight;
    }
}

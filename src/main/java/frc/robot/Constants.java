package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    /*NEED CONFIG FOR PRACTICE BOT */
    public static final double kGearRatio = 10.71;
    public static final double kSensorGearRatio = 1; //educated guess
    public static final double kWheelDiameterInches = 6; 
    public static final double kTrackWidthInches = 23.1875;
    public static final double kMassPounds = 65; //very rough approximation of comp bot
    public static final double kJKgMetersSquared = 2.1; //Moment of Interia of 2.1 kg m^2 of drivetrain about its center
    public static final double k100msPerSecond = 10; //arbitrary
    
    public static final double kMagEncoderUnitsPerRev = 4096; //sensor units per revolution for the CTRE Mag Encoder
    public static final double kNEOPulsesPerRev = 42;

    
    public static enum ControllerScaling {LIMITED_POLYNOMIC, LINEAR, SQUARED, CUBIC};
    public static enum DriveControllerMode {LEFT_STICK, TRIGGER_ACCEL};
    public static enum DriveType {ARCADE, REG_CURVATURE, SEMI_CURVATURE};
    public static enum ArmControlType {POSITION, VELOCITY, CARTESIAN};


    public static final class DriveConstants {
        public static final int kLeftLeaderPort = 1;
        public static final int kLeftFollowerPort = 2;
        public static final int kRightLeaderPort = 3;
        public static final int kRightFollowerPort = 4; 

        public static final double kTurnGain = 2.0;
        public static final double kDeadband = 0.199413;
        public static final double kDriveGain = 2.0;
        public static final double kRampRate = 4.023;
        public static final double kSlewRateLimiter = 0.8; // units per second

        public static final double kSpeedOutputModifier = 6.0; //arbitrary
        public static final double kRotationOutputModifier = 0.6; //arbitrary

    }

    public static final class AutoConstants {
        public static final double kAutoForwardP = 1;
        public static final double kAutoForwardI = 0;
        public static final double kAutoForwardD = 0;
        public static final double kAutoForwardConstraints = 0;
        public static final double kAutoForwardPTol = 1; //arbirary
        public static final double kAutoForwardVTol = 1; //arbirary

        public static final double kSecUntilAutoCharge = 10;
        
    }

    public static final class ArmConstants{
        
        public static final double kJ1GearRatio = 10;
        public static final double kJ2GearRatio = 210;
        public static final double kJ3GearRatio = 45;
        public static final double kJ4GearRatio = 5;
        public static final double kJ5GearRatio = 10;
        
        //Position needed at startup
        public static final double kJ1AngleInit = 4;
        public static final double kJ2AngleInit = 4;
        public static final double kJ3AngleInit = 4;
        public static final double kJ4AngleInit = 4;
        public static final double kJ5AngleInit = 4;

        //Assume max & min oriented using unit circle (clockwise) 
        public static final double kJ1AngleMax = 4; //Total range of 250 deg, reduced by 5 deg for safety
        public static final double kJ2AngleMax = 4; //Total range of 216 deg, reduced by 5 deg for safety
        public static final double kJ3AngleMax = 4; //Total range of 180 deg, reduced by 5 deg for safety
        public static final double kJ4AngleMax = 4; //Total range of 180 deg, reduced by 5 deg for safety
        public static final double kJ5AngleMax = 4; //?? 

        public static final double kJ1AngleMin = 4;
        public static final double kJ2AngleMin = 4;
        public static final double kJ3AngleMin = 4;
        public static final double kJ4AngleMin = 4;
        public static final double kJ5AngleMin = 4;

        public static final double kJ1DegPerSecMax = 4;
        public static final double kJ2DegPerSecMax = 4;
        public static final double kJ3DegPerSecMax = 4;
        public static final double kJ4DegPerSecMax = 4;

        public static final double armSlewRateLimiter = 0.0009;
        public static final double controllerDeadzone = 0.1;

        
    }


    // public static final class TurretConstants {
    //     public static final double kTurnP = 1;
    //     public static final double kTurnI = 0;
    //     public static final double kTurnD = 0;

    //     public static final double kMaxTurnRateDegPerS = 100;
    //     public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    //     public static final double kTurnToleranceDeg = 5;
    //     public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    // }

    // public static final class VerticalWristConstants {
    //     public static final double kTurnP = 1;
    //     public static final double kTurnI = 0;
    //     public static final double kTurnD = 0;

    //     public static final double kMaxTurnRateDegPerS = 100;
    //     public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    //     public static final double kTurnToleranceDeg = 5;
    //     public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    // }

    // public static final class HorizontalWristConstants {
    //     public static final double kTurnP = 1;
    //     public static final double kTurnI = 0;
    //     public static final double kTurnD = 0;

    //     public static final double kMaxTurnRateDegPerS = 100;
    //     public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    //     public static final double kTurnToleranceDeg = 5;
    //     public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    // }

    // // Needs to be updated from rotation to open
    // public static final class ClawConstants {
    //     public static final double kTurnP = 1;
    //     public static final double kTurnI = 0;
    //     public static final double kTurnD = 0;

    //     public static final double kMaxTurnRateDegPerS = 100;
    //     public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    //     public static final double kTurnToleranceDeg = 5;
    //     public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    // }
    
    public static final class FieldConstants {
        public static final double kchargeStationLengthMeters = Units.inchesToMeters(48);
    }
}


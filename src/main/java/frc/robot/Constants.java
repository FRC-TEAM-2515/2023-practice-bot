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
        
        public static final double kJ1GearRatio = 1;
        public static final double kJ2GearRatio = 210;
        public static final double kJ3GearRatio = 45;
        public static final double kJ4GearRatio = 1;
        
        //Position in rotations
        public static final double kJ1PositionInit = 0.48; //172.8 deg
        public static final double kJ2PositionInit = 0.08; //28.8 deg
        public static final double kJ3PositionInit = 0.16; //57.6
        public static final double kJ4PositionInit = 0.50; //180 

        //8 degree margin of error
        public static final double kJ1PaddingAddRot8 = 0.27; //=96.4 
        public static final double kJ2PaddingAddRot8 = 0.72; //=260
        public static final double kJ3PaddingAddRot8 = 0.73; //=263.6 
        public static final double kJ4PaddingAddRot8 = 0.22; //=79.8 
        public static final double[] kPaddingAddRot8Array = new double[] {kJ1PaddingAddRot8, kJ2PaddingAddRot8, kJ3PaddingAddRot8, kJ4PaddingAddRot8};

        //8 degree margin of error
        public static final double kJ1PaddingMinusRot8 = 0.54; //=195.2 
        public static final double kJ2PaddingMinusRot8 = 0.30; //=107.2 
        public static final double kJ3PaddingMinusRot8 = 0.15; //=53.2 
        public static final double kJ4PaddingMinusRot8 = 0.09; //=31.8 
        public static final double[] kPaddingMinusRot8Array = new double[] {kJ1PaddingMinusRot8, kJ2PaddingMinusRot8, kJ3PaddingMinusRot8, kJ4PaddingMinusRot8};

        /*Bounds for the absolute maximum joint range of motion
        Reported in rotations for later scaling functions
        'Minus' indicates that the value will decrease to reducing maximum range
        'Add' indicates that the value will increase to reduce maximum range*/

        //Maximum range of 277.2 degress 
        public static final double kJ1PaddingMinusRot = 0.29; //104.4 degrees
        public static final double kJ1PaddingAddRot = 0.52; //187.2 degrees

        //Maximum range of 223.2 degrees
        public static final double kJ2PaddingMinusRot = 0.32; //115.2 degrees
        public static final double kJ2PaddingAddRot = 0.70; //252.0 degrees

        //Maximum range of 165.6 degrees
        public static final double kJ3PaddingMinusRot = 0.17; //61.2 degrees
        public static final double kJ3PaddingAddRot = 0.71; //255.6 degrees

        //Maximum range of 54.0 degrees - no discontinuity so subtracted by an extra 360 degrees to get the correct range
        public static final double kJ4PaddingMinusRot = 0.23; //82.8 degrees
        public static final double kJ4PaddingAddRot = 0.08; //28.8 degrees

        public static final double kJ1MaxLeftXControllerDeg = -75.6;
        public static final double kJ1MaxRightXControllerDeg = 187;

        public static final double kJ2MaxLeftXControllerDeg = -72;
        public static final double kJ2MaxRightXControllerDeg = 115.27;
        
        public static final double kJ3MaxLeftXControllerDeg = -75;
        public static final double kJ3MaxRightXControllerDeg = 61.2;
        
        public static final double[] kJointMaxLeftStickDeg = new double[] {kJ1MaxLeftXControllerDeg, kJ1MaxLeftXControllerDeg, kJ1MaxLeftXControllerDeg};
        public static final double[] kJointMaxRightStickDeg = new double[] {kJ1MaxRightXControllerDeg, kJ1MaxRightXControllerDeg, kJ1MaxRightXControllerDeg, kJ1MaxRightXControllerDeg};


        public static final double kJ1DegPerSecMax = 4;
        public static final double kJ2DegPerSecMax = 4;
        public static final double kJ3DegPerSecMax = 4;
        public static final double kJ4DegPerSecMax = 4;

        public static final double armSlewRateLimiter = 0.0009;
        public static final double controllerDeadzone = 0.2;
        public static double kJ2MaxRightStickDeg;


        
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


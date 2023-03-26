package frc.robot.util;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.util.Units;

public class RobotMath {

    public static double magEncoderConvertMeters(double rawPosition) { // raw sensor units -> meters
        return (rawPosition/ Constants.kMagEncoderUnitsPerRev * (Math.PI * Units.inchesToMeters(Constants.kWheelDiameterInches)) / Constants.kGearRatio);
    }

    public static double magEncoderConvertFeet(double rawPosition) { // raw sensor units -> feet
        return Units.metersToFeet(magEncoderConvertMetersPerSec(rawPosition));
    }

    public static double magEncoderConvertMetersPerSec(double rawSpeed) { // raw sensor units per 100 ms -> meters per sec
        return (magEncoderConvertMeters(rawSpeed) / 1000);
    }

    public static double magEncoderConvertMilesPerHour(double rawSpeed) { // raw sensor units per 100 ms -> miles per hour (for sense of scale & also just for fun)
        return (Units.metersToFeet(magEncoderConvertMetersPerSec(rawSpeed)) / 5680 * 60);
    }

    public static double magEncoderConvertDegrees(double rawPosition) { // raw sensor units -> degrees 
        return ((rawPosition % Constants.kMagEncoderUnitsPerRev) / Constants.kMagEncoderUnitsPerRev / Constants.kGearRatio * 360); // modulo to stop each rotation adding another full rotation worth of ticks
    }
    
    public static double j1EncoderConvertDegrees(double rawPosition) { // raw rotation -> degrees 
        return (rawPosition / Constants.kMagEncoderUnitsPerRev / Constants.ArmConstants.kJ1GearRatio * 360);
    }
    
    public static double j2EncoderConvertDegrees(double rawPosition) { // raw rotation -> degrees 
        return (rawPosition / Constants.kMagEncoderUnitsPerRev / Constants.ArmConstants.kJ2GearRatio * 360); 
    }
    
    public static double j3EncoderConvertDegrees(double rawPosition) {  // raw rotation -> degrees
        return (rawPosition  / Constants.kNEOPulsesPerRev / Constants.ArmConstants.kJ3GearRatio * 360);
    }
    
    public static double j4EncoderConvertDegrees(double rawPosition) {  // raw rotation -> degrees
        return (rawPosition / Constants.kMagEncoderUnitsPerRev / Constants.ArmConstants.kJ4GearRatio * 360);
    }
    
    public static double j5EncoderConvertDegrees(double rawPosition) {  // raw rotation -> degrees
        return (rawPosition / Constants.kMagEncoderUnitsPerRev / Constants.ArmConstants.kJ5GearRatio * 360);
    }



}

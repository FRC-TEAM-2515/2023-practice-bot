package frc.robot.util;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.util.Units;

public class RobotMath {

    public static double truncate(double n, int decimalplace) {
        n = n * Math.pow(10, decimalplace);
        n = Math.floor(n);
        n = n / Math.pow(10, decimalplace);
        return n;
    }

    public static double magEncoderConvertMeters(double rawPosition) { // raw sensor units -> meters
        return truncate((rawPosition / Constants.kMagEncoderUnitsPerRev
                * (Math.PI * Units.inchesToMeters(Constants.kWheelDiameterInches)) / Constants.kGearRatio), 3);
    }

    public static double magEncoderConvertFeet(double rawPosition) { // raw sensor units -> feet
        return truncate(Units.metersToFeet(magEncoderConvertMetersPerSec(rawPosition)), 3);
    }

    public static double magEncoderConvertMetersPerSec(double rawSpeed) { // raw sensor units per 100 ms -> meters per
                                                                          // sec
        return truncate((magEncoderConvertMeters(rawSpeed) / 1000), 3);
    }

    public static double magEncoderConvertMilesPerHour(double rawSpeed) { // raw sensor units per 100 ms -> miles per
                                                                          // hour (for sense of scale & also just for
                                                                          // fun)
        return truncate((Units.metersToFeet(magEncoderConvertMetersPerSec(rawSpeed)) / 5680 * 60),3);
    }

    public static double magEncoderConvertDegrees(double rawPosition) { // raw sensor units -> degrees
        return truncate(((rawPosition % Constants.kMagEncoderUnitsPerRev) / Constants.kMagEncoderUnitsPerRev
                / Constants.kGearRatio * 360), 3); // modulo to stop each rotation adding another full rotation worth of
                                               // ticks
    }

    public static double armEncoderConvertDegrees(double rawRotation) { // rotations -> degrees
        return rawRotation * 360;
    }

    public static double armDegreesConvertEncoder(double rawDegrees) { // degrees -> rotations
        return rawDegrees / 360;
    }
    
    public static double armRangePaddingAddDeg(double bookendAdd, double padding) {
        return armEncoderConvertDegrees(bookendAdd) + padding;
    }

    public static double armRangePaddingMinusDeg(double bookendSub, double padding) {
        return armEncoderConvertDegrees(bookendSub) - padding;
    }
    
    public static double armRangePaddingAddEncoder(double bookendAdd, double padding){
        return armDegreesConvertEncoder(armRangePaddingAddDeg(bookendAdd, padding));
    }

    public static double armRangePaddingMinusEncoder(double bookendSub, double padding){
        return armDegreesConvertEncoder(armRangePaddingAddDeg(bookendSub, padding));
    }

    public static double armRangeDegrees(double padding){
       return Math.abs(armRangePaddingAddDeg(ArmConstants.kJ2BookendAdd, padding) - armRangePaddingMinusDeg(ArmConstants.kJ2BookendSub, padding) - 360);
    }
   

}

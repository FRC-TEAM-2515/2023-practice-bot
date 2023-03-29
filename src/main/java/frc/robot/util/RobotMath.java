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

    public static double armEncoderConvertDegrees(double rawRotation) {
        return rawRotation * 360;
    }

    public static double armDegreesConvertEncoder(double rawDegrees) {
        return rawDegrees / 360;
    }

    // public static double armRangeErrorMarginAdd(double bookendAdd, double error) {
    //     return armDegreesConvertEncoder((armEncoderConvertDegrees(bookendAdd) + error));
    // }

    
    public static double armRangeErrorMarginAddD(double bookendAdd, double error) {
        return armEncoderConvertDegrees(bookendAdd) + error;
    }


    // public static double armRangeErrorMarginSub(double bookendSub, double error) {
    //     return armDegreesConvertEncoder((armEncoderConvertDegrees(bookendSub) - error));
    // }

    public static double armRangeDegrees(double padding, double jointPaddingAdd, double jointPaddingMinus){
       return Math.abs(armRangePaddingAddDeg(jointPaddingAdd, padding) - armRangePaddingMinusDeg(jointPaddingMinus, padding) - 360);
    }

}

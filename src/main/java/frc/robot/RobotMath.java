package frc.robot;

import frc.robot.Constants.DriveConstants;
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
}
 //Forward Kinematic 3dof Matrix
// [cos(t1 + t2)*cos(t3), -1.0*cos(t1 + t2)*sin(t3),      sin(t1 + t2),      d3*sin(t1 + t2)]
// [sin(t1 + t2)*cos(t3), -1.0*sin(t1 + t2)*sin(t3), -1.0*cos(t1 + t2), -1.0*d3*cos(t1 + t2)]
// [             sin(t3),                   cos(t3),                 0,              d1 + d2]
// [                   0,                         0,                 0,                  1.0]
 

 //Forward Kinematic 4dof Matrix

//  [- 1.0*sin(t1)*sin(t4) - cos(t4)*(1.0*cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)), 1.0*sin(t4)*(1.0*cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) - 1.0*cos(t4)*sin(t1), sin(t2 + t3)*cos(t1), d4*(cos(t1)*cos(t2)*sin(t3) + 1.0*cos(t1)*cos(t3)*sin(t2)) - 1.0*d2*sin(t1) - 1.0*d3*sin(t1)]
// [  1.0*cos(t1)*sin(t4) - cos(t4)*(1.0*sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)), 1.0*cos(t1)*cos(t4) + 1.0*sin(t4)*(1.0*sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)), sin(t2 + t3)*sin(t1), d4*(cos(t2)*sin(t1)*sin(t3) + 1.0*cos(t3)*sin(t1)*sin(t2)) + 1.0*d2*cos(t1) + 1.0*d3*cos(t1)]
// [                                                              -1.0*sin(t2 + t3)*cos(t4),                                                                      sin(t2 + t3)*sin(t4),         cos(t2 + t3),                                                                         d1 + d4*cos(t2 + t3)]
// [                                                                                      0,                                                                                         0,                    0,                                                                                          1.0]
 
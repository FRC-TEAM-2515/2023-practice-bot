package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMath.*;

public class OIReporters {

        public void updateOIReporters() {
               configDriveValuesReporters();
               configManualDriveReporters();
        }


        public static class DriveReporters {

                // Preferences
                public static String driveControllerMode = "";
                public static double lStickSpeed = 0;
                public static double tAccelSpeed = 0;

                public static String scalingMode = "";
                public static double originalSpeed = 0;
                public static double originalRotation = 0;
                public static String linearScaled = "";
                public static String squaredScaled = "";
                public static String cubicScaled = "";
                public static String fancyScaled = "";

                public static String driveType = "";
                public static boolean semiCurvature = false;

                // Buttons
                public static boolean brakesEnabled = false;
                public static double inversionMult = 0;
                public static boolean inverted = false;

                //Gyro
                public static double gyroYaw = 0;
                public static double gyroAngle = 0;
                public static double gyroRoll = 0;
                public static double gryoPitch = 0;

                public static boolean gyroConnected = false;
                public static boolean gyroCalibrating = false;
                public static boolean gyroMoving = false;

                // Odometry 
                public static double leftSpeedFromCommand = 0;
                public static double rightSpeedFromCommand = 0;

                public static double leftEncoderSpeed = 0;
                public static double rightEncoderSpeed = 0;
                public static double leftEncoderDistance = 0;
                public static double rightEncoderDistance = 0;

        }

        public void configManualDriveReporters() {

                /* SmartDashboard */

                // Drive Controller
                SmartDashboard.putString("DControl Mode", DriveReporters.driveControllerMode);
                SmartDashboard.putNumber("LStick Speed", DriveReporters.lStickSpeed);
                SmartDashboard.putNumber("TAccel Speed", DriveReporters.tAccelSpeed);

                // Scaling
                SmartDashboard.putString("Scaling", DriveReporters.scalingMode);
                SmartDashboard.putString("Linear", DriveReporters.linearScaled);
                SmartDashboard.putString("Squared", DriveReporters.squaredScaled);
                SmartDashboard.putString("Cubic", DriveReporters.cubicScaled);
                SmartDashboard.putString("Fancy", DriveReporters.fancyScaled);

                SmartDashboard.putNumber("Unscaled Speed", DriveReporters.originalSpeed);
                SmartDashboard.putNumber("Unscaled Rotation", DriveReporters.originalRotation);

                // Drive type
                SmartDashboard.putString("Drive Type Chosen", DriveReporters.driveType);
                SmartDashboard.putBoolean("Semi Curvature", DriveReporters.semiCurvature);

                /* Controller */

                SmartDashboard.putBoolean("Enabled Brakes", DriveReporters.brakesEnabled);
                SmartDashboard.putNumber("Inversion Multiplier", DriveReporters.inversionMult);
                SmartDashboard.putBoolean("Inverted", isInverted());
        }

        public void configDriveValuesReporters() {

                // Gyro
                SmartDashboard.putNumber("imu-yaw", DriveReporters.gyroYaw);
                SmartDashboard.putNumber("imu-pitch", DriveReporters.gryoPitch);
                SmartDashboard.putNumber("imu-roll", DriveReporters.gyroRoll);
                SmartDashboard.putNumber("imu-angle", DriveReporters.gyroAngle);

                SmartDashboard.putBoolean("imu-moving", DriveReporters.gyroMoving);
                SmartDashboard.putBoolean("imu-connected", DriveReporters.gyroConnected);
                SmartDashboard.putBoolean("imu-calibrating", DriveReporters.gyroCalibrating);

                // Raw Encoder Values
                SmartDashboard.putNumber("Left Speed (raw units 100 ms)", DriveReporters.leftEncoderSpeed);
                SmartDashboard.putNumber("Right Speed (raw units 100 ms)", DriveReporters.rightEncoderSpeed);
                SmartDashboard.putNumber("Left Distance (raw units)", DriveReporters.leftEncoderDistance);
                SmartDashboard.putNumber("Right Distance (raw units)", DriveReporters.rightEncoderDistance);

                SmartDashboard.putNumber("Left Speed from DriveCommand", DriveReporters.leftSpeedFromCommand);
                SmartDashboard.putNumber("Right Speed from DriveCommand", DriveReporters.rightSpeedFromCommand);
                
                // Converted Encoder Values
                SmartDashboard.putNumber("Left Speed (m s)", RobotMath.magEncoderConvertMetersPerSec(DriveReporters.leftEncoderSpeed));
                SmartDashboard.putNumber("Right Speed (m s) ", RobotMath.magEncoderConvertMetersPerSec(DriveReporters.rightEncoderSpeed));
                SmartDashboard.putNumber("Left Speed (mi hr) ", RobotMath.magEncoderConvertMilesPerHour(DriveReporters.leftEncoderSpeed));
                SmartDashboard.putNumber("Right Speed (mi hr) ", RobotMath.magEncoderConvertMilesPerHour(DriveReporters.rightEncoderSpeed));

                SmartDashboard.putNumber("Left Distance (m)", RobotMath.magEncoderConvertMeters(DriveReporters.leftEncoderDistance));
                SmartDashboard.putNumber("Right Distance (m)", RobotMath.magEncoderConvertMeters(DriveReporters.rightEncoderDistance));
                SmartDashboard.putNumber("Left Distance (ft)", RobotMath.magEncoderConvertFeet(DriveReporters.leftEncoderDistance));
                SmartDashboard.putNumber("Right Distance (ft)", RobotMath.magEncoderConvertFeet(DriveReporters.rightEncoderDistance));
        }

        public boolean isInverted() {
                if (DriveReporters.inversionMult == -1) {
                        return true;
                } else {
                        return false;
                }
        }
}

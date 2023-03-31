package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.RobotMath;
import frc.robot.util.RobotMath.*;

public class OIReporters {

        public void updateOIReporters() {
               configDriveValuesReporters();
               configManualDriveReporters();
        }
        public static enum ControllerScaling {LIMITED_POLYNOMIC, LINEAR, SQUARED, CUBIC};
        public static enum DriveControllerMode {LEFT_STICK, TRIGGER_ACCEL};
        public static enum DriveType {ARCADE, REG_CURVATURE, SEMI_CURVATURE};
        public static enum ArmControlType {POSITION, VELOCITY, CARTESIAN};
        public static enum AutoCommand {SIMPLE};
        public static enum ArmBrake {BRAKE, COAST};

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
                public static double gyroPitch = 0;

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
                SmartDashboard.putNumber("LStick Speed", RobotMath.truncate(DriveReporters.lStickSpeed, 3));
                SmartDashboard.putNumber("TAccel Speed", RobotMath.truncate(DriveReporters.tAccelSpeed, 3));

                // Scaling
                SmartDashboard.putString("Scaling", DriveReporters.scalingMode);
                SmartDashboard.putString("Linear", DriveReporters.linearScaled);
                SmartDashboard.putString("Squared", DriveReporters.squaredScaled);
                SmartDashboard.putString("Cubic", DriveReporters.cubicScaled);
                SmartDashboard.putString("Fancy", DriveReporters.fancyScaled);

                SmartDashboard.putNumber("Unscaled Speed", RobotMath.truncate(DriveReporters.originalSpeed, 3));
                SmartDashboard.putNumber("Unscaled Rotation", RobotMath.truncate(DriveReporters.originalRotation, 3));

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
                SmartDashboard.putNumber("imu-pitch", DriveReporters.gyroPitch);
                SmartDashboard.putNumber("imu-roll", DriveReporters.gyroRoll);
                SmartDashboard.putNumber("imu-angle", DriveReporters.gyroAngle);

                SmartDashboard.putBoolean("imu-moving", DriveReporters.gyroMoving);
                SmartDashboard.putBoolean("imu-connected", DriveReporters.gyroConnected);
                SmartDashboard.putBoolean("imu-calibrating", DriveReporters.gyroCalibrating);

                // Raw Encoder Values
                SmartDashboard.putNumber("Left Speed (raw units 100 ms)", RobotMath.truncate(DriveReporters.leftEncoderSpeed, 3));
                SmartDashboard.putNumber("Right Speed (raw units 100 ms)", RobotMath.truncate(DriveReporters.rightEncoderSpeed, 3));
                SmartDashboard.putNumber("Left Distance (raw units)", RobotMath.truncate(DriveReporters.leftEncoderDistance,3 ));
                SmartDashboard.putNumber("Right Distance (raw units)", RobotMath.truncate(DriveReporters.rightEncoderDistance, 3));

                SmartDashboard.putNumber("Left Speed from DriveCommand", RobotMath.truncate(DriveReporters.leftSpeedFromCommand, 3));
                SmartDashboard.putNumber("Right Speed from DriveCommand", RobotMath.truncate(DriveReporters.rightSpeedFromCommand, 3));
                
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

        public static class ArmReporters {
              
                        // Preferences
                        public static String armControlMode = "";
                        public static double j1JoystickCommand = 0;
                        public static double j2JoystickCommand = 0;
                        public static double j3JoystickCommand = 0;
                        public static double j4JoystickCommand = 0;
        
                        public static String scalingMode = "";
                        public static double originalLeftX = 0;
                        public static double originalLeftY = 0;
                        public static String linearScaledX = "";
                        public static String squaredScaledX = "";
                        public static String cubicScaledX = "";
                        public static String fancyScaledX = "";
                        public static String linearScaledY = "";
                        public static String squaredScaledY = "";
                        public static String cubicScaledY = "";
                        public static String fancyScaledY = "";

                        public static double j1LeftX = 0;
                        public static double leftX = 0;
                        public static double j2LeftY = 0;
                        public static double j3RightY = 0;

                        public static double rightY = 10;
                        public static double leftY = 10;
                        
                        public static boolean kCoast = false;
                        public static double padding = 0;
                        private static double j1, j2, j3, j4;
                        public static double[] ControllerScaled = {j1, j2, j3, j4};
                        public static double j1Raw, j2Raw, j3Raw, j4Raw;
                        public static double[] ControllerRaw = {j1Raw, j2Raw, j3Raw, j4Raw};
        }
        

        public void manualArmReporters(){

                // Scaling
                SmartDashboard.putString("Arm Scaling", ArmReporters.scalingMode);
                SmartDashboard.putString("Arm LinearX", ArmReporters.linearScaledX);
                SmartDashboard.putString("Arm SquaredX", ArmReporters.squaredScaledX);
                SmartDashboard.putString("Arm CubicX", ArmReporters.cubicScaledX);
                SmartDashboard.putString("Arm FancyX", ArmReporters.fancyScaledX);

                SmartDashboard.putString("Arm LinearY", ArmReporters.linearScaledY);
                SmartDashboard.putString("Arm SquaredY", ArmReporters.squaredScaledY);
                SmartDashboard.putString("Arm CubicY", ArmReporters.cubicScaledY);
                SmartDashboard.putString("Arm FancyY", ArmReporters.fancyScaledY);

                // Arm Control Mode
                SmartDashboard.putString("Arm Control Mode", ArmReporters.armControlMode);
                SmartDashboard.putNumber("J1 Joystick Command", RobotMath.truncate(ArmReporters.j1JoystickCommand, 3));
                SmartDashboard.putNumber("J2 Joystick Command", RobotMath.truncate(ArmReporters.j2JoystickCommand, 3));
                SmartDashboard.putNumber("J3 Joystick Command", RobotMath.truncate(ArmReporters.j3JoystickCommand, 3));
                SmartDashboard.putNumber("J4 Joystick Command", RobotMath.truncate(ArmReporters.j4JoystickCommand, 3));

                SmartDashboard.putNumber("ArmTest/j1LeftX", ArmReporters.j1LeftX);
                SmartDashboard.putNumber("ArmTest/j1LeftXRaw", ArmReporters.leftX);
                SmartDashboard.putNumber("ArmTest/j2LeftY", ArmReporters.j2LeftY);
                SmartDashboard.putNumber("ArmTest/j2LeftYRaw", ArmReporters.leftY);
                SmartDashboard.putNumber("ArmTest/j3RightY", ArmReporters.j3RightY);
                SmartDashboard.putNumber("ArmTest/j3RightYRaw", ArmReporters.rightY);
        
                SmartDashboard.getNumber("RoM Safety Error", ArmReporters.padding);
                SmartDashboard.putNumber("RoM Safety Error", ArmReporters.padding);
                
                SmartDashboard.putNumber("ArmTest/j1LeftX", ArmReporters.j1LeftX);
                SmartDashboard.putNumber("ArmTest/j1LeftXRaw", ArmReporters.leftX);
                SmartDashboard.putNumber("ArmTest/j2LeftY", ArmReporters.j2LeftY);
                SmartDashboard.putNumber("ArmTest/j2LeftYRaw", ArmReporters.leftY);
                SmartDashboard.putNumber("ArmTest/j3RightY", ArmReporters.j3RightY);
                SmartDashboard.putNumber("ArmTest/j3RightYRaw", ArmReporters.rightY);

        
                SmartDashboard.putNumber("J1 RoM w/ Safety Margin", RobotMath.armRangeDegrees(ArmReporters.padding, ArmConstants.kJ1PaddingAddRot, ArmConstants.kJ1PaddingMinusRot));
                SmartDashboard.putNumber("J2 RoM w/ Safety Margin", RobotMath.armRangeDegrees(ArmReporters.padding, ArmConstants.kJ2PaddingAddRot, ArmConstants.kJ2PaddingMinusRot));
                SmartDashboard.putNumber("J3 RoM w/ Safety Margin", RobotMath.armRangeDegrees(ArmReporters.padding, ArmConstants.kJ3PaddingAddRot, ArmConstants.kJ3PaddingMinusRot));
                SmartDashboard.putNumber("J4 RoM w/ Safety Margin", RobotMath.armRangeDegrees(ArmReporters.padding, ArmConstants.kJ4PaddingAddRot, ArmConstants.kJ4PaddingMinusRot) - 360);
        
        
 
        }

        
}

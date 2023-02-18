package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OIReporters {

        public static String driveControllerMode = "";
        public static String scalingMode = "";
        public static String driveType = "";

        public static double lStickSpeed = 0;
        public static double tAccelSpeed = 0;

        public static double originalSpeed = 0;
        public static double originalRotation = 0;
        public static String linearScaled = "";
        public static String squaredScaled = "";
        public static String cubicScaled = "";
        public static String fancyScaled = "";

        public static boolean semiCurvature = false;

        public static boolean brakesEnabled = false;
        public static double inversionMult = 0;

        
public void updateReporters() {

        SmartDashboard.putString("DControl Mode", OIReporters.driveControllerMode);
        SmartDashboard.putNumber("LStick Speed", OIReporters.lStickSpeed );
        SmartDashboard.putNumber("TAccel Speed", OIReporters.tAccelSpeed);
      
        SmartDashboard.putNumber("OG Speed", OIReporters.originalSpeed );
        SmartDashboard.putNumber("OG Rotation", OIReporters.originalRotation );
      
        SmartDashboard.putString("Scaling", OIReporters.scalingMode);
        SmartDashboard.putString("Linear", OIReporters.linearScaled);
        SmartDashboard.putString("Squared", OIReporters.squaredScaled);
        SmartDashboard.putString("Cubic", OIReporters.cubicScaled);
        SmartDashboard.putString("Fancy", OIReporters.fancyScaled);
      
        SmartDashboard.putString("Drive Type Chosen", OIReporters.driveType);
        SmartDashboard.putBoolean("Semi Curvature", OIReporters.semiCurvature);
        
        SmartDashboard.putBoolean("Enabled Brakes", OIReporters.brakesEnabled);
        SmartDashboard.putNumber("Inversion Multiplier", OIReporters.inversionMult);

        SmartDashboard.putNumber("imu-yaw", gyro.getYaw());
        SmartDashboard.putNumber("imu-pitch", gyro.getPitch());
        SmartDashboard.putNumber("imu-roll", gyro.getRoll());
        SmartDashboard.putNumber("imu-angle", gyro.getAngle());

        SmartDashboard.putBoolean("imu-moving", gyro.isMoving());
        SmartDashboard.putBoolean("imu-connected", gyro.isConnected());
        SmartDashboard.putBoolean("imu-calibrating", gyro.isCalibrating());

        SmartDashboard.putNumber("Left Velocity", driveLeftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Velocity", driveRightLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Left Distance", driveLeftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Distance", driveRightLeader.getSelectedSensorPosition());
      
      }
}

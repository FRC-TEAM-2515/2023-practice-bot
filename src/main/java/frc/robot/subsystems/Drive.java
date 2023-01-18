// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/**
 *
 */
public class Drive extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX leftLeader;
    private WPI_VictorSPX leftFollower;
    private MotorControllerGroup leftSideDrive;
    private WPI_TalonSRX rightLeader;
    private WPI_VictorSPX rightFollower;
    private MotorControllerGroup rightSideDrive;
    private DifferentialDrive differentialDrive;
    private AHRS ahrs;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    protected XboxController driveController;
    static final double turnGain = DriveConstants.kTurnGain;
    static final double deadband = DriveConstants.kDeadband;
    static final double driveGain = DriveConstants.kDriveGain;
    private boolean curvatureDriveMode = false;
    private boolean rightStickEnabled = false;
    private final DifferentialDriveOdometry odometry;


    /**
    *
    */
    public Drive() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        leftLeader = new WPI_TalonSRX(DriveConstants.kLeftLeaderPort);

        /* Factory default hardware to prevent unexpected behavior */
        leftLeader.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
        leftLeader.setInverted(false);
        leftLeader.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
        leftLeader.configNominalOutputForward(0, 30);
        leftLeader.configNominalOutputReverse(0, 30);
        leftLeader.configPeakOutputForward(1, 30);
        leftLeader.configPeakOutputReverse(-1, 30);

        /* Configure Sensor */
        // Phase sensor to have positive increment when driving Talon Forward (Green
        // LED)
        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        leftLeader.setSensorPhase(false);

        /* Set gains in slot0 - see documentation */
        leftLeader.selectProfileSlot(0, 0);
        leftLeader.config_kF(0, 0.0, 30);
        leftLeader.config_kP(0, 0.0, 30);
        leftLeader.config_kI(0, 0.0, 30);
        leftLeader.config_kD(0, 0.0, 30);

        leftFollower = new WPI_VictorSPX(DriveConstants.kLeftFollowerPort);

        leftSideDrive = new MotorControllerGroup(leftLeader, leftFollower);
        addChild("leftSideDrive", leftSideDrive);

        rightLeader = new WPI_TalonSRX(DriveConstants.kRightLeaderPort);

        /* Factory default hardware to prevent unexpected behavior */
        rightLeader.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
        rightLeader.setInverted(true);
        rightLeader.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
        rightLeader.configNominalOutputForward(0, 30);
        rightLeader.configNominalOutputReverse(0, 30);
        rightLeader.configPeakOutputForward(1, 30);
        rightLeader.configPeakOutputReverse(-1, 30);

        /* Configure Sensor */
        // Phase sensor to have positive increment when driving Talon Forward (Green
        // LED)
        rightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        rightLeader.setSensorPhase(true);

        /* Set gains in slot0 - see documentation */
        rightLeader.selectProfileSlot(0, 0);
        rightLeader.config_kF(0, 0.0, 30);
        rightLeader.config_kP(0, 0.0, 30);
        rightLeader.config_kI(0, 0.0, 30);
        rightLeader.config_kD(0, 0.0, 30);

        rightFollower = new WPI_VictorSPX(DriveConstants.kRightFollowerPort);
        leftFollower.follow(leftLeader);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        rightFollower.follow(rightLeader);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        rightFollower.setInverted(true);


        rightSideDrive = new MotorControllerGroup(rightLeader, rightFollower);
        addChild("rightSideDrive", rightSideDrive);

        differentialDrive = new DifferentialDrive(leftSideDrive, rightSideDrive);
        addChild("DifferentialDrive", differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        try {
            ahrs = new AHRS(Port.kUSB1);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);
        // LiveWindow.addSensor("Drive", "ahrs", ahrs);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS



        odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), driveGain, deadband);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void manualDrive() {
        if (driveController == null) {
            driveController = RobotContainer.getInstance().getDriveController();
        }
        if (curvatureDriveMode) {
            if (rightStickEnabled) {
                differentialDrive.curvatureDrive(applyGain(driveController.getLeftY(), driveGain),
                        applyGain(driveController.getRightX(), turnGain), driveController.getRawButton(10));
            } else {
                differentialDrive.curvatureDrive(applyGain(driveController.getLeftY(), driveGain),
                        applyGain(driveController.getLeftX(), turnGain), driveController.getAButton());
            }
        } else {
            if (rightStickEnabled) {
                differentialDrive.arcadeDrive(applyGain(driveController.getLeftY(), driveGain),
                        applyGain(driveController.getRightX(), turnGain));
            } else {
                differentialDrive.arcadeDrive(applyGain(driveController.getLeftY(), driveGain),
                        applyGain(driveController.getLeftX(), turnGain));
            }
        }
    }

    private double applyGain(double x, double gain) {
        if (x > -deadband && x < deadband) {
            x = 0;
        } else if (x >= deadband) {
            x = x - deadband;
            x = x / (1 - deadband);
            x = Math.pow(x, gain);
            x = -x;
        } else {
            x = x + deadband;
            x = x / (1 - deadband);
            x = Math.pow(x, gain);
        }
        return x;
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("imu-yaw", ahrs.getYaw());
        SmartDashboard.putNumber("imu-pitch", ahrs.getPitch());
        SmartDashboard.putNumber("imu-roll", ahrs.getRoll());
        SmartDashboard.putNumber("imu-angle", ahrs.getAngle());

		SmartDashboard.putBoolean("imu-moving", ahrs.isMoving());
		SmartDashboard.putBoolean("imu-connected", ahrs.isConnected());
		SmartDashboard.putBoolean("imu-calibrating", ahrs.isCalibrating());
		// SmartDashboard.putData("imu", ahrs);

        SmartDashboard.putNumber("Left Velocity", leftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Velocity", rightLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Left Distance", leftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Distance", rightLeader.getSelectedSensorPosition());

    }
   /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // Convert RPMs to Meters per Second
        return new DifferentialDriveWheelSpeeds(
                leftLeader.getSelectedSensorVelocity(),
                rightLeader.getSelectedSensorVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(ahrs.getRotation2d(),leftLeader.getSelectedSensorPosition(),rightLeader.getSelectedSensorPosition(),pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftLeader.getSelectedSensorPosition() + rightLeader.getSelectedSensorPosition() / 2.0);
    }

    // /**
    //  * Gets the left drive encoder.
    //  *
    //  * @return the left drive encoder
    //  */
    // public RelativeEncoder getLeftEncoder() {
    //     return m_driveEncoderLeft1;
    // }

    // /**
    //  * Gets the right drive encoder.
    //  *
    //  * @return the right drive encoder
    //  */
    // public RelativeEncoder getRightEncoder() {
    //     return m_driveEncoderRight1;
    // }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        ahrs.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -ahrs.getRate();
    }

}

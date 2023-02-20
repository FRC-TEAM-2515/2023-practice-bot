package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveForward extends CommandBase{
    
    private double m_autoDistance;
    private double m_autoSpeed = 0.5; //arbitrary
    private double initalRightDistance;
    private double initalLeftDistance;

    private double rightError = 1;
    private double leftError = 1;

    /** Tune these two variables */

    // The value the scale based on error this would be the M in y = mx + b
    private double speedValue = .2;

    // The constant speed to set the motor to this would be b in y = mx + b
    private double speedConstant = .1;

    private Pose2d initPose;

    private ProfiledPIDController autoForwardController;
    private double m_autoForwardGoal; //meters
    private DriveTrain m_driveTrain;

public AutoDriveForward(DriveTrain driveTrain, double autoForwardDistance) {
    this.m_driveTrain = driveTrain;
    this.m_autoForwardGoal = autoForwardDistance;

    addRequirements(driveTrain);

    // autoForwardController = new ProfiledPIDController(AutoConstants.kAutoForwardP,AutoConstants.kAutoForwardI,AutoConstants.kAutoForwardD, AutoConstants.kAutoForwardConstraints);
    // autoForwardController.setTolerance(AutoConstants.kAutoForwardPTol,AutoConstants.kAutoForwardVTol);
}


@Override
public void initialize() {

    
    initalLeftDistance = m_driveTrain.leftMeters();
    initalRightDistance = m_driveTrain.rightMeters();

    initPose = this.m_driveTrain.getPose();
   // autoForwardController.setGoal(this.m_autoForwardGoal);
}

@Override
  public void execute() {
        rightError = (initalRightDistance + m_autoForwardGoal) - m_driveTrain.rightMeters();
        leftError = (initalLeftDistance + m_autoForwardGoal) - m_driveTrain.leftMeters();

        // Speeds to set the motors too
        double rightSpeed = (Math.max((rightError / m_autoForwardGoal), .5) * speedValue) + speedConstant;
        double leftSpeed = (Math.max((leftError / m_autoForwardGoal), .5) * speedValue) + speedConstant;

        m_driveTrain.setWheelSpeedsAuto(leftSpeed, rightSpeed);
        
        //m_driveTrain.arcadeDrive(m_autoSpeed, 0);
  }

public double getDistance() {
    return new Transform2d(initPose, m_driveTrain.getPose()).getTranslation().getX();
}

private double getDisplacement() {
    return this.initPose.getTranslation().getDistance(this.m_driveTrain.getPose().getTranslation());
  }

@Override
public boolean isFinished() {
    return rightError <= 0 && leftError <= 0;

    // if (this.getDisplacement() > m_autoDistance) {
    //     return true;
    // } 
    // return false;
}

@Override 
public void end(boolean interrupt) {
    m_driveTrain.arcadeDrive(0, 0);
    m_driveTrain.stopMotors();
}
}

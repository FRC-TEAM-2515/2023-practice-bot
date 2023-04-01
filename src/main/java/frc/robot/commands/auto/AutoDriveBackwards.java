package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.*;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveBackwards extends CommandBase {

    private double distance;
    private double autoSpeed = -0.4;

    private Pose2d initPose;
    private DriveTrain driveTrain;
    private double startTime;
    private double initalLeftDistance;
    private double initalRightDistance;
    private double speedValue = .2;
    private double speedConstant = .1;
    private double rightError = 1;
    private double leftError = 1;

  public AutoDriveBackwards(DriveTrain driveTrain, double distance) {
    
    this.driveTrain = driveTrain;
    this.distance = distance;
 
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    initalLeftDistance = driveTrain.leftMeters();
    initalRightDistance = driveTrain.rightMeters();
  }


  @Override
  public void execute() {
    // Calculates how far we are
    rightError = (initalRightDistance + distance) - driveTrain.rightMeters();
    leftError = (initalLeftDistance + distance) - driveTrain.leftMeters();

    // Speeds to set the motors too
    double rightSpeed = (Math.max((rightError / distance), .5) * speedValue) + speedConstant;
    double leftSpeed = (Math.max((leftError / distance), .5) * speedValue) + speedConstant;

    driveTrain.setWheelSpeedsAuto(leftSpeed, rightSpeed);
    
  }

  public boolean withinBounds() {
    return rightError <= 0 && leftError <= 0;

    }

  private double getDisplacement() {
    return this.initPose.getTranslation().getDistance(this.driveTrain.getPose().getTranslation());
  }


 
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 5;
  }

  @Override
  public void end(boolean interrupt) {
    driveTrain.arcadeDrive(0, 0);
  }
}

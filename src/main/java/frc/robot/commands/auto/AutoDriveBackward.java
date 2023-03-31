package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.*;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveBackward extends CommandBase {
  DriveTrain drivetrain;
  private double distance;
  private double autoSpeed = -0.2;

  private Pose2d initPose;

  public AutoDriveBackward(DriveTrain drivetrain, double distance) {

    this.drivetrain = drivetrain;
    this.distance = distance;
   
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initPose = this.drivetrain.getPose();
  }

  @Override
  public void execute() {
    drivetrain.arcadeDrive(autoSpeed, 0);
  }

  public boolean withinBounds() {
    if (this.getDisplacement() > distance) {
      return true;
    }
    return false;
  }

  private double getDisplacement() {
    return this.initPose.getTranslation().getDistance(this.drivetrain.getPose().getTranslation());
  }


  @Override
  public boolean isFinished() {
    return withinBounds();
  }

  @Override
  public void end(boolean interrupt) {
    drivetrain.arcadeDrive(0, 0);
  }
}



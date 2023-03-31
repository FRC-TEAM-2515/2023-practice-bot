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
    private DriveTrain drivetrain;
    private double startTime;

  public AutoDriveBackwards(DriveTrain drivetrain, double distance) {
    
    this.drivetrain = drivetrain;
    this.distance = distance;
 
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initPose = this.drivetrain.getPose();
    startTime = Timer.getFPGATimestamp();
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
    return Timer.getFPGATimestamp() - startTime > 5;
  }

  @Override
  public void end(boolean interrupt) {
    drivetrain.arcadeDrive(0, 0);
  }
}

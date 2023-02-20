package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;

public class AutoLevel extends CommandBase{
    
    private PIDController autoLevelController;
    private double goal; 
    private double moveLimit;
    private DriveTrain m_driveTrain;
    private boolean isBalancing = false;

public AutoLevel(DriveTrain driveTrain) {
    this.m_driveTrain = driveTrain;
    addRequirements(driveTrain);

    autoLevelController = new PIDController(AutoConstants.kAutoForwardP,AutoConstants.kAutoForwardI,AutoConstants.kAutoForwardD);
}

// @Override
// public void initialize() {
//     this.moveLimit = FieldConstants.kchargeStationLengthMeters / 2;
// }

// @Override
//   public void execute() {
//     double levelSpeed = autoLevelController.calculate(m_driveTrain.get)
//     m_driveTrain.arcadeDrive(m_autoSpeed, 0);
//   }

// public double getDistance() {
//     return new Transform2d(initPose, m_driveTrain.getPose()).getTranslation().getX();
// }

// private double getDisplacement() {
//     return this.initPose.getTranslation().getDistance(this.m_driveTrain.getPose().getTranslation());
//   }

// @Override
// public boolean isFinished() {
//     if (this.getDisplacement() > m_autoDistance) {
//         return true;
//     } 
//     return false;
// }

// @Override 
// public void end(boolean interrupt) {
//     m_driveTrain.arcadeDrive(0, 0);
// }
}

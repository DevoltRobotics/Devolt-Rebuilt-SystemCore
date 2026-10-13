package frc.robot.Commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

import frc.robot.ShooterController;
import frc.robot.ShooterController.ShooterResult;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class StoreBallsCommand extends Command {

  CommandSwerveDrivetrain drivetrain;

  ShooterSubsystem shooter;
  TurretSubsystem turret;

  double Goal_X_Red = 12.8;

  double Goal_X_Blue = 3.496;

  double GoalRight_Y = 5.633;
  double GoalLeft_Y = 2.644;

  double Goal_X = 0;
  double Goal_Y = 0;

  private boolean getAlliance = false;

  private double lastTime = Timer.getMonotonicTimestamp();
  private double avgDt = 0.02;

  public StoreBallsCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem) {
    this.drivetrain = drivetrain;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    addRequirements(shooter, turret);
  }

  @Override
  public void execute() {
    double now = Timer.getMonotonicTimestamp();
    double dt = now - lastTime;
    lastTime = now;

    // filtro para que no brinque
    avgDt = 0.9 * avgDt + 0.1 * dt;

    Logger.recordOutput("SOTF/dt", dt);
    Logger.recordOutput("SOTF/avgDt", avgDt);
    Optional<Alliance> alliance = MatchState.getAlliance();
    Pose2d pose = drivetrain.getState().Pose;


    if (alliance.isPresent() && !getAlliance) {
      if (alliance.get() == Alliance.RED) {
        if (pose.getY() > 4) {
            Goal_X = Goal_X_Red;
            Goal_Y = GoalRight_Y;
        }else if(pose.getY() < 4){
            Goal_X = Goal_X_Red;
            Goal_Y = GoalLeft_Y;
        }  
      } else if (alliance.get() == Alliance.BLUE) {
        if (pose.getY() > 4) {
            Goal_X = Goal_X_Blue;
            Goal_Y = GoalRight_Y;
        }else if(pose.getY() < 4){
            Goal_X = Goal_X_Blue;
            Goal_Y = GoalLeft_Y;
        }  
      } else {
        Goal_X = Goal_X_Red;
        Goal_Y = GoalRight_Y;
      }
    }
    ChassisVelocities speeds = drivetrain.getState().Velocity;
    Translation2d turretFieldPos = pose.getTranslation().plus(turret.turretOffset.rotateBy(pose.getRotation()));

    ShooterResult result = ShooterController.calculate(
        turretFieldPos,
        new Translation2d(speeds.vx, speeds.vy),
        new Translation2d(Goal_X, Goal_Y),
        avgDt
    );

    shooter.setVelocity(result.requiredRps());
    turret.setAngle(result.turretFieldAngle().minus(pose.getRotation()).getDegrees());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

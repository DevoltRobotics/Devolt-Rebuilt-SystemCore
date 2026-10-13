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

public class SOTFCommand extends Command {

  CommandSwerveDrivetrain drivetrain;

  ShooterSubsystem shooter;
  TurretSubsystem turret;

  double Goal_X_Red = 11.920;
  double Goal_Y_Red = 4.035;

  double Goal_X_Blue = 4.634;
  double Goal_Y_Blue = 4.035;

  double Goal_X = 11.920;
  double Goal_Y = 4.035;

  private boolean getAlliance = false;

  private double lastTime = Timer.getMonotonicTimestamp();
  private double avgDt = 0.02;

  public SOTFCommand(
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

    Logger.recordOutput(turret.getName() + "/SOTF/dt", dt);
    Logger.recordOutput(turret.getName() + "/SOTF/avgDt", avgDt);
    Optional<Alliance> alliance = MatchState.getAlliance();

    if (alliance.isPresent() && !getAlliance) {
      if (alliance.get() == Alliance.RED) {
        Goal_X = Goal_X_Red;
        Goal_Y = Goal_Y_Red;
        getAlliance = true;
      } else if (alliance.get() == Alliance.BLUE) {
        Goal_X = Goal_X_Blue;
        Goal_Y = Goal_Y_Blue;
        getAlliance = true;
      } else {
        Goal_X = Goal_X_Red;
        Goal_Y = Goal_Y_Red;
      }
    }
    Pose2d pose = drivetrain.getState().Pose;
    ChassisVelocities speeds = drivetrain.getState().Velocity;

    Translation2d rotatedTurretOffset = turret.turretOffset.rotateBy(pose.getRotation());
    Translation2d turretFieldPos = pose.getTranslation().plus(rotatedTurretOffset);

    Logger.recordOutput("rotatedTurretOffset", rotatedTurretOffset);
    Logger.recordOutput("Chasis speed X", speeds.vx);
    Logger.recordOutput("Chasis speed y", speeds.vy);
    Logger.recordOutput("Chasis speed vamg", Math.hypot(speeds.vx, speeds.vy));


    Logger.recordOutput(turret.getName() + "/SOTF/turretFieldPos", new Pose2d(turretFieldPos, pose.getRotation().plus(turret.getAngle())));

    ShooterResult result = ShooterController.calculate(
        turretFieldPos,
        new Translation2d(speeds.vx, speeds.vy),
        new Translation2d(Goal_X, Goal_Y),
        0.092
    );

    shooter.setVelocity(result.requiredRps());
    turret.setAngle(result.turretFieldAngle().minus(pose.getRotation()).getDegrees());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

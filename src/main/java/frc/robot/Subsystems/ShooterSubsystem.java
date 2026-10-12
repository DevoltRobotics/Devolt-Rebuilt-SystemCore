// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX Lflywheel;

  private final TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

  private VelocityVoltage Velocity = new VelocityVoltage(0);
  
  Pose2d positionInRobot = new Pose2d();
  ChassisVelocities Speed = new ChassisVelocities();

  private double desiredVelocity;

  double TurretOut = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(int canId, Pose2d positionInRobot) {
    Lflywheel = new TalonFX(canId, new CANBus(CANPort.CAN_D0));

    FlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimit = 35;
    FlywheelConfig.Slot0.kP = 0.15;
    FlywheelConfig.Slot0.kV = 0.115;
    FlywheelConfig.Slot0.kS = 0.4;
    Lflywheel.getConfigurator().apply(FlywheelConfig);

    this.positionInRobot = positionInRobot;
    

    setName("Turret" + canId);
  }

  @Override
  public void periodic() {
    Lflywheel.setControl(Velocity.withVelocity(desiredVelocity));
    Logger.recordOutput("Shooter/Target vel", desiredVelocity);

  }

  public void setVelocity(double velocity) {
    desiredVelocity = velocity;
  }

  public Command SetVelocityCMD(double Vel) {
    return new InstantCommand(() -> {
      setVelocity(Vel);
    });
  }

}

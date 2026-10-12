// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.discrete.DigitalInput;

import frc.robot.Constants.CANId.CAN_s1;
import frc.robot.Constants.CANId.CAN_s2;

public class TransferSubsystem extends SubsystemBase {
  private DigitalInput laserLeft = new DigitalInput(0);
  private DigitalInput laserRight = new DigitalInput(1);

  private SparkMax beltMotor = new SparkMax(1, CAN_s1.TransferCan, MotorType.kBrushless);

  private TalonFX kickerMotor = new TalonFX(CAN_s2.KickerCan, CANBus.systemCore(2));
  private DutyCycleOut Power = new DutyCycleOut(0);

  private final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
  private final SparkMaxConfig beltConfig = new SparkMaxConfig();



  public TransferSubsystem() {
    beltConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);

      beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = 35;

    kickerMotor.getConfigurator().apply(kickerConfig);

  }

  
  public void setSpeeds(double beltSpeed, double kickerSpeed){
    beltMotor.setThrottle(beltSpeed);
    kickerMotor.setControl(Power.withOutput(kickerSpeed));
  }

  public Command TransferShootCMD(TransferSubsystem transferSubsystem){
    return new InstantCommand(()->{
        transferSubsystem.setSpeeds(1, 1);
    });
  }

  public Command StopTransferCMD(TransferSubsystem transferSubsystem){
        return new InstantCommand(()->{
          transferSubsystem.setSpeeds(0,0);
        });
  }
  @Override
  public void periodic(){
    Logger.recordOutput("leftTransfer", laserLeft.get());
    Logger.recordOutput("rightTransfer", laserRight.get());
  }
}

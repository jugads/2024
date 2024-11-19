// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  DutyCycleEncoder m_encoder;
  public Arm() {
    m_leftMotor = new CANSparkMax(19, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(20, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor Speed", m_leftMotor.get());
    SmartDashboard.putNumber("Encoder Position", m_encoder.getAbsolutePosition());
  }

  public void moveArm(double speed) {
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class CenterArm extends Command {
  /** Creates a new CenterArm. */
  PIDController controller;
  Arm m_arm;
  double desiredAngle = 0.;
  double currentAngle;
  public CenterArm(Arm arm) {
    controller = new PIDController(0.015, 0.0, 0.000005);
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // Replace with desired angle`
   currentAngle = -controller.calculate(desiredAngle, m_arm.getAngle());
   m_arm.moveArm(currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

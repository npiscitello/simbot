// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
import frc.robot.input.*;


public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Controller m_controller = new Controller(0);

  public RobotContainer() { 
    // teleop driving
    m_drivetrain.setDefaultCommand(new RunCommand(() -> 
      m_drivetrain.arcadeDrive(-m_controller.getAxis(Controller.Axis.kLeftY), m_controller.getAxis(Controller.Axis.kRightX)),
      //m_drivetrain.tankDrive(-m_controller.getAxis(Controller.Axis.kLeftY), m_controller.getAxis(Controller.Axis.kRightY)),
      m_drivetrain));
  }

  public Command getAutonomousCommand() { return null; }
}

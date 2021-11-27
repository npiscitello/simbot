// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.Characterize;
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

  public Command getAutonomousCommand() {
      // todo - Shuffleboard widget to choose between these
      //return new Characterize(m_drivetrain);
      return getTrajectoryCommand();
    }

    private Command getTrajectoryCommand() {
      TrajectoryConstraint autoVoltageContstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, Constants.kMaxVoltage);

      TrajectoryConfig config = 
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.kDriveKinematics)
        .addConstraint(autoVoltageContstraint);

      // all units meters
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // start
        new Pose2d(2, 2, new Rotation2d(0)),
        // pass through
        List.of(
          new Translation2d(3, 3),
          new Translation2d(4, 1)
        ),
        // end
        new Pose2d(5, 2, new Rotation2d(0)),
        config
      );

      RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory, 
        m_drivetrain::getPose, 
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain
      );

      m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
      return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
    }
}

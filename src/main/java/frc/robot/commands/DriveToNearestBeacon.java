package frc.robot.commands;

import frc.robot.subsystems.BeaconSensor;
import frc.robot.subsystems.RomiDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToNearestBeacon extends CommandBase {

  private Integer iterations = 0;

  public DriveToNearestBeacon(BeaconSensor sensor, RomiDrivetrain drivetrain) {
      addRequirements(sensor);
      addRequirements(drivetrain);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("DriveToNearestBeacon iterations:", ++iterations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
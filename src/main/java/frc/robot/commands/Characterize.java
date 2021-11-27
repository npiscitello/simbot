package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Characterize extends CommandBase {

    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    String data = "";
    int counter = 0;
    double priorAutospeed = 0;
    double numberArray[] = new double[10];
    ArrayList<Double> entries = new ArrayList<Double>();

    private Drivetrain m_drivetrain;
    
    public Characterize(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set the update rate instead of using flush because of a ntcore bug
        // -> probably don't want to do this on a robot in competition
        NetworkTableInstance.getDefault().setUpdateRate(0.010);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        double leftPosition = m_drivetrain.getLeftPositionMeters();
        double leftRate = m_drivetrain.getLeftVelocityMetersPerSecond();
    
        double rightPosition = m_drivetrain.getRightPositionMeters();
        double rightRate = m_drivetrain.getRightVelocityMetersPerSecond();
    
        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);
    
        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;

        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        m_drivetrain.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, -autospeed, false);

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = m_drivetrain.getGyroAngleRadians();

        // Add data to a string that is uploaded to NT
        for (double num : numberArray) {
            entries.add(num);
        }
        counter++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.tankDrive(0, 0, false);
        data = entries.toString();
        data = data.substring(1, data.length() - 1) + ", ";
        telemetryEntry.setString(data);
        entries.clear();
        data = "";
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}

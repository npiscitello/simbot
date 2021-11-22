package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SimDrivetrain;

public class TankDriveCommand extends CommandBase {

    private final SimDrivetrain m_drive;
    private final Joystick m_joystick = new Joystick(0);

    public TankDriveCommand(SimDrivetrain drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drive.tankDrive(m_joystick.getX(), -m_joystick.getY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

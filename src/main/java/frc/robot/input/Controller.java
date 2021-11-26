package frc.robot.input;

import edu.wpi.first.wpilibj.GenericHID;

// very similar to the xBox controller, but since I'm using a generic Logitech, I need to change button/axis IDs
// it appears Linux sees the triggers as axes but wpilib sees them as buttons
public class Controller extends GenericHID {

    public enum Button {
        kLeftBumper(4),
        kLeftTrigger(6),
        kLeftStick(10),
        kRightBumper(5),
        kRightTrigger(7),
        kRightStick(11),
        kA(1),
        kB(2),
        kX(0),
        kY(3),
        kBack(8),
        kStart(9);

        public final int id;
    
        Button(int id) { this.id = id; }
    }

    public enum Axis {
        kLeftX(0),
        kRightX(2),
        kLeftY(1),
        kRightY(3);

        public final int id;

        Axis(int id) { this.id = id; }
    }

    public Controller(final int port) { super(port); }
    public double getAxis(Axis axis) { return getRawAxis(axis.id); }
    public double getAxis(int axis_id) { return getRawAxis(axis_id); }
    public boolean getButton(Button button) { return getRawButton(button.id); }

    // these are not intended to be used, but must be overridden as abstract methods of the super class
    @Override
    public double getX(Hand hand) {
        switch(hand) {
            case kLeft: return getRawAxis(Axis.kLeftX.id);
            case kRight: return getRawAxis(Axis.kRightX.id);
            default: return 0.0;
        }
    }

    @Override
    public double getY(Hand hand) {
        switch(hand) {
            case kLeft: return getRawAxis(Axis.kLeftY.id);
            case kRight: return getRawAxis(Axis.kRightY.id);
            default: return 0.0;
        }
    }
}
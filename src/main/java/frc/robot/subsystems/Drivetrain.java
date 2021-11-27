package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    
    // real hardware
    private final Spark m_leftMotor = new Spark(0);
    private final Spark m_rightMotor = new Spark(1);
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    // simulation versions of hardware
    // we're pretending we have a kitbot until I can get my hands on a Romi again to characterize
    public static final DifferentialDrivetrainSim kDriveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.EightInch, null);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    private final  Field2d m_field = new Field2d();


    public Drivetrain() {
        m_leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
        m_rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
        resetEncoders();
        SmartDashboard.putData("Field", m_field);
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed, true);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        // arcade drive only works properly when inputs are flipped and I can't figure out why...
        m_drive.arcadeDrive(zRotation, xSpeed, true);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        // tell the simulator what we're telling the motors to do
        kDriveSim.setInputs(
            m_leftMotor.get() * RobotController.getInputVoltage(),
            m_rightMotor.get() * RobotController.getInputVoltage());
        
        // tick tock
        kDriveSim.update(0.020);

        // get all the new sensor values
        m_leftEncoderSim.setDistance(kDriveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(kDriveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(kDriveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(kDriveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-kDriveSim.getHeading().getDegrees());
    }

    /* TRAJECTORY FOLLOWING STUFF */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
        m_drive.feed();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /* CHARACTERIZATION STUFF */
    public double getLeftPositionMeters() { return m_leftEncoder.getDistance(); }
    public double getLeftVelocityMetersPerSecond() { return m_leftEncoder.getRate(); }
    public double getRightPositionMeters() { return m_rightEncoder.getDistance(); }
    public double getRightVelocityMetersPerSecond() { return m_rightEncoder.getRate(); }
    // gyro angle must be negated for characterization
    public double getGyroAngleRadians() { return -1 * m_gyro.getRotation2d().getRadians(); }
    public void tankDrive(double leftSpeed, double rightSpeed, boolean square) { m_drive.tankDrive(leftSpeed, rightSpeed, square); }
}
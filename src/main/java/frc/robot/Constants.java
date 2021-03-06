// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /* PHYSICAL ROBOT VALUES */
    // this shouldn't matter for just simulations, but make sure this matches the effective counts per wheel revolution on a real bot
    public static final double kCountsPerRevolution = 1440.0;
    // make sure this matches the DifferentialDrivetrainSim constructor above (or the wheel size on your real bot)
    public static final double kWheelDiameterMeters = Units.inchesToMeters(8);    

    /* DIFFERENTIAL DRIVE TRAJECTORY VALUES */
    // these constants have been generated by characterizing the simulated robot defined above
    // DO NOT USE ON A REAL ROBOT
    public static final double ksVolts = 0.24;
    public static final double kvVoltSecondsPerMeter = 2.18;
    public static final double kaVoltSecondsSquaredPerMeter = 0.298;
    public static final double kPDriveVel = 2.49;
    // pulled from kitbot definitino in the DifferentialDrivetrainSim source
    // for some reason, the characterization tool estimates a much smaller track width
    public static final double kTrackwidthMeters = Units.inchesToMeters(26);

    // kinematics object
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    // maximums
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxVoltage = 10;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}

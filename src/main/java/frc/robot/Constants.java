// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	/**
	 * The constants pertaining to the Driver station
	 */
	public static class OperatorConstants {

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

	}

	public static class ShooterConstants {

		public static final int kFlywheel1MotorID = 10;
		public static final int kKickerMotorID = 11;

		public static final int kFlywheel1SmartCurrentLimit = 10;

		public static final boolean kFlywheel1IsInverted = false;

		public static final IdleMode kFlywheel1IdleMode = IdleMode.kCoast;

		public static final double kFlywheel1GearRatio = 1 / 1;
		public static final double kFlywheel1Diameter = 4; // Inches

		public static final double kFlywheel1P = 0.1;
		public static final double kFlywheel1I = 0.0;
		public static final double kFlywheel1D = 0.0;


		public static final double kShooterIdleSpeed = 1;
		// assosciate a distance in meters with a speed for the shooter
		public static final HashMap<Double, Double> kDistanceToSpeedMap = new HashMap<Double, Double>() {
			{
				put(0d, 0d);
				put(1d, 5d);
				put(1.5, 8d);
				put(2d, 12d);
			}
		};

	}

	public static class TurretConstants {

		public static final int kTurretMotorID = 20;

		public static final int kTurretSmartCurrentLimit = 5;

		public static final double kTurretGearRatio = 1 / 2;

		public static final double kTurretP = 0.1;
		public static final double kTurretI = 0.0;
		public static final double kTurretD = 0.0;

		public static final double kForwardLimit = 180;
		public static final double kReverseLimit = -180;	

		public static final double kTargetCommandDeadband = 2;

		public static enum TurretStates {
			kSearching,
			kTargetGoal
		}
	}

	/**
	 * The constants pertaining to the any Limelights
	 */
	public static class LimelightConstants {

	}

}

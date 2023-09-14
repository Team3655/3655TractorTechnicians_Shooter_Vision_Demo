// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.TractorToolbox.TractorParts.PIDGains;
import frc.lib.TractorToolbox.TractorParts.SwerveConstants;
import frc.lib.TractorToolbox.TractorParts.SwerveModuleConstants;
import frc.robot.commands.Limelight.LLAlignCommand;

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

	public static class ModuleConstants {

		public static final int kMaxRezeroAttempts = 10;

		// gains set for R1 SDS mk4i using dual neo motors
		public static final PIDGains kModuleDriveGains = new PIDGains(.075, 0, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(1.5, 0.002, 0.0016);

		public static final class GenericModuleConstants {
			// Current limits for the wheels
			public static final int kTurnMotorCurrentLimit = 25;
			public static final int kDriveMotorCurrentLimit = 35;

			// Constants set for the _SDS MK4i_
			public static final double kTurnGearRatio = 1d / (150d / 7d);
			public static final double kDriveGearRatio = 1d / (38250 / 6885);
			public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;

			// The max speeds the modules are capable of
			public static final double kMaxModuleAccelMetersPerSecond = 5;
			public static final double kMaxModuleSpeedMetersPerSecond = 5.35;

			// Retune feedforward values for turning
			// public static final double kvTurning = .43205;
			// public static final double ksTurning = .17161; // Tuned February 2, 2023

			public static final double kDriveFeedForward = .2;

			public static final SwerveConstants kSwerveConstants = new SwerveConstants(
					kTurnMotorCurrentLimit,
					kDriveMotorCurrentLimit,
					kTurnGearRatio,
					kDriveGearRatio,
					kWheelCircumference,
					kMaxModuleAccelMetersPerSecond,
					kMaxModuleSpeedMetersPerSecond,
					kDriveFeedForward);
		}

		// module specific constants
		public static final class FrontLeftModule {
			public static final int kTurningMotorID = 10;
			public static final int kLeaderDriveMotorID = 11;
			public static final int kFollowerDriveMotorID = 12;
			// public static final int kAbsoluteEncoderID = 9;
			public static final double kAngleOffset = 26.3367;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}

		public static final class FrontRightModule {
			public static final int kTurningMotorID = 13;
			public static final int kLeaderDriveMotorID = 14;
			public static final int kFollowerDriveMotorID = 15;
			// public static final int kAbsoluteEncoderID = 10;
			public static final double kAngleOffset = 310.4834;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}

		public static final class BackLeftModule {
			public static final int kTurningMotorID = 16;
			public static final int kLeaderDriveMotorID = 17;
			public static final int kFollowerDriveMotorID = 18;
			// public static final int kAbsoluteEncoderID = 11;
			public static final double kAngleOffset = 31.5678;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}

		public static final class BackRightModule {
			public static final int kTurningMotorID = 19;
			public static final int kLeaderDriveMotorID = 20;
			public static final int kFollowerDriveMotorID = 21;
			// public static final int kAbsoluteEncoderID = 12;
			public static final double kAngleOffset = 43.5073;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}
	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.0;
		public static final double kMaxSpeedMetersPerSecond = 5.5;

		// this sets turning speed (keep this low) KsKs
		public static final double kMaxRPM = 8;

		public static final int kPigeonID = 2;

		public static final double kBumperToBumperWidth = Units.inchesToMeters(31);

		public static final double kTrackWidth = Units.inchesToMeters(20); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(20); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

		public static final boolean kGyroReversed = true;
		public static final boolean kFeildCentric = true;

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			public static final PIDGains kPPDriveGains = new PIDGains(8.5, 0, 0);
			public static final PIDGains kPPTurnGains = new PIDGains(3.5, 0, 0);

			public static final double kPPMaxVelocity = 4.00;
			public static final double kPPMaxAcceleration = 2.50;

			public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
				{
					put("TargetTape", new LLAlignCommand(false));
					put("TargetTag", new LLAlignCommand(true));
				}
			};
		}

		public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0.0003, 0);
		public static final double kTurnCommandMaxVelocity = 1;
		public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 0.5;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = 2;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.006, 0, 0);
		public static final double kMaxBalancingVelocity = 1000;
		public static final double kMaxBalancingAcceleration = 5000;
	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

		// button bindings 
		

		public static final double KDeadBand = .1;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	public static class LimelightConstants {

		// declare ID's of pipelines here
		public static final int kCubePipeline = 0;
		public static final int kReflectivePipeline = 1;
		public static final int kApriltagPipeline = 2;

		// PID values for limelight
		public static final PIDGains kLLTargetGains = new PIDGains(0.008, 0, 0);

		public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0); // .008
		public static final PIDGains kLLPuppyDriveGains = new PIDGains(0.008, 0, 0);
		public static final double kPuppyTurnMotionSmoothing = 0.3;
		public static final double kPuppyDriveMotionSmoothing = 0.4;

		public static final PIDGains kLLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
		public static final PIDGains kLLAlignDriveGains = new PIDGains(.025, 0.0015, 0.0005);
		public static final double kAlignDriveMotionSmoothing = 0;
		public static final double kAlignStrafeMotionSmoothing = 0;
	}

	public static final String kRioCANBusName = "rio";

	public static final String kCTRECANBusName = "rio";

}

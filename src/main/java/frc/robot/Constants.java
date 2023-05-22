// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TractorToolbox.TractorParts.PIDGains;
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

	public static class BaseModuleConstants {

		public static final boolean kIsThroughBore = false;
		public static final boolean kUseThroughBoreFeedback = false;
		public static final boolean kIsCANCoder = true;


		// Current limits for the wheels
		public static final int kTurnMotorCurrentLimit = 25;
		public static final int kDriveMotorCurrentLimit = 35;

		// Constants set for the _SDS MK4i_
		public static final double kdriveGearRatio = 1d / 5.56;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(18);

		public static final double ksVolts = .1;
		public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; // Tuned February 2, 2023

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 1;
		public static final int kFrontRightTurningMotorPort = 2;
		public static final int kRearLeftTurningMotorPort = 3;
		public static final int kRearRightTurningMotorPort = 4;

		public static final int kPrimaryFrontLeftDriveMotorPort = 5;
		public static final int kPrimaryFrontRightDriveMotorPort = 6;
		public static final int kPrimaryRearLeftDriveMotorPort = 7;
		public static final int kPrimaryRearRightDriveMotorPort = 8;

		// NEO drive motor CAN ID's
		public static final int kSecondaryFrontLeftDriveMotorPort = 9;
		public static final int kSecondaryFrontRightDriveMotorPort = 10;
		public static final int kSecondaryRearLeftDriveMotorPort = 11;
		public static final int kSecondaryRearRightDriveMotorPort = 12;

		// CANcoder CAN ID's
		public static final int kFrontLeftAbsoluteEncoderPort = 13;
		public static final int kFrontRightAbsoluteEncoderPort = 14;
		public static final int kRearLeftAbsoluteEncoderPort = 15;
		public static final int kRearRightAbsoluteEncoderPort = 16;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = 35.508;
		public static final double kFrontRightAngleZero = 0;
		public static final double kRearLeftAngleZero = 0;
		public static final double kRearRightAngleZero = 0;

		// gains set for SDS mk4i using neo motors
		public static final PIDGains kModuleDriveGains = new PIDGains(.15, 0.00005, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(1.5, 0, 0.0016);
	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.0;
		public static final double kMaxSpeedMetersPerSecond = 5.52;

		// this sets turning speed (keep this low)
		public static final double kMaxRPM = 10;

		public static final int kPigeonPort = 20;

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
		public static final boolean kGyroTuring = false;

		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

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

		public static final double kScoreSequenceDropTime = 3; // in seconds

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
	public static class DriverConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

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

		public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0); //.008
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.BaseModuleConstants;
import frc.robot.TractorToolbox.SparkMaxMaker;
import frc.robot.TractorToolbox.TractorParts.PIDGains;

public class SwerveModule {

	private final CANSparkMax turnMotor;
	private final CANSparkMax leaderDriveMotor;
	private final CANSparkMax followerDriveMotor;

	private final CANCoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;

	private final SparkMaxPIDController drivePID;
	private final SparkMaxPIDController turnPID;

	private final double angleZeroOffset;

	private final String moduleName;

	private SwerveModuleState optimizedState;

	/** Creates a new SwerveModule. */
	public SwerveModule(
			String moduleName,
			int turningMotorID,
			int leaderDriveMotorID,
			int followerDriveMotorID,
			int absoluteEncoderID,
			double angleZeroOffset,
			PIDGains angularPIDGains,
			PIDGains drivePIDGains) {

		this.moduleName = moduleName;
		this.angleZeroOffset = angleZeroOffset;

		// Initalize CANcoder
		absoluteEncoder = new CANCoder(absoluteEncoderID, Constants.kDriveCANBusName);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();

		// Initialize the motors
		// Initialize turning motor, encoder, and PID
		// Turn Motor
		turnMotor = SparkMaxMaker.createSparkMax(turningMotorID);
		turnMotor.setInverted(true);
		turnMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setSmartCurrentLimit(BaseModuleConstants.kTurnMotorCurrentLimit);
		// Turn Encoder
		turnEncoder = turnMotor.getEncoder();
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * BaseModuleConstants.kturnGearRatio); // radians
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * BaseModuleConstants.kturnGearRatio * (1d / 60d)); // radians per second
		turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition() - angleZeroOffset));
		// Turn PID
		turnPID = turnMotor.getPIDController();
		turnPID.setP(angularPIDGains.kP);
		turnPID.setI(angularPIDGains.kI);
		turnPID.setD(angularPIDGains.kD);
		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(-Math.PI);
		turnPID.setPositionPIDWrappingMaxInput(Math.PI);

		// Leader Drive Motor
		leaderDriveMotor = SparkMaxMaker.createSparkMax(leaderDriveMotorID);
		leaderDriveMotor.setIdleMode(IdleMode.kBrake);
		leaderDriveMotor.setSmartCurrentLimit(BaseModuleConstants.kDriveMotorCurrentLimit);
		// Leader Drive Encoder
		driveEncoder = leaderDriveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(
				BaseModuleConstants.kdriveGearRatio * BaseModuleConstants.kwheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				BaseModuleConstants.kdriveGearRatio
						* BaseModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second
		// Leader Drive PID
		drivePID = leaderDriveMotor.getPIDController();
		drivePID.setP(drivePIDGains.kP);
		drivePID.setI(drivePIDGains.kI);
		drivePID.setD(drivePIDGains.kD);
		drivePID.setFF(BaseModuleConstants.kDriveFeedForward);

		// Follower Drive Motor
		followerDriveMotor = SparkMaxMaker.createSparkMax(followerDriveMotorID);
		followerDriveMotor.follow(leaderDriveMotor);

	}

	// Returns headings of the module

	public double getHeading() {
		return turnEncoder.getPosition();
	}

	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition() - angleZeroOffset;
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		double moduleAngleRadians = getHeading();

		double distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(distanceMeters, new Rotation2d(moduleAngleRadians));
	}

	// region: Setters

	public void setDesiredState(SwerveModuleState desiredState) {
		setDesiredState(desiredState, false);
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isTurbo) {

		double moduleAngleRadians = getHeading();

		// Optimize the reference state to avoid spinning further than 90 degrees to the
		// desired state
		optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(moduleAngleRadians));

		if (isTurbo) {
			// Squeeze every bit if power out of turbo
			leaderDriveMotor.set(Math.signum(optimizedState.speedMetersPerSecond));
		} else {
			drivePID.setReference(
					optimizedState.speedMetersPerSecond,
					ControlType.kVelocity);
		}

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

	}

	public void resetAbsoluteEncoder() {
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.clearStickyFaults();
	}

	public void stopMotors() {
		leaderDriveMotor.stopMotor();
		turnMotor.stopMotor();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber(moduleName + " Offset", angleZeroOffset);
		SmartDashboard.putString(moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
		SmartDashboard.putNumber(moduleName + " Drive Current Draw", leaderDriveMotor.getOutputCurrent());
		SmartDashboard.putNumber(moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		SmartDashboard.putNumber(moduleName + " SparkEncoder Angle",
				Units.radiansToDegrees(turnEncoder.getPosition()));
	}

	// endregion: setters

}

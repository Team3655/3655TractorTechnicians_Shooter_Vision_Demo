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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.BaseModuleConstants;
import frc.robot.TractorToolbox.TractorParts.PIDGains;

public class SwerveModule {
	/** Creates a new SwerveModule. */

	private final CANSparkMax driveMotor;
	private final CANSparkMax turnMotor;

	private final CANCoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;

	private final SparkMaxPIDController drivePID;
	private final SparkMaxPIDController turnPID;

	public final double angleZeroOffset;

	private final String moduleName;

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZeroOffset,
			PIDGains angularPIDGains,
			PIDGains drivePIDGains) {

		this.moduleName = moduleName;
		this.angleZeroOffset = angleZeroOffset;

		// Initialize the motors
		driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		turnMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		driveMotor.setInverted(true);
		turnMotor.setInverted(true);

		turnMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		turnMotor.setInverted(true);

		// Initalize CANcoder
		absoluteEncoder = new CANCoder(absoluteEncoderPort, Constants.kDriveCANBusName);
		Timer.delay(1);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(
				BaseModuleConstants.kdriveGearRatio * BaseModuleConstants.kwheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				BaseModuleConstants.kdriveGearRatio
						* BaseModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		turnEncoder = turnMotor.getEncoder();
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * BaseModuleConstants.kturnGearRatio);
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * BaseModuleConstants.kturnGearRatio * (1d / 60d));
		turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition() - angleZeroOffset));

		// Initialize PID's
		drivePID = driveMotor.getPIDController();
		drivePID.setP(drivePIDGains.kP);
		drivePID.setI(drivePIDGains.kI);
		drivePID.setD(drivePIDGains.kD);

		turnPID = turnMotor.getPIDController();
		turnPID.setP(angularPIDGains.kP);
		turnPID.setI(angularPIDGains.kI);
		turnPID.setD(angularPIDGains.kD);

		drivePID.setFF(BaseModuleConstants.kDriveFeedForward);

		// Configure current limits for motors
		driveMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setSmartCurrentLimit(BaseModuleConstants.kTurnMotorCurrentLimit);
		driveMotor.setSmartCurrentLimit(BaseModuleConstants.kDriveMotorCurrentLimit);

		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(-Math.PI);
		turnPID.setPositionPIDWrappingMaxInput(Math.PI);

		SmartDashboard.putNumber(this.moduleName + " Offset", angleZeroOffset);
		SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
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

		double moduleAngleRadians = Math.toRadians(absoluteEncoder.getAbsolutePosition());

		double distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(distanceMeters, new Rotation2d(moduleAngleRadians));
	}

	// region: Setters

	public void setDesiredState(SwerveModuleState desiredState) {
		setDesiredState(desiredState, false);
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isTurbo) {

		double moduleAngleRadians = turnEncoder.getPosition();

		// Optimize the reference state to avoid spinning further than 90 degrees to the
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(moduleAngleRadians));

		if (isTurbo) {
			driveMotor.set(Math.signum(optimizedState.speedMetersPerSecond));

		} else {
			drivePID.setReference(
					optimizedState.speedMetersPerSecond,
					ControlType.kVelocity);
		}

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

		SmartDashboard.putNumber(this.moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(this.moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		SmartDashboard.putNumber(this.moduleName + " SparkEncoder Angle",
				Units.radiansToDegrees(turnEncoder.getPosition()));
	}

	public void resetAbsoluteEncoder() {
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.clearStickyFaults();
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}

	// endregion: setters

}

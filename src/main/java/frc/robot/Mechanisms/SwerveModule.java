// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
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
	// private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;

	private final String moduleName;

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPIDGains,
			PIDGains drivePIDGains) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		// Initialize the motors
		driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		turnMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		driveMotor.setInverted(true);
		turnMotor.setInverted(true);

		turnMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		turnMotor.setInverted(true);
		// driveMotor.setInverted(true);

		// Initalize CANcoder
		absoluteEncoder = new CANCoder(absoluteEncoderPort, Constants.kDriveCANBusName);
		Timer.delay(1);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		//absoluteEncoder.configMagnetOffset(-1 * angleZero);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatio * ModuleConstants.kwheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatio
						* ModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		turnEncoder = turnMotor.getEncoder();
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio);
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio * (1d / 60d));
		turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition() - angleZero));

		// Initialize PID's
		drivePID = driveMotor.getPIDController();
		drivePID.setP(drivePIDGains.kP);
		drivePID.setI(drivePIDGains.kI);
		drivePID.setD(drivePIDGains.kD);

		turnPID = turnMotor.getPIDController();
		turnPID.setP(angularPIDGains.kP);
		turnPID.setI(angularPIDGains.kI);
		turnPID.setD(angularPIDGains.kD);

		// m_turningPIDController = new ProfiledPIDController(
		// angularPID.kP,
		// angularPID.kI,
		// angularPID.kD,
		// new TrapezoidProfile.Constraints( // radians/s?
		// 2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
		// 2 * Math.PI * 1200));

		this.drivePID.setFF(ModuleConstants.kDriveFeedForward);

		this.drivePID.setFeedbackDevice(driveMotor.getEncoder());

		this.drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors
		driveMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
		driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);

		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(-Math.PI);
		turnPID.setPositionPIDWrappingMaxInput(Math.PI);

		SmartDashboard.putNumber(this.moduleName + " Offset", angleZero);
		SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition();
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

	public void setDesiredState(SwerveModuleState desiredState) {

		double moduleAngleRadians = Math.toRadians(absoluteEncoder.getAbsolutePosition());

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(moduleAngleRadians));

		// final var angularPIDOutput =
		// m_turningPIDController.calculate(moduleAngleRadians,
		// optimizedState.angle.getRadians());

		// final var angularFFOutput =
		// turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		// final var turnOutput = angularPIDOutput + angularFFOutput;

		// turnMotor.setVoltage(turnOutput);

		drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity);

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

		SmartDashboard.putNumber(this.moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(this.moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		// SmartDashboard.putNumber(this.moduleName + " Turn Output", turnOutput);

	}

	public void resetEncoders() {
		Timer.delay(.1);
		absoluteEncoder.configFactoryDefault();
		Timer.delay(.1);
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		Timer.delay(.1);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		Timer.delay(.1);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		Timer.delay(.1);
		absoluteEncoder.clearStickyFaults();
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}

}

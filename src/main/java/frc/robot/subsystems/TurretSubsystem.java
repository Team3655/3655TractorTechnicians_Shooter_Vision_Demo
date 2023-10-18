// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

	private static TurretSubsystem subsystem;

	private CANSparkMax turretMotor;
	private RelativeEncoder turretEncoder;
	private SparkMaxPIDController turretPidController;

	private double forwardLimit;
	private double reverseLimit;

	private double targetAngle;

	// private constructor to force use of getInstance() to create the Subsystem
	// object following the Singleton Design Pattern
	/** Creates a new TurretSubsystem. */
	private TurretSubsystem() {
		turretMotor = new CANSparkMax(TurretConstants.kTurretMotorID, MotorType.kBrushless);
		turretMotor.setSmartCurrentLimit(TurretConstants.kTurretSmartCurrentLimit);

		turretEncoder = turretMotor.getEncoder();
		turretEncoder.setPositionConversionFactor(TurretConstants.kTurretGearRatio * (2 * Math.PI));
		turretEncoder.setVelocityConversionFactor(turretEncoder.getPositionConversionFactor() * (1d / 60d));

		turretPidController = turretMotor.getPIDController();
		turretPidController.setP(TurretConstants.kTurretP);
		turretPidController.setI(TurretConstants.kTurretI);
		turretPidController.setD(TurretConstants.kTurretD);

		forwardLimit = TurretConstants.kForwardLimit;
		reverseLimit = TurretConstants.kReverseLimit;
	}

	public static TurretSubsystem getInstance() {
		if (subsystem == null)
			subsystem = new TurretSubsystem();
		return subsystem;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		turretPidController.setReference(targetAngle, ControlType.kPosition);
	}

	public void updateTargetFromVisionError(double visionError) {
		double updatedTarget = turretEncoder.getPosition() - visionError;
		targetAngle = optimize(updatedTarget);
	}

	/**
	 * optimizes the turrets target so that it never turns further than the safe
	 * operating range (safe range is defined by reverseLimit and forwardLimit)
	 * 
	 * @param angle The angle to be optimized
	 * 
	 * @return an angle as close to the input as possible that has been constrained
	 *         to the safe operating range
	 */
	public double optimize(double angle) {

		if (angle > forwardLimit && angle < reverseLimit) {

			double errorToReverseLimit = Math.abs(angle - reverseLimit);
			double errorToForwardLimit = Math.abs(angle - forwardLimit);

			if (errorToForwardLimit < errorToReverseLimit)
				return forwardLimit;
			else
				return reverseLimit;

		}

		if (angle > forwardLimit)
			return angle - 360;

		if (angle < reverseLimit)
			return angle + 360;

		return angle;
	}

	public boolean getAtTarget(double deadband) {

		double errorToTarget = Math.abs(turretEncoder.getPosition() - targetAngle);

		if (Math.abs(errorToTarget) < deadband) {
			return true;
		}

		return false;
	}
}
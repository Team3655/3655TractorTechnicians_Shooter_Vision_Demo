// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TractorToolbox.SparkMaxUtils;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Mechanisms.Flywheel;

public class ShooterSubsystem extends SubsystemBase {

	private static ShooterSubsystem subsystem;

	private final Flywheel flywheel1;

	private final HashMap<Double, Double> distanceToSpeedMap;

	// private constructor to force use of getInstance() to create the Subsystem
	// object following the Singleton Design Pattern
	/** Creates a new FlywheelSubsystem. */
	private ShooterSubsystem() {

		this.flywheel1 = new Flywheel(
				ShooterConstants.kFlywheel1MotorID,
				ShooterConstants.kFlywheel1SmartCurrentLimit,
				ShooterConstants.kFlywheel1IsInverted,
				ShooterConstants.kFlywheel1IdleMode,
				ShooterConstants.kFlywheel1GearRatio,
				ShooterConstants.kFlywheel1Diameter,
				ShooterConstants.kFlywheel1P,
				ShooterConstants.kFlywheel1I,
				ShooterConstants.kFlywheel1D);

		distanceToSpeedMap = ShooterConstants.kDistanceToSpeedMap;
	}

	public static ShooterSubsystem getInstance() {
		if (subsystem == null)
			subsystem = new ShooterSubsystem();
		return subsystem;
	}

	public void setVelocityFromDistance(double distanceToTarget) {

		double minGreater = Double.NEGATIVE_INFINITY;
		double maxLesser = Double.POSITIVE_INFINITY;

		for (double distKey : distanceToSpeedMap.keySet()) {
			if (distKey > distanceToTarget && distKey < minGreater)
				minGreater = distKey;

			if (distKey < distanceToTarget && distKey > maxLesser)
				maxLesser = distKey;
		}

		double lerpPercentage = (distanceToTarget - maxLesser) / (minGreater - maxLesser);

		double fromSpeed;

		// setVelocity(distanceMeters);
	}

	public void setVelocity(double velocity) {
		flywheel1.setVelocity(velocity);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SparkMaxUtils.sendMoterTelemetry("flywheel1", flywheel1.getMotor());
	}
}

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

	/**
	 * Takes a distance from the target to the robot as an input. Then interpolates
	 * between the shooter velocities directly above and below the distance in a
	 * HashMap, to get the optimal speed for shooting based on a distance
	 * 
	 * @param distanceToTarget
	 */
	public double getVelocityFromDistance(double distanceToTarget) {

		// minAbove and minBelow are the closest distance values above and below the
		// distanceToTarget in the HashMap. (i.e. if you have a distance of 4.5 and a
		// map associating the values {5=20, 4=10, 3=30} then maxBelow would be 5 and
		// minAbove would be 4)
		// Setting these to negetive and positive infinity guarentees any value in the
		// map will be greater or lesser and therefore they will always immeditly recive
		// a value from the map
		double minAbove = Double.NEGATIVE_INFINITY;
		double maximum = Double.NEGATIVE_INFINITY;
		double maxBelow = Double.POSITIVE_INFINITY;
		double minimum = Double.POSITIVE_INFINITY;

		// finding the minAbove and minBelow values
		for (double distKey : distanceToSpeedMap.keySet()) {
			if (distKey > distanceToTarget && distKey < minAbove)
				minAbove = distKey;

			if (distKey >= maximum)
				maximum = distKey;

			if (distKey < distanceToTarget && distKey > maxBelow)
				maxBelow = distKey;

			if (distKey <= minimum)
				minimum = distKey;
		}

		if (minAbove == Double.NEGATIVE_INFINITY) 
			return distanceToSpeedMap.get(maximum);

		else if (maxBelow == Double.POSITIVE_INFINITY)
			return distanceToSpeedMap.get(minimum);

		// finds the progress from minAbove to minBelow as a percentage to be used in
		// interpolating shooter speeds
		double lerpPercentage = (distanceToTarget - maxBelow) / (minAbove - maxBelow);

		// gets the speeds associated with the minAbove and minBelow distance
		double fromSpeed = distanceToSpeedMap.get(maxBelow);
		double toSpeed = distanceToSpeedMap.get(minAbove);

		// interpolates between the two speeds using all the values calculated before
		double lerpedVelocity = fromSpeed + ((toSpeed - fromSpeed) * lerpPercentage);

		return lerpedVelocity;
	}

	public void setVelocity(double velocity) {
		flywheel1.setVelocity(velocity);
	}

	public void setVelocityFromDistance(double distanceToTarget) {
		double velocity = getVelocityFromDistance(distanceToTarget);
		flywheel1.setVelocity(velocity);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SparkMaxUtils.sendMoterTelemetry("flywheel1", flywheel1.getMotor());
	}
}

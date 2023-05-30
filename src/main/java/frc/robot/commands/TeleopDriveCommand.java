// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.TractorToolbox.JoystickUtils;
import frc.robot.Constants.DriverConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {

	private DriveSubsystem driveSubsystem;

	private DoubleSupplier forwardSupplier;
	private DoubleSupplier strafeSupplier;
	private DoubleSupplier rotationSupplier;
	private BooleanSupplier isTurboSupplier;
	private BooleanSupplier isSneakSupplier;

	//private SlewRateLimiter speedLimiter;
	//private SlewRateLimiter rotationLimiter;

	/** Creates a new TeleopDriveCommand. */
	public TeleopDriveCommand(
		DoubleSupplier forwardSupplier,
		DoubleSupplier strafeSupplier,
		DoubleSupplier rotationSupplier,
		BooleanSupplier isTurboSupplier,
		BooleanSupplier isSneakSupplier) {

		// Use addRequirements() here to declare subsystem dependencies.
		driveSubsystem = RobotContainer.driveSubsystem;
		addRequirements(driveSubsystem);

		this.forwardSupplier = forwardSupplier;
		this.strafeSupplier = strafeSupplier;
		this.rotationSupplier = rotationSupplier;
		this.isTurboSupplier = isTurboSupplier;

		//speedLimiter = new SlewRateLimiter(5);
		//rotationLimiter = new SlewRateLimiter(5);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		double forward = forwardSupplier.getAsDouble();
		double strafe = strafeSupplier.getAsDouble();
		double rotation = rotationSupplier.getAsDouble();
		boolean isTurbo = isTurboSupplier.getAsBoolean();
		boolean isSneak = isSneakSupplier.getAsBoolean();

		Translation2d translation = new Translation2d(forward, strafe);

		// double speed = translation.getNorm();
		// Rotation2d angle = translation.getAngle();

		// Slew limited attempt
		// speed = speedLimiter.calculate(speed);
		// rotation = rotationLimiter.calculate(rotation);

		// translation = new Translation2d(speed, angle);

		// Curved input
		translation = JoystickUtils.curveTranslation2d(translation, DriverConstants.KDeadBand);
		rotation = JoystickUtils.curveInput(rotation, DriverConstants.KDeadBand);

		driveSubsystem.drive(translation, rotation, isTurbo, isSneak);
	}

}
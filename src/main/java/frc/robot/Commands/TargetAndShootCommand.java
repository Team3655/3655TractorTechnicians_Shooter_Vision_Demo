// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TargetAndShootCommand extends CommandBase {

	ShooterSubsystem shooterSubsystem;
	TurretSubsystem turretSubsystem;

	/** Creates a new TargetAndShootCommand. */ 
	public TargetAndShootCommand() {

		shooterSubsystem = ShooterSubsystem.getInstance();
		turretSubsystem = TurretSubsystem.getInstance();

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooterSubsystem, turretSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		double visionError = LimelightHelpers.getTX("limelight");
		turretSubsystem.updateTargetFromVisionError(visionError);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.setVelocity(ShooterConstants.kShooterIdleSpeed);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

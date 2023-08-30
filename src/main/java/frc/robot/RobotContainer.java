// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.TractorToolbox.TractorParts.PathBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.Autonomous.BalanceCommand;
import frc.robot.commands.Limelight.LLAlignCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...
	private static final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

	private static final PathBuilder autoBuilder = new PathBuilder();

	private final CommandJoystick driveJoystick = new CommandJoystick(
			OperatorConstants.kDriveJoystickPort);
	private final CommandJoystick turnJoystick = new CommandJoystick(
			OperatorConstants.kTurnJoystickPort);
	private final CommandGenericHID operatorController = new CommandGenericHID(
			OperatorConstants.kOperatorControllerPort);
	private final CommandXboxController programmerController = new CommandXboxController(
			OperatorConstants.kProgrammerControllerPort);

	private SendableChooser<Command> autoChooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// region Def Auto
		Shuffleboard.getTab("Driver").add(autoChooser);

		autoBuilder.populatePathMap();

		autoChooser.addOption("Square", autoBuilder.getPathCommand("Square"));
		// endregion
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		// region Targeting Commmands
		driveJoystick.button(3).whileTrue(new LLAlignCommand(false));
		driveJoystick.button(4).whileTrue(new LLAlignCommand(true));
		driveJoystick.button(5).whileTrue(new BalanceCommand());
		programmerController.a().whileTrue(new LLAlignCommand(false));
		programmerController.x().whileTrue(new TurnCommand(180));
		// endregion

		// test balance
		operatorController.button(12).whileTrue(new BalanceCommand());

		// region Drive Commands
		driveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		driveJoystick.button(12).onTrue(driveSubsystem.toggleFieldCentric());

		programmerController.button(8).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		programmerController.button(6).onTrue(driveSubsystem.toggleFieldCentric());

		driveJoystick.povUp().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(0.05, 0, 0), driveSubsystem));
		driveJoystick.povDown().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(-0.05, 0, 0), driveSubsystem));

		// Swerve Drive command is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new TeleopDriveCommand(
						() -> -driveJoystick.getY() -programmerController.getLeftY(),
						() -> -driveJoystick.getX() -programmerController.getLeftX(),
						() -> -turnJoystick.getX() -programmerController.getRightX(),
						() -> driveJoystick.getHID().getRawButton(1)
								|| programmerController.rightBumper().getAsBoolean(),
						() -> driveJoystick.getHID().getRawButton(2)
								|| programmerController.rightBumper().getAsBoolean()));
		// endregion
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		driveSubsystem.setHeading(180);
		Timer.delay(0.05);
		// the command to be run in autonomous
		return autoChooser.getSelected();
	}
}
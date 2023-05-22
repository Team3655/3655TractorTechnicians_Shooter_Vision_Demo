// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TelemetrySubsystem extends SubsystemBase {

	private static DriveSubsystem driveSubsystem;

	public TelemetrySubsystem() {
		driveSubsystem = RobotContainer.driveSubsystem;
	}

	@Override
	public void periodic() {
		updateDriveTelemetry();
	}

	public void updateDriveTelemetry() {
		driveSubsystem.updateTelemetry();
	}

	public static class PIDConstants {

		// PID Shuffleboard tab
		private static ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");

		// Entries for Drive/Turn PID

		private static GenericEntry Drive_kP = PIDTab.addPersistent("Drive kP", 0).getEntry();

		public static double getDrive_kP() {
			return Drive_kP.getDouble(0);
		}

	}
	
}

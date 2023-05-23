package frc.robot.TractorToolbox.TractorParts;

public class SwerveModuleParameters {

	public final int kCANCoderID;
	public final int kTurnMotorID;
	public final int kLeaderDriveMotorID;
	public final int kFolloweDriveMotorID;
	public final double kAngleZeroOffset;

	public SwerveModuleParameters(
			int CANCoderID,
			int TurnMotorID,
			int LeaderDriveMotorID,
			int FolloweDriveMotorID,
			int AngleZeroOffset) {
		kCANCoderID = CANCoderID;
		kTurnMotorID = TurnMotorID;
		kLeaderDriveMotorID = LeaderDriveMotorID;
		kFolloweDriveMotorID = FolloweDriveMotorID;
		kAngleZeroOffset = AngleZeroOffset;
	}

}
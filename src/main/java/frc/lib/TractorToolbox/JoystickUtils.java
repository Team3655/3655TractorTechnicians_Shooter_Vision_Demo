package frc.lib.TractorToolbox;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriverConstants;

public class JoystickUtils {

	/**
	 * squares the joystick input and uses clever math to compansate for the offset
	 * caused by the deadband
	 * 
	 * @param input The input from the joystick
	 * @return The corrected joystick values
	 */
	public static double curveInput(double input) {

		// returns zero if input is less than deadband
		if (MathUtil.applyDeadband(input, DriverConstants.KDeadBand) == 0)
			return 0;

		double correctedValue = input;

		// does funky math to force linear output between deanband and 1
		correctedValue = (correctedValue - (DriverConstants.KDeadBand * Math.signum(correctedValue)))
				/ (1 - DriverConstants.KDeadBand);

		// raises input to a specified power for a smoother feel
		correctedValue = Math.copySign(Math.pow(Math.abs(correctedValue), DriverConstants.kJoystickPow), input);

		return correctedValue;
	}

	public static Translation2d curveTranslation2d(Translation2d translation) {
		// gets the length and rotarion of the Translation2d (vector)
		double norm = translation.getNorm();
		Rotation2d angle = translation.getAngle();

		// applies outer deadband (because in this case the combination of x and y can cause the length to be greater than one)
		if (norm > 1)
			norm = 1;

		// curves the length of the vector for smoother feel
		double curvedNorm = curveInput(norm);

		// create new curved Translation2d
		translation = new Translation2d(curvedNorm, angle);

		return translation;
	}

}
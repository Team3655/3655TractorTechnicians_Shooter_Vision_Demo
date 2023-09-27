package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Flywheel {

	private final double gearRatio;
	private final double wheelDiameter;

	private final CANSparkMax motor;
	private final RelativeEncoder encoder;
	private final SparkMaxPIDController pidController;

	/**
	 * 
	 * @param motorID
	 * @param currentLimit
	 * @param isInverted
	 * @param idleMode
	 * @param gearRatio
	 * @param wheelDiameter
	 * @param P
	 * @param I
	 * @param D
	 */
	public Flywheel(
			final int motorID,
			final int currentLimit,
			final boolean isInverted,
			final IdleMode idleMode,
			final double gearRatio,
			final double wheelDiameter,
			final double P,
			final double I,
			final double D) {

		this.gearRatio = gearRatio;
		this.wheelDiameter = wheelDiameter;

		this.motor = new CANSparkMax(motorID, MotorType.kBrushless);
		this.motor.setInverted(isInverted);
		this.motor.setIdleMode(idleMode);
		this.motor.setSmartCurrentLimit(currentLimit);

		this.encoder = motor.getEncoder();
		// sets position conversion factor to the units of the wheelDiameter (i.e. if
		// the input for wheel diamter is in feet the encoder will output its position
		// in feet)
		this.encoder.setPositionConversionFactor(gearRatio * (wheelDiameter * Math.PI));
		// sets position conversion factor to the units of the wheelDiameter per second
		// (i.e. if the input for wheel diamter is in feet the encoder will output its
		// velocity in feet per second)
		this.encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() * (1d / 60d));

		// Initialize SparkMaxPIDController so we can target a specific velocity when
		// shooting
		this.pidController = motor.getPIDController();
		this.pidController.setP(P);
		this.pidController.setI(I);
		this.pidController.setD(D);
	}

	// region: setters
	public void setVelocity(final double velocity) {
		pidController.setReference(velocity, ControlType.kVelocity);
	}
	// endregion

	// region: getters
	/**
	 * @return the gearRatio
	 */
	public double getGearRatio() {
		return gearRatio;
	}

	/**
	 * @return the wheelDiameter
	 */
	public double getWheelDiameter() {
		return wheelDiameter;
	}

	/**
	 * @return the encoder
	 */
	public RelativeEncoder getEncoder() {
		return encoder;
	}

	/**
	 * @return the motor
	 */
	public CANSparkMax getMotor() {
		return motor;
	}

	/**
	 * @return the pidController
	 */
	public SparkMaxPIDController getPidController() {
		return pidController;
	}
	// endregion:

}

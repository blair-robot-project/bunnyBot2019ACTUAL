package org.usfirst.frc.team449.robot.components;

import edu.wpi.first.wpilibj.drive.Vector2d;
import org.jetbrains.annotations.Nullable;

/**
 * A component to determine the 4 motor outputs for a mecanum drive from an x vector, y vector, and theta.
 */
public class MecanumComponent {

	/**
	 * Desired velocities on [-1, 1]. (Rotation is in CCW+ rotations/sec)
	 */
	private double desiredLongitudinalVelocity, desiredLateralVelocity, desiredRotationalVelocity;

	/**
	 * Gyro angle in degrees. Can be null if not gyro is not being used (and control is thus not field-oriented).
	 */
	@Nullable
	private Double gyroAngularDisplacement;

	/**
	 * True if the drive is in O configuration, false if it is in X configuration.
	 */
	private boolean isOConfig;

	/**
	 * Whether the control should be field-oriented.
	 */
	private boolean isFieldOriented;

	/**
	 * Default constructor
	 *
	 * @param isOConfig                   Whether the drive is in O configuration.
	 * @param isFieldOriented             Whether the control should be field-oriented.
	 */
	public MecanumComponent(boolean isOConfig, boolean isFieldOriented) {
		this.isOConfig = isOConfig;
		this.isFieldOriented = isFieldOriented;
	}

	/**
	 * Update the desired velocities and gyro angular displacement for this component.
	 *
	 * @param desiredLongitudinalVelocity The desired forward-backward velocity, on [-1, 1].
	 * @param desiredLateralVelocity      The desired left-right velocity, on [-1, 1].
	 * @param desiredRotationalVelocity   The desired counter-clockwise positive rotations/sec, on [-1, 1].
	 * @param gyroAngularDisplacement     The gyro's angular displacement in degrees.
	 *                                    Can be null if not gyro is not being used (and control is thus not field-oriented).
	 */
	public void updateDesiredVelocitiesAndGyro(double desiredLongitudinalVelocity, double desiredLateralVelocity, double desiredRotationalVelocity,
	                                           @Nullable Double gyroAngularDisplacement) {
		this.desiredLongitudinalVelocity = desiredLongitudinalVelocity;
		this.desiredLateralVelocity = desiredLateralVelocity;
		this.desiredRotationalVelocity = desiredRotationalVelocity;
		this.gyroAngularDisplacement = gyroAngularDisplacement;
	}

	/**
	 * Determines wheel velocities based on desired robot velocities.<br>
	 * See {@link edu.wpi.first.wpilibj.drive.MecanumDrive#driveCartesian(double, double, double, double)} for math.
	 * @return A double array containing the four motor velocities to command (in the order of front right, front left,
	 * rear left, rear right).
	 */
	public double[] calculateMotorOutputs() {
		Vector2d input = new Vector2d(desiredLateralVelocity, desiredLongitudinalVelocity);
		// Compensate for gyro angle.
		if (isFieldOriented) {
			input.rotate(-gyroAngularDisplacement);
		}

		//Compute wheel speeds
		double[] motorOutputs = new double[4];
		//Negate lateral factor to use X configuration
		int posIfONegIfX = isOConfig ? 1 : -1;
		//Front right
		motorOutputs[0] = posIfONegIfX * -input.x + input.y + desiredRotationalVelocity;
		//Front left
		motorOutputs[1] = posIfONegIfX * input.x + input.y - desiredRotationalVelocity;
		//Rear left
		motorOutputs[2] = posIfONegIfX * -input.x + input.y - desiredRotationalVelocity;
		//Rear right
		motorOutputs[3] = posIfONegIfX * input.x + input.y + desiredRotationalVelocity;

		//Normalize wheel speeds
		double maxMagnitude = Math.abs(motorOutputs[0]);
		for (int i = 1; i < motorOutputs.length; i++) {
			double temp = Math.abs(motorOutputs[i]);
			if (maxMagnitude < temp) {
				maxMagnitude = temp;
			}
		}
		if (maxMagnitude > 1.0) {
			for (int i = 0; i < motorOutputs.length; i++) {
				motorOutputs[i] = motorOutputs[i] / maxMagnitude;
			}
		}

		return motorOutputs;
	}

}

package org.usfirst.frc.team449.robot.drive.omnidirectional.mecanum;

import com.fasterxml.jackson.annotation.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.components.MecanumComponent;
import org.usfirst.frc.team449.robot.drive.omnidirectional.DriveOmnidirectional;
import org.usfirst.frc.team449.robot.drive.unidirectional.DriveUnidirectional;
import org.usfirst.frc.team449.robot.generalInterfaces.simpleMotor.SimpleMotor;

/**
 * A mecanum drive, which uses mecanum wheels to drive omnidirectionally, with no gyro.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT, property = "@class")
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class DriveMecanumSimple extends Subsystem implements DriveOmnidirectional, DriveUnidirectional {

	/**
	 * The front right, front left, rear left, and rear right Talons.
	 */
	@NotNull
	private final SimpleMotor frontRightMotor, frontLeftMotor, rearLeftMotor, rearRightMotor;

	/**
	 * A mecanumComponent to convert desired velocities to motor outputs.
	 */
	@NotNull
	private MecanumComponent mecanumComponent;

	/**
	 * Default constructor.
	 *
	 * @param frontRightMotor The motor for the front right wheel.
	 * @param frontLeftMotor  The motor for the front left wheel.
	 * @param rearLeftMotor   The motor for the rear left wheel.
	 * @param rearRightMotor  The motor for the rear right wheel.
	 * @param isOConfig       Whether the drive is in O configuration. Defaults to true.
	 */
	@JsonCreator
	public DriveMecanumSimple(@NotNull @JsonProperty(required = true) SimpleMotor frontRightMotor,
	                          @NotNull @JsonProperty(required = true) SimpleMotor frontLeftMotor,
	                          @NotNull @JsonProperty(required = true) SimpleMotor rearLeftMotor,
	                          @NotNull @JsonProperty(required = true) SimpleMotor rearRightMotor,
	                          Boolean isOConfig) {
		this.frontRightMotor = frontRightMotor;
		this.frontLeftMotor = frontLeftMotor;
		this.rearLeftMotor = rearLeftMotor;
		this.rearRightMotor = rearRightMotor;
		boolean isOConfig_ = (isOConfig == null ? true : isOConfig);
		mecanumComponent = new MecanumComponent(isOConfig_, false);
	}

	/**
	 * Initialize the default command for a subsystem By default subsystems have no default command,
	 * but if they do, the default command is set with this method. It is called on all Subsystems by
	 * CommandBase in the users program after all the Subsystems are created.
	 */
	@Override
	protected void initDefaultCommand() {
		//Do nothing, the default command gets set with setDefaultCommandManual
	}

	/**
	 * Set the forward, strafe, and rotational velocities.
	 *
	 * @param desiredLongitudinalVelocity The desired forward-backward velocity, on [-1, 1].
	 * @param desiredLateralVelocity      The desired left-right velocity, on [-1, 1].
	 * @param desiredRotationalVelocity   The desired counter-clockwise positive rotations/sec, on [-1, 1].
	 */
	@Override
	public void setDirection(double desiredLongitudinalVelocity, double desiredLateralVelocity, double desiredRotationalVelocity) {
		mecanumComponent.updateDesiredVelocitiesAndGyro(desiredLongitudinalVelocity, desiredLateralVelocity,
				desiredRotationalVelocity, null);
		double[] motorOutputs = mecanumComponent.calculateMotorOutputs();
		frontRightMotor.setVelocity(motorOutputs[0]);
		frontLeftMotor.setVelocity(motorOutputs[1]);
		rearLeftMotor.setVelocity(motorOutputs[2]);
		rearRightMotor.setVelocity(motorOutputs[3]);
	}

	/**
	 * Completely stop the robot by setting the voltage to each side to be 0.
	 */
	@Override
	public void fullStop() {
		// Do nothing!
	}

	/**
	 * If this drive uses motors that can be disabled, enable them.
	 */
	@Override
	public void enableMotors() {
		frontRightMotor.enable();
		frontLeftMotor.enable();
		rearLeftMotor.enable();
		rearRightMotor.enable();
	}

	/**
	 * Reset the position of the drive if it has encoders.
	 */
	@Override
	public void resetPosition() {
		// Do nothing!
	}

	/**
	 * Set the output of each side of the drive.
	 *
	 * @param left  The output for the left side of the drive, from [-1, 1]
	 * @param right the output for the right side of the drive, from [-1, 1]
	 */
	@Override
	public void setOutput(double left, double right) {
		frontLeftMotor.setVelocity(left);
		rearLeftMotor.setVelocity(left);
		frontRightMotor.setVelocity(right);
		rearRightMotor.setVelocity(right);
	}

	/**
	 * Get the velocity of the left side of the drive.
	 *
	 * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getLeftVel() {
		return null;
	}

	/**
	 * Get the velocity of the right side of the drive.
	 *
	 * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getRightVel() {
		return null;
	}

	/**
	 * Get the position of the left side of the drive.
	 *
	 * @return The signed position in feet, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getLeftPos() {
		return null;
	}

	/**
	 * Get the position of the right side of the drive.
	 *
	 * @return The signed position in feet, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getRightPos() {
		return null;
	}

	/**
	 * Get the cached velocity of the left side of the drive.
	 *
	 * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getLeftVelCached() {
		return null;
	}

	/**
	 * Get the cached velocity of the right side of the drive.
	 *
	 * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getRightVelCached() {
		return null;
	}

	/**
	 * Get the cached position of the left side of the drive.
	 *
	 * @return The signed position in feet, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getLeftPosCached() {
		return null;
	}

	/**
	 * Get the cached position of the right side of the drive.
	 *
	 * @return The signed position in feet, or null if the drive doesn't have encoders.
	 */
	@Override
	public @Nullable Double getRightPosCached() {
		return null;
	}

	/**
	 * Updates all cached values with current ones.
	 */
	@Override
	public void update() {
		// Do nothing!
	}
}

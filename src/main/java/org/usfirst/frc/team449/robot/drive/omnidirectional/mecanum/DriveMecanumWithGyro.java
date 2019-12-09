package org.usfirst.frc.team449.robot.drive.omnidirectional.mecanum;

import com.fasterxml.jackson.annotation.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.components.MecanumComponent;
import org.usfirst.frc.team449.robot.drive.omnidirectional.DriveOmnidirectional;
import org.usfirst.frc.team449.robot.drive.unidirectional.DriveUnidirectional;
import org.usfirst.frc.team449.robot.generalInterfaces.simpleMotor.SimpleMotor;
import org.usfirst.frc.team449.robot.jacksonWrappers.MappedAHRS;
import org.usfirst.frc.team449.robot.subsystem.interfaces.AHRS.SubsystemAHRS;

/**
 * A mecanum drive, which uses mecanum wheels to drive omnidirectionally.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT, property = "@class")
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class DriveMecanumWithGyro extends Subsystem implements SubsystemAHRS, DriveOmnidirectional, DriveUnidirectional {

	/**
	 * The front right, front left, rear left, and rear right Talons.
	 */
	@NotNull
	private final SimpleMotor frontRightMotor, frontLeftMotor, rearLeftMotor, rearRightMotor;

	/**
	 * The NavX gyro
	 */
	@NotNull
	private final MappedAHRS ahrs;

	/**
	 * A mecanumComponent to convert desired velocities to motor outputs.
	 */
	@NotNull
	private MecanumComponent mecanumComponent;

	/**
	 * Whether or not to use the NavX for driving straight
	 */
	private boolean overrideGyro;

	/**
	 * Default constructor.
	 *
	 * @param frontRightMotor The motor for the front right wheel.
	 * @param frontLeftMotor  The motor for the front left wheel.
	 * @param rearLeftMotor   The motor for the rear left wheel.
	 * @param rearRightMotor  The motor for the rear right wheel.
	 * @param ahrs            The NavX gyro for calculating this drive's angular displacement.
	 * @param isOConfig       Whether the drive is in O configuration. Defaults to true.
	 * @param isFieldOriented Whether the control should be field-oriented. Defaults to true.
	 */
	@JsonCreator
	public DriveMecanumWithGyro(@NotNull @JsonProperty(required = true) SimpleMotor frontRightMotor,
	                            @NotNull @JsonProperty(required = true) SimpleMotor frontLeftMotor,
	                            @NotNull @JsonProperty(required = true) SimpleMotor rearLeftMotor,
	                            @NotNull @JsonProperty(required = true) SimpleMotor rearRightMotor,
	                            @NotNull @JsonProperty(required = true) MappedAHRS ahrs,
	                            Boolean isOConfig,
	                            Boolean isFieldOriented) {
		this.frontRightMotor = frontRightMotor;
		this.frontLeftMotor = frontLeftMotor;
		this.rearLeftMotor = rearLeftMotor;
		this.rearRightMotor = rearRightMotor;
		this.ahrs = ahrs;
		boolean isOConfig_ = (isOConfig == null ? true : isOConfig);
		boolean isFieldOriented_ = (isFieldOriented == null ? true : isFieldOriented);
		mecanumComponent = new MecanumComponent(isOConfig_, isFieldOriented_);
		overrideGyro = false;
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
				desiredRotationalVelocity, ahrs.getCachedHeading());
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
	 * Get the robot's heading.
	 *
	 * @return robot heading, in degrees, on [-180, 180].
	 */
	@Override
	public double getHeading() {
		return ahrs.getHeading();
	}

	/**
	 * Set the robot's heading.
	 *
	 * @param heading The heading to set to, in degrees on [-180, 180].
	 */
	@Override
	public void setHeading(double heading) {
		ahrs.setHeading(heading);
	}

	/**
	 * Get the robot's cached heading.
	 *
	 * @return robot heading, in degrees, on [-180, 180].
	 */
	@Override
	public double getHeadingCached() {
		return ahrs.getCachedHeading();
	}

	/**
	 * Get the robot's angular velocity.
	 *
	 * @return Angular velocity in degrees/sec
	 */
	@Override
	public double getAngularVel() {
		return ahrs.getAngularVelocity();
	}

	/**
	 * Get the robot's cached angular velocity.
	 *
	 * @return Angular velocity in degrees/sec
	 */
	@Override
	public double getAngularVelCached() {
		return ahrs.getCachedAngularVelocity();
	}

	/**
	 * Get the robot's angular displacement since being turned on.
	 *
	 * @return Angular displacement in degrees.
	 */
	@Override
	public double getAngularDisplacement() {
		return ahrs.getAngularDisplacement();
	}

	/**
	 * Get the robot's cached angular displacement since being turned on.
	 *
	 * @return Angular displacement in degrees.
	 */
	@Override
	public double getAngularDisplacementCached() {
		return ahrs.getCachedAngularDisplacement();
	}

	/**
	 * Get the pitch value.
	 *
	 * @return The pitch, in degrees from [-180, 180]
	 */
	@Override
	public double getPitch() {
		return ahrs.getPitch();
	}

	/**
	 * Get the cached pitch value.
	 *
	 * @return The pitch, in degrees from [-180, 180]
	 */
	@Override
	public double getCachedPitch() {
		return ahrs.getCachedPitch();
	}

	/**
	 * @return true if the gyroscope is currently overriden, false otherwise.
	 */
	@Override
	public boolean getOverrideGyro() {
		return overrideGyro;
	}

	/**
	 * @param override true to override the gyro, false to un-override it.
	 */
	@Override
	public void setOverrideGyro(boolean override) {
		overrideGyro = override;
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

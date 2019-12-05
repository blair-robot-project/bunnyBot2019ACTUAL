package org.usfirst.frc.team449.robot.sparkMax;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.components.RunningLinRegComponent;

import java.util.HashMap;
import java.util.Map;

/**
 * {@link org.usfirst.frc.team449.robot.jacksonWrappers.FPSTalon}, but as an abstract class.
 */
public abstract class CANMotorControllerBase {
	/**
	 * The PDP this motor is connected to.
	 */
	@NotNull
	protected final org.usfirst.frc.team449.robot.jacksonWrappers.PDP PDP;
	/**
	 * The counts per rotation of the encoder being used, or null if there is no encoder.
	 */
	@Nullable
	protected final Integer encoderCPR;
	/**
	 * The coefficient the output changes by after being measured by the encoder, e.g. this would be 1/70 if there was a
	 * 70:1 gearing between the encoder and the final output.
	 */
	protected final double postEncoderGearing;
	/**
	 * The number of feet travelled per rotation of the motor this is attached to, or null if there is no encoder.
	 */
	protected final double feetPerRotation;
	/**
	 * The minimum number of points that must be in the bottom-level MP buffer before starting a profile.
	 */
	protected final int minNumPointsInBottomBuffer;

	/**
	 * The period for bottomBufferLoader, in seconds.
	 */
	protected final double updaterProcessPeriodSecs;
	/**
	 * A list of all the gears this robot has and their settings.
	 */
	@NotNull
	protected final Map<Integer, SparkWrapper.PerGearSettings> perGearSettings = new HashMap<>();
	/**
	 * The motor's name, used for logging purposes.
	 */
	@NotNull
	protected final String name;
	/**
	 * The component for doing linear regression to find the resistance.
	 */
	@NotNull
	protected final RunningLinRegComponent voltagePerCurrentLinReg;
	/**
	 * Whether the forwards or reverse limit switches are normally open or closed, respectively.
	 */
	protected final boolean fwdLimitSwitchNormallyOpen, revLimitSwitchNormallyOpen;
	/**
	 * The settings currently being used by this Talon.
	 */
	@NotNull
	protected SparkWrapper.PerGearSettings currentGearSettings;
	/**
	 * The time at which the motion profile status was last checked. Only getting the status once per tic avoids CAN
	 * traffic.
	 */
	protected long timeMPStatusLastRead;
	/**
	 * The most recently set setpoint.
	 */
	protected double setpoint;
	/**
	 * RPS as used in a unit conversion method. Field to avoid garbage collection.
	 */
	protected Double RPS;
	/**
	 * The setpoint in native units. Field to avoid garbage collection.
	 */
	protected double nativeSetpoint;

	protected static final double voltageCompensation = 12;

	public CANMotorControllerBase(@NotNull org.usfirst.frc.team449.robot.jacksonWrappers.PDP PDP,
								  @Nullable Integer encoderCPR,
								  double postEncoderGearing,
								  double feetPerRotation,
								  int minNumPointsInBottomBuffer,
								  double updaterProcessPeriodSecs,
								  @NotNull String name,
								  @NotNull RunningLinRegComponent voltagePerCurrentLinReg,
								  boolean fwdLimitSwitchNormallyOpen,
								  boolean revLimitSwitchNormallyOpen) {
		this.PDP = PDP;
		this.encoderCPR = encoderCPR;
		this.postEncoderGearing = postEncoderGearing;
		this.feetPerRotation = feetPerRotation;
		this.minNumPointsInBottomBuffer = minNumPointsInBottomBuffer;
		this.updaterProcessPeriodSecs = updaterProcessPeriodSecs;
		this.name = name;
		this.voltagePerCurrentLinReg = voltagePerCurrentLinReg;
		this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
		this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
	}

	abstract void enable();
	abstract void disable();

	protected static class PerGearSettings {

	}
}

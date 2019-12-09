package org.usfirst.frc.team449.robot.oi.omnidirectional;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.generalInterfaces.loggable.Loggable;
import org.usfirst.frc.team449.robot.oi.throttles.Throttle;

/**
 * An OI to control a robot with an omnidirectional drive.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class OIOmnidirectionalSimple implements OIOmnidirectional, Loggable {

	/**
	 * Cached longitudinal/lateral/rotational values.
	 */
	private double[] motionCommandsCached;

	/**
	 * Right stick's y-axis (fwd/rev control) throttle
	 */
	@NotNull
	private final Throttle longitudinalThrottle;

	/**
	 * Right stick's x-axis (left/right control) throttle
	 */
	@NotNull
	private final Throttle lateralThrottle;

	/**
	 * Left (rotation control) stick's throttle
	 */
	@NotNull
	private final Throttle rotThrottle;

	/**
	 * Default constructor
	 *
	 * @param longitudinalThrottle The throttle for going forward and back.
	 * @param lateralThrottle The throttle for strafing left and right.
	 * @param rotThrottle The throttle for rotating the robot.
	 */
	@JsonCreator
	public OIOmnidirectionalSimple(@NotNull @JsonProperty(required = true) Throttle longitudinalThrottle,
								   @NotNull @JsonProperty(required = true) Throttle lateralThrottle,
								   @NotNull @JsonProperty(required = true) Throttle rotThrottle) {
		this.longitudinalThrottle = longitudinalThrottle;
		this.lateralThrottle = lateralThrottle;
		this.rotThrottle = rotThrottle;
	}

	/**
	 * The longitudinal, lateral and rotational motion commands.
	 *
	 * @return An array of length 3, where the first element is the longitudinal command, the second is lateral,
	 * and the third is rotational, all from [-1, 1].
	 */
	@Override
	public double[] getMotionCommands() {
		return new double[] {longitudinalThrottle.getValue(), lateralThrottle.getValue(), rotThrottle.getValue()};
	}

	/**
	 * The cached longitudinal, lateral and rotational motion commands.
	 *
	 * @return An array of length 3, where the first element is the longitudinal command, the second is lateral,
	 * and the third is rotational, all from [-1, 1].
	 */
	@Override
	public double[] getMotionCommandsCached() {
		return motionCommandsCached;
	}

	/**
	 * Updates all cached values with current ones.
	 */
	@Override
	public void update() {
		motionCommandsCached = getMotionCommands();
	}

	/**
	 * Get the headers for the data this subsystem logs every loop.
	 *
	 * @return An N-length array of String labels for data, where N is the length of the Object[] returned by getData().
	 */
	@NotNull
	@Override
	public String[] getHeader() {
		return new String[]{
				"longitudinal",
				"lateral",
				"rotational"
		};
	}

	/**
	 * Get the data this subsystem logs every loop.
	 *
	 * @return An N-length array of Objects, where N is the number of labels given by getHeader.
	 */
	@NotNull
	@Override
	public Object[] getData() {
		return new Object[] {
				getMotionCommandsCached()[0],
				getMotionCommandsCached()[1],
				getMotionCommandsCached()[2]};
	}

	/**
	 * Get the name of this object.
	 *
	 * @return A string that will identify this object in the log file.
	 */
	@NotNull
	@Override
	public String getLogName() {
		return "OI";
	}
}

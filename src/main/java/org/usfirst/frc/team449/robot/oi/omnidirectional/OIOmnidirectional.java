package org.usfirst.frc.team449.robot.oi.omnidirectional;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import org.usfirst.frc.team449.robot.oi.OI;

/**
 * An OI to control a robot with an omnidirectional drive.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT, property = "@class")
public interface OIOmnidirectional extends OI {

	/**
	 * The longitudinal, lateral and rotational motion commands.
	 *
	 * @return An array of length 3, where the first element is the longitudinal command, the second is lateral,
	 * and the third is rotational, all from [-1, 1].
	 */
	double[] getMotionCommands();

	/**
	 * The cached longitudinal, lateral and rotational motion commands.
	 *
	 * @return An array of length 3, where the first element is the longitudinal command, the second is lateral,
	 * and the third is rotational, all from [-1, 1].
	 */
	double[] getMotionCommandsCached();
}

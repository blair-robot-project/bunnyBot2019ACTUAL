package org.usfirst.frc.team449.robot.drive.omnidirectional;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import org.usfirst.frc.team449.robot.drive.DriveSubsystem;
import org.usfirst.frc.team449.robot.generalInterfaces.updatable.Updatable;

/**
 A drive with multiple degrees of movement ("omnidirectional").
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT, property = "@class")
public interface DriveOmnidirectional extends DriveSubsystem, Updatable {

	/**
	 * Set the forward, strafe, and rotational velocities.
	 *
	 * @param desiredLongitudinalVelocity The desired forward-backward velocity, on [-1, 1].
	 * @param desiredLateralVelocity      The desired left-right velocity, on [-1, 1].
	 * @param desiredRotationalVelocity   The desired counter-clockwise positive rotations/sec, on [-1, 1].
	 */
	void setDirection(double desiredLongitudinalVelocity, double desiredLateralVelocity, double desiredRotationalVelocity);

}
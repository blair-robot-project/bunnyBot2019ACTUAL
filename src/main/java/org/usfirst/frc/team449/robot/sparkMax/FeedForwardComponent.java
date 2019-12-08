package org.usfirst.frc.team449.robot.sparkMax;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import org.jetbrains.annotations.NotNull;

import java.util.function.DoubleUnaryOperator;

/**
 * A component for calculating feedforwards for a Talon. Takes the setpoint and returns the correct feedforward voltage.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT, property = "@class")
public abstract class FeedForwardComponent implements DoubleUnaryOperator {

	/**
	 * The CANSpark this controls the feedforward for.
	 */
	protected SparkWrapper spark;

	/**
	 * Get a FeedForwardComponent that gives no feedforward.
	 *
	 * @return A FeedForwardComponent whose methods all return 0.
	 */
	@NotNull
	public static FeedForwardComponent getZeroFeedForward() {
		return new FeedForwardZeroComponent();
	}

	/**
	 * Set the spark to get information from. This is a setter instead of being in the constructor to avoid circular
	 * referencing.
	 *
	 * @param spark The spark this controls the feedforward for.
	 */
	public void setController(@NotNull SparkWrapper spark) {
		this.spark = spark;
	}

	/**
	 * Calculate the voltage for a setpoint in MP mode with a position, velocity, and acceleration setpoint.
	 *
	 * @param positionSetpoint The desired position, in feet.
	 * @param velSetpoint      The desired velocity, in feet/sec.
	 * @param accelSetpoint    The desired acceleration, in feet/sec^2.
	 * @return The voltage, from [-12, 12] needed to achieve that velocity and acceleration.
	 */
	public abstract double calcMPVoltage(double positionSetpoint, double velSetpoint, double accelSetpoint);

	/**
	 * Calculate the voltage for the given input.
	 *
	 * @param operand the setpoint, in feet, feet/sec, feet/sec^2, etc.
	 * @return the feedforward voltage to use for that input.
	 */
	@Override
	public abstract double applyAsDouble(double operand);

	/**
	 * A {@link org.usfirst.frc.team449.robot.generalInterfaces.doubleUnaryOperator.FeedForwardComponent.FeedForwardComponent} that gives no output.
	 */
	public static class FeedForwardZeroComponent extends FeedForwardComponent {

		/**
		 * Default constructor. Not public and not annotated with @JsonCreator because it should only be constructed via
		 * {@link org.usfirst.frc.team449.robot.generalInterfaces.doubleUnaryOperator.FeedForwardComponent.FeedForwardComponent}'s getZeroFeedForward().
		 */
		FeedForwardZeroComponent() {
			//Do nothing
		}

		/**
		 * Calculate the voltage for a setpoint in MP mode with a position, velocity, and acceleration setpoint.
		 *
		 * @param positionSetpoint The desired position, in feet.
		 * @param velSetpoint      The desired velocity, in feet/sec.
		 * @param accelSetpoint    The desired acceleration, in feet/sec^2.
		 * @return The voltage, from [-12, 12] needed to achieve that velocity and acceleration.
		 */
		@Override
		public double calcMPVoltage(double positionSetpoint, double velSetpoint, double accelSetpoint) {
			return 0;
		}

		/**
		 * Calculate the feedforward for the given input.
		 *
		 * @param operand the setpoint, in feet, feet/sec, feet/sec^2, etc.
		 * @return the feedforward (kF gain) to use for that input.
		 */
		@Override
		public double applyAsDouble(double operand) {
			return 0;
		}
	}
}


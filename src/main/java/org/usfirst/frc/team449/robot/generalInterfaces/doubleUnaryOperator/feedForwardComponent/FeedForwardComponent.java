package org.usfirst.frc.team449.robot.generalInterfaces.doubleUnaryOperator.feedForwardComponent;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.sparkMax.SmartMotorController;

import java.util.function.DoubleUnaryOperator;

/**
 * A component for calculating feedforwards for a Talon. Takes the setpoint and returns the correct feedforward
 * voltage.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT, property = "@class")
public abstract class FeedForwardComponent implements DoubleUnaryOperator {

    /**
     * The motorController this controls the feedforward for.
     */
    protected SmartMotorController motorController;

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
     * Set the motorController to get information from. This is a setter instead of being in the constructor to avoid circular
     * referencing.
     *
     * @param motorController The motorController this controls the feedforward for.
     */
    public void setMotorController(@NotNull SmartMotorController motorController) {
        this.motorController = motorController;
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
     * Calculate the voltage for a given velocity setpoint when at a given position.
     * @param position The position, in feet.
     * @param velSetpoint The desired velocity, in feet/sec.
     * @return The voltage, from [-12, 12] needed to achieve that velocity at steady-state.
     */
    public double calcVelVoltage(double position, double velSetpoint){
        return calcMPVoltage(position, velSetpoint, 0);
    }

    /**
     * Calculate the voltage to use as a feedforward for the given position.
     *
     * @param operand the setpoint, in feet.
     * @return the feedforward voltage to use for that input.
     */
    @Override
    public abstract double applyAsDouble(double operand);
}

package org.usfirst.frc.team449.robot.sparkMax;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.generalInterfaces.loggable.Loggable;
import org.usfirst.frc.team449.robot.generalInterfaces.shiftable.Shiftable;
import org.usfirst.frc.team449.robot.generalInterfaces.simpleMotor.SimpleMotor;
import org.usfirst.frc.team449.robot.other.MotionProfileData;

public interface SmartMotorController extends SimpleMotor, Shiftable, Loggable {
    /**
     * Set the motor output voltage to a given percent of available voltage.
     *
     * @param percentVoltage percent of total voltage from [-1, 1]
     */
    void setPercentVoltage(double percentVoltage);

    /**
     * @return The gear this subsystem is currently in.
     */
    @Override
    int getGear();

    /**
     * Shift to a specific gear.
     *
     * @param gear Which gear to shift to.
     */
    @Override
    void setGear(int gear);

    /**
     * Set a position setpoint for the Talon.
     *
     * @param feet An absolute position setpoint, in feet.
     */
    void setPositionSetpoint(double feet);

    /**
     * Get the velocity of the CANTalon in FPS.
     *
     * @return The CANTalon's velocity in FPS, or null if no encoder CPR was given.
     */
    @Nullable Double getVelocity();

    /**
     * Set the velocity for the motor to go at.
     *
     * @param velocity the desired velocity, on [-1, 1].
     */
    @Override
    void setVelocity(double velocity);

    /**
     * Get the current closed-loop velocity error in FPS. WARNING: will give garbage if not in velocity mode.
     *
     * @return The closed-loop error in FPS, or null if no encoder CPR was given.
     */
    @Nullable Double getError();

    /**
     * Get the current velocity setpoint of the Talon in FPS. WARNING: will give garbage if not in velocity mode.
     *
     * @return The closed-loop velocity setpoint in FPS, or null if no encoder CPR was given.
     */
    @Nullable Double getSetpoint();

    /**
     * Get the voltage the Talon is currently drawing from the PDP.
     *
     * @return Voltage in volts.
     */
    double getOutputVoltage();

    /**
     * Get the voltage available for the Talon.
     *
     * @return Voltage in volts.
     */
    double getBatteryVoltage();

    /**
     * Get the current the Talon is currently drawing from the PDP.
     *
     * @return Current in amps.
     */
    double getOutputCurrent();

    /**
     * Get the current control mode of the Talon. Please don't use this for anything other than logging.
     *
     * @return Control mode as a string.
     */
    String getControlMode();

    /**
     * Enables the motor, if applicable.
     */
    @Override
    void enable();

    /**
     * Disables the motor, if applicable.
     */
    @Override
    void disable();

    /**
     * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
     *
     * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given gear.
     * @param gear     The number of the gear to use the max speed from to scale the velocity.
     */
    void setGearScaledVelocity(double velocity, int gear);

    /**
     * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
     *
     * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given gear.
     * @param gear     The gear to use the max speed from to scale the velocity.
     */
    void setGearScaledVelocity(double velocity, gear gear);

    /**
     * @return the position of the motorController in feet, or null of inches per rotation wasn't given.
     */
    Double getPositionFeet();

    /**
     * Resets the position of the Talon to 0.
     */
    void resetPosition();

    /**
     * Get the status of the forwards limit switch.
     *
     * @return True if the forwards limit switch is closed, false if it's open or doesn't exist.
     */
    boolean getFwdLimitSwitch();

    /**
     * Get the status of the reverse limit switch.
     *
     * @return True if the reverse limit switch is closed, false if it's open or doesn't exist.
     */
    boolean getRevLimitSwitch();

    /**
     * Whether this motorController is ready to start running a profile.
     *
     * @return True if minNumPointsInBottomBuffer points have been loaded or the top buffer is empty, false otherwise.
     */
    boolean readyForMP();

    /**
     * Whether this motorController has finished running a profile.
     *
     * @return True if the active point in the motorController is the last point, false otherwise.
     */
    boolean MPIsFinished();

    /**
     * Starts running the loaded motion profile.
     */
    void startRunningMP();

    /**
     * Holds the current position point in MP mode.
     */
    void holdPositionMP();

    /**
     * Disables the motorController and loads the given profile into the motorController.
     *
     * @param data The profile to load.
     */
    void loadProfile(MotionProfileData data);

    /**
     * Get the headers for the data this subsystem logs every loop.
     *
     * @return An N-length array of String labels for data, where N is the length of the Object[] returned by getData().
     */
    @NotNull
    @Override
    String[] getHeader();

    /**
     * Get the data this subsystem logs every loop.
     *
     * @return An N-length array of Objects, where N is the number of labels given by getHeader.
     */
    @NotNull
    @Override
    Object[] getData();

    /**
     * Get the name of this object.
     *
     * @return A string that will identify this object in the log file.
     */
    @NotNull
    @Override
    String getLogName();

}

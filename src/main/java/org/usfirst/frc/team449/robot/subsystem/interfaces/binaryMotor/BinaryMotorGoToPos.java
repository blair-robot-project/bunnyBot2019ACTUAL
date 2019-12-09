package org.usfirst.frc.team449.robot.subsystem.interfaces.binaryMotor;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.sparkMax.SmartMotorController;

/**
 * A binary motor subsystem that uses PID to go to a given position when turned on.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class BinaryMotorGoToPos extends Subsystem implements SubsystemBinaryMotor {

    /**
     * The motorController to move to the given position.
     */
    @NotNull
    private final SmartMotorController motorController;

    /**
     * The position, in feet, for the motorController to go to.
     */
    private final double positionFeet;

    /**
     * Whether or not the motor is on.
     */
    private boolean motorOn;

    /**
     * Default constructor
     *
     * @param motorController        The motorController to move to the given position.
     * @param positionFeet The position, in feet, for the motorController to go to. Defaults to 0.
     */
    @JsonCreator
    public BinaryMotorGoToPos(@JsonProperty(required = true) @NotNull SmartMotorController motorController,
                              double positionFeet) {
        this.motorController = motorController;
        this.positionFeet = positionFeet;
        motorOn = false;
    }

    /**
     * Do nothing.
     */
    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Turns the motor on, and sets it to a map-specified position.
     */
    @Override
    public void turnMotorOn() {
        motorController.enable();
        motorController.setPositionSetpoint(positionFeet);
        motorOn = true;
    }

    /**
     * Turns the motor off.
     */
    @Override
    public void turnMotorOff() {
        motorController.disable();
        motorOn = false;
    }

    /**
     * @return true if the motor is on, false otherwise.
     */
    @Override
    public boolean isMotorOn() {
        return motorOn;
    }
}

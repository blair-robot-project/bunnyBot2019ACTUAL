package org.usfirst.frc.team449.robot.subsystem.interfaces.position;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.components.PathGenerator;
import org.usfirst.frc.team449.robot.generalInterfaces.smartMotor.SmartMotor;
import org.usfirst.frc.team449.robot.generalInterfaces.updatable.Updatable;
import org.usfirst.frc.team449.robot.other.MotionProfileData;
import org.usfirst.frc.team449.robot.generalInterfaces.smartMotor.SmartMotorBase;
import org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.SubsystemMP;

/**
 * A SubsystemPosition that moves using motion profiles.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class SubsystemPositionOnboardMP extends Subsystem implements SubsystemPosition, Updatable, SubsystemMP {

    /**
     * The motor controller this subsystem controls.
     */
    protected final SmartMotorBase motorController;

    /**
     * The object for generating the paths for the motor to run.
     */
    private final PathGenerator pathGenerator;
    /**
     * Whether or not to start running the profile loaded into the motor controller.
     */
    protected boolean shouldStartProfile;
    /**
     * The previously observed motor velocity. Used for calculating acceleration.
     */
    private double lastVel;
    /**
     * The acceleration of the motor.
     */
    private double accel;

    /**
     * Default constructor.
     *
     * @param motorController         The motorController this subsystem controls.
     * @param pathGenerator The object for generating the paths for the motor to run.
     */
    @JsonCreator
    public SubsystemPositionOnboardMP(@NotNull @JsonProperty(required = true) SmartMotorBase motorController,
                                      @NotNull @JsonProperty(required = true) PathGenerator pathGenerator) {
        this.motorController = motorController;
        this.pathGenerator = pathGenerator;
        shouldStartProfile = false;
    }

    /**
     * Initialize the default command for a subsystem By default subsystems have no default command, but if they do, the
     * default command is set with this method. It is called on all Subsystems by CommandBase in the users program after
     * all the Subsystems are created.
     */
    @Override
    protected void initDefaultCommand() {
        //Do nothing
    }

    /**
     * Set the position setpoint
     *
     * @param feet Setpoint in feet from the limit switch used to zero
     */
    @Override
    public void setPositionSetpoint(double feet) {
        disableMotor();
        loadMotionProfile(pathGenerator.generateProfile(motorController.getPositionFeet(), motorController.getVelocity(), accel, feet));
        shouldStartProfile = true;
    }

    /**
     * Set a % output setpoint for the motor.
     *
     * @param output The speed for the motor to run at, on [-1, 1]
     */
    @Override
    public void setMotorOutput(double output) {
        motorController.setVelocity(output);
    }

    /**
     * Get the state of the reverse limit switch.
     *
     * @return True if the reverse limit switch is triggered, false otherwise.
     */
    @Override
    public boolean getReverseLimit() {
        return motorController.getRevLimitSwitch();
    }

    /**
     * Get the state of the forwards limit switch.
     *
     * @return True if the forwards limit switch is triggered, false otherwise.
     */
    @Override
    public boolean getForwardLimit() {
        return motorController.getFwdLimitSwitch();
    }

    /**
     * Set the position to 0.
     */
    @Override
    public void resetPosition() {
        motorController.resetPosition();
    }

    /**
     * Check if the mechanism has reached the setpoint.
     *
     * @return True if the setpoint has been reached, false otherwise.
     */
    @Override
    public boolean onTarget() {
        //Don't stop before we start the profile
        if (profileFinished() && !shouldStartProfile) {
            motorController.holdPositionMP();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Enable the motors of this subsystem.
     */
    @Override
    public void enableMotor() {
        motorController.enable();
    }

    /**
     * Disable the motors of this subsystem.
     */
    @Override
    public void disableMotor() {
        motorController.disable();
    }

    /**
     * Updates all cached values with current ones.
     */
    @Override
    public void update() {
        //Update acceleration
        accel = motorController.getVelocity() - lastVel;
        //Do clever math to get the motorController velocity back out
        lastVel = accel + lastVel;
    }

    /**
     * When the run method of the scheduler is called this method will be called.
     * <p>
     * Starts running the profile if it's ready.
     */
    @Override
    public void periodic() {
        //Start the profile if it's ready
        if (shouldStartProfile && readyToRunProfile()) {
            startRunningLoadedProfile();
            shouldStartProfile = false;
        }
    }

    /**
     * Loads a profile into the MP buffer.
     *
     * @param profile The profile to be loaded.
     */
    @Override
    public void loadMotionProfile(@NotNull MotionProfileData profile) {
        motorController.loadProfile(profile);
    }

    /**
     * Start running the profile that's currently loaded into the MP buffer.
     */
    @Override
    public void startRunningLoadedProfile() {
        motorController.startRunningMP();
    }

    /**
     * Get whether this subsystem has finished running the profile loaded in it.
     *
     * @return true if there's no profile loaded and no profile running, false otherwise.
     */
    @Override
    public boolean profileFinished() {
        return motorController.MPIsFinished();
    }

    /**
     * Disable the motors.
     */
    @Override
    public void disable() {
        disableMotor();
    }

    /**
     * Hold the current position.
     */
    @Override
    public void holdPosition() {
        motorController.holdPositionMP();
    }

    /**
     * Get whether the subsystem is ready to run the loaded profile.
     *
     * @return true if a profile is loaded and ready to run, false otherwise.
     */
    @Override
    public boolean readyToRunProfile() {
        return motorController.readyForMP();
    }
}

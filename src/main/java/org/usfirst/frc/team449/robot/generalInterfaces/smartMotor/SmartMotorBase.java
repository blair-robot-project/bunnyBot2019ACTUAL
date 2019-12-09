package org.usfirst.frc.team449.robot.generalInterfaces.smartMotor;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.components.RunningLinRegComponent;

import java.util.HashMap;
import java.util.Map;

import static org.usfirst.frc.team449.robot.util.Util.defaultIfNull;

/**
 * Abstract base class for motion-profile supporting motor controllers such as {@link org.usfirst.frc.team449.robot.jacksonWrappers.FPSTalon}.
 */
public abstract class SmartMotorBase implements SmartMotor {
    /**
     * The PDP this motor is connected to.
     */
    @Nullable
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
     * The period for this controller's motion updater, in seconds.
     */
    protected final double updaterProcessPeriodSecs;
    /**
     * A list of all the gears this robot has and their settings.
     */
    @NotNull
    protected final Map<Integer, PerGearSettings> perGearSettings;
    /**
     * The motor's name, used for logging purposes.
     */
    @NotNull
    protected final String name;
    /**
     * The component for doing linear regression to find the resistance.
     */
    @Nullable
    protected final RunningLinRegComponent voltagePerCurrentLinReg;

    protected static final double VOLTAGE_COMPENSATION = 12;
    /**
     * Whether the forwards or reverse limit switches are normally open or closed, respectively.
     */
    protected final boolean fwdLimitSwitchNormallyOpen;
    protected final boolean revLimitSwitchNormallyOpen;
    /**
     * The minimum number of points that must be in the bottom-level MP buffer before starting a profile.
     */
    protected final int minNumPointsInBottomBuffer;
    /**
     * The settings currently being used by this Talon.
     */
    @NotNull
    protected PerGearSettings currentGearSettings;
    /**
     * The most recently set setpoint.
     */
    protected double setpoint;
    /**
     * The setpoint in native units. Field to avoid garbage collection.
     */
    protected double nativeSetpoint;
    /**
     * The time at which the motion profile status was last checked. Only getting the status once per tic avoids CAN
     * traffic.
     */
    protected long timeMPStatusLastRead;
    /**
     * RPS as used in a unit conversion method. Field to avoid garbage collection.
     */
    protected Double RPS;

    public SmartMotorBase(@JsonProperty(required = true) int port,
                          @Nullable org.usfirst.frc.team449.robot.jacksonWrappers.PDP PDP,
                          @Nullable Integer encoderCPR,
                          double postEncoderGearing,
                          double feetPerRotation,
                          double updaterProcessPeriodSecs,
                          @Nullable String name,
                          @Nullable RunningLinRegComponent voltagePerCurrentLinReg, boolean fwdLimitSwitchNormallyOpen, boolean revLimitSwitchNormallyOpen, @Nullable Integer minNumPointsInBottomBuffer) {
        this.PDP = PDP;
        this.encoderCPR = encoderCPR;
        this.postEncoderGearing = postEncoderGearing;
        this.feetPerRotation = feetPerRotation;
        this.updaterProcessPeriodSecs = updaterProcessPeriodSecs;
        this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
        this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
        this.name = defaultIfNull(name, this.getClass().getName() + "_" + port);
        this.voltagePerCurrentLinReg = voltagePerCurrentLinReg;
        this.perGearSettings = new HashMap<>();
        this.minNumPointsInBottomBuffer = minNumPointsInBottomBuffer != null ? minNumPointsInBottomBuffer : 20;
    }
}

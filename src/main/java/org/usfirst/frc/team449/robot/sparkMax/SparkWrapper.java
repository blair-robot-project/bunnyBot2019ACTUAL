package org.usfirst.frc.team449.robot.sparkMax;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Notifier;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.components.RunningLinRegComponent;
import org.usfirst.frc.team449.robot.generalInterfaces.loggable.Loggable;
import org.usfirst.frc.team449.robot.generalInterfaces.shiftable.Shiftable;
import org.usfirst.frc.team449.robot.generalInterfaces.simpleMotor.SimpleMotor;
import org.usfirst.frc.team449.robot.jacksonWrappers.FPSTalon;
import org.usfirst.frc.team449.robot.other.Clock;
import org.usfirst.frc.team449.robot.other.Logger;
import org.usfirst.frc.team449.robot.other.MotionProfileData;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * todo find out if wpi's Spark or revrobotics' SparkMax should be used. Probably the latter
 * todo also, make sure the slot is 0 for everything
 * Component wrapper on the SPARK MAX motor, with unit conversions to/from FPS built in. Every non-unit-conversion
 * in this class takes arguments in post-gearing FPS.
 *
 * @see FPSTalon
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class SparkWrapper extends CANMotorControllerBase implements SimpleMotor, Shiftable, Loggable {
    /**
     * test out how many PWM cycles it should go on after
     * overcurrent, or not at all. Putting it at 0 because
     * the talon's peak current limit had a timeout of 0 ms
     */
    private static final int chopCycles = 0;

    /**
     * The SPARK-MAX that this class is a wrapper on
     */
//	@NotNull
    protected final SparkMaxMotorController canSpark;
    /**
     * Forward limit switch
     */
    protected final CANDigitalInput fwdLimitSwitch;
    /**
     * Reverse limit switch
     */
    protected final CANDigitalInput revLimitSwitch;

    /**
     * A cache for the results of querying status using {@link SparkMaxMotorController#getMotionProfileStatus(MotionProfileStatus)}.
     */
    @NotNull
    private final MotionProfileStatus motionProfileStatus;

    /**
     * The encoder that records everything. I'm assuming
     * we don't use analog (or hall effect) because the
     * constructor changes the type to quadrature if it is absolute or relative
     */
    private final CANEncoder encoder;

    /**
     *
     */
    private final Notifier executorNotifier;

    /**
     * Default constructor.
     *
     * @param port                       CAN port of this Spark.
     * @param name                       The talon's name, used for logging purposes. Defaults to talon_portnum
     * @param reverseOutput              Whether to reverse the output.
     * @param enableBrakeMode            Whether to brake or coast when stopped.
     * @param voltagePerCurrentLinReg    The component for doing linear regression to find the resistance.
     * @param PDP                        The PDP this Spark is connected to.
     * @param fwdLimitSwitchNormallyOpen Whether the forward limit switch is normally open or closed. If this is null,
     *                                   the forward limit switch is disabled.
     * @param revLimitSwitchNormallyOpen Whether the reverse limit switch is normally open or closed. If this is null,
     *                                   the reverse limit switch is disabled.
     * @param fwdSoftLimit               The forward software limit, in feet. If this is null, the forward software
     *                                   limit is disabled. Ignored if there's no encoder.
     * @param revSoftLimit               The reverse software limit, in feet. If this is null, the reverse software
     *                                   limit is disabled. Ignored if there's no encoder.
     * @param postEncoderGearing         The coefficient the output changes by after being measured by the encoder, e.g.
     *                                   this would be 1/70 if there was a 70:1 gearing between the encoder and the
     *                                   final output. Defaults to 1.
     * @param feetPerRotation            The number of feet travelled per rotation of the motor this is attached to.
     *                                   Defaults to 1.
     * @param currentLimit               The max amps this device can draw. If this is null, no current limit is used.
     * @param enableVoltageComp          Whether or not to use voltage compensation. Defaults to false.
     * @param voltageCompSamples         The number of 1-millisecond samples to use for voltage compensation. Defaults
     *                                   to 32.
     * @param feedbackDevice             The type of encoder used to measure the output velocity of this motor. Can be
     *                                   null if there is no encoder attached to this Spark.
     *                                   todo ensure that it can take both a CANAnalog sensor and a CANEncoder
     * @param encoderCPR                 The counts per rotation of the encoder on this Spark. Can be null if
     *                                   feedbackDevice is, but otherwise must have a value.
     * @param reverseSensor              Whether or not to reverse the reading from the encoder on this Spark. Ignored
     *                                   if feedbackDevice is null. Defaults to false.
     * @param perGearSettings            The settings for each gear this motor has. Can be null to use default values
     *                                   and gear # of zero. Gear numbers can't be repeated.
     * @param startingGear               The gear to start in. Can be null to use startingGearNum instead.
     * @param startingGearNum            The number of the gear to start in. Ignored if startingGear isn't null.
     *                                   Defaults to the lowest gear.
     * @param minNumPointsInBottomBuffer The minimum number of points that must be in the MP buffer before starting a
     *                                   profile. Defaults to 20.
     * @param updaterProcessPeriodSecs   The period for the Notifier that loads the next MP point if the Spark has
     *                                   attained the current one. Defaults to 0.005. TODO: Default should probably be much smaller
     * @param statusFrameRatesMillis     The update rates, in millis, for each of the Spark status frames.
     * @param controlFrameRateMillis     The update rate, in milliseconds, for each of the control frame.
     */
    @JsonCreator
    public SparkWrapper(@JsonProperty(required = true) final int port,
                        @Nullable final String name,
                        final boolean reverseOutput,
                        @JsonProperty(required = true) final boolean enableBrakeMode,
                        @NotNull @JsonProperty(required = true) final RunningLinRegComponent voltagePerCurrentLinReg,
                        @NotNull @JsonProperty(required = true) final org.usfirst.frc.team449.robot.jacksonWrappers.PDP PDP,
                        @Nullable final Boolean fwdLimitSwitchNormallyOpen,
                        @Nullable final Boolean revLimitSwitchNormallyOpen,
                        @Nullable final Double fwdSoftLimit,
                        @Nullable final Double revSoftLimit,
                        @Nullable final Double postEncoderGearing,
                        @Nullable final Double feetPerRotation,
                        @Nullable final Integer currentLimit,
                        final boolean enableVoltageComp,
                        @Nullable final Integer voltageCompSamples,
                        @Nullable final CANEncoder feedbackDevice,
                        @Nullable final Integer encoderCPR,
                        final boolean reverseSensor,
                        @Nullable final List<PerGearSettings> perGearSettings,
                        @Nullable final Shiftable.gear startingGear,
                        @Nullable final Integer startingGearNum,
                        @Nullable final Integer minNumPointsInBottomBuffer,
                        @Nullable final Double updaterProcessPeriodSecs,
                        @Nullable final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis,
                        @Nullable final // todo figure out if this is right; there doesn't seem to be any use of it in the map.yml anyways.
                                Integer controlFrameRateMillis) {
        super(PDP, feedbackDevice != null ? encoderCPR : null,
                postEncoderGearing != null ? postEncoderGearing : 1.,
                feetPerRotation != null ? feetPerRotation : 1,
                minNumPointsInBottomBuffer != null ? minNumPointsInBottomBuffer : 20,
                updaterProcessPeriodSecs != null ? updaterProcessPeriodSecs : 0.005,
                name != null ? name : ("talon_" + port),
                voltagePerCurrentLinReg,
                fwdLimitSwitchNormallyOpen != null ? fwdLimitSwitchNormallyOpen : true,
                revLimitSwitchNormallyOpen != null ? revLimitSwitchNormallyOpen : true);

        // TODO: Maybe pull this out to a parameter.
        //Instantiate the base canSpark this is a wrapper on.
        /* TODO: figure out if the first argument is the port or what.
            The first argument is the CAN arbitration ID.
            FPSTalon forwards its port argument to the single-parameter TalonSRX constructor,
            which ORs it with 0x02040000 to obtain a final arb ID.
         */
        this.canSpark = new SparkMaxMotorController(port, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Set this to false because we only use reverseOutput for slaves.
        this.canSpark.setInverted(reverseOutput);
        //Set brake mode
        this.canSpark.setIdleMode(enableBrakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

        //Make CAN calls non-blocking
        this.canSpark.setCANTimeout(0);

        //Set frame rates
        if (controlFrameRateMillis != null) {
            // Must be between 1 and 100 ms.
            this.canSpark.setControlFramePeriodMs(controlFrameRateMillis);
        }
        if (statusFrameRatesMillis != null) {
            for (final CANSparkMaxLowLevel.PeriodicFrame frame : statusFrameRatesMillis.keySet()) {
                this.canSpark.setPeriodicFramePeriod(frame, statusFrameRatesMillis.get(frame));
            }
        }

        //Initialize
        this.motionProfileStatus = new MotionProfileStatus();
        this.timeMPStatusLastRead = 0;

        //If given no gear settings, use the default values.
        if (perGearSettings == null || perGearSettings.size() == 0) {
            this.perGearSettings.put(0, new PerGearSettings());
            this.perGearSettings.get(0).getFeedForwardComponent().setController(this);
        }
        //Otherwise, map the settings to the gear they are.
        else {
            for (final PerGearSettings settings : perGearSettings) {
                settings.getFeedForwardComponent().setController(this);
                this.perGearSettings.put(settings.getGear(), settings);
            }
        }

        int currentGear;
        //If the starting gear isn't given, assume we start in low gear.
        if (startingGear == null) {
            if (startingGearNum == null) {
                currentGear = Integer.MAX_VALUE;
                for (final Integer gear : this.perGearSettings.keySet()) {
                    if (gear < currentGear) {
                        currentGear = gear;
                    }
                }
            } else {
                currentGear = startingGearNum;
            }
        } else {
            currentGear = startingGear.getNumVal();
        }
        this.currentGearSettings = this.perGearSettings.get(currentGear);

        //Only enable the limit switches if it was specified if they're normally open or closed.
        if (fwdLimitSwitchNormallyOpen != null) {
            this.fwdLimitSwitch = this.canSpark.
                    getForwardLimitSwitch(fwdLimitSwitchNormallyOpen ?
                            CANDigitalInput.LimitSwitchPolarity.kNormallyOpen :
                            CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);

            //is this the right one?
            //canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, !fwdLimitSwitchNormallyOpen);

			/*original:
			canSpark.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
					fwdLimitSwitchNormallyOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 0);*/
        } else {
            //canSpark.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
            this.fwdLimitSwitch = this.canSpark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
            this.fwdLimitSwitch.enableLimitSwitch(true);
        }
        if (revLimitSwitchNormallyOpen != null) {
            this.revLimitSwitch = this.canSpark.
                    getReverseLimitSwitch(revLimitSwitchNormallyOpen ?
                            CANDigitalInput.LimitSwitchPolarity.kNormallyOpen :
                            CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
            //is this the right one?
            //canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, !revLimitSwitchNormallyOpen);

			/*canSpark.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
					revLimitSwitchNormallyOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 0);*/
        } else {
            /*canSpark.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);*/
            this.revLimitSwitch = this.canSpark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
            this.revLimitSwitch.enableLimitSwitch(true);
        }

        //Set up the feedback device if it exists.
        if (feedbackDevice != null) {
            //CTRE encoder use RPM instead of native units, and can be used as QuadEncoders, so we switch them to avoid
            //having to support RPM.
            //todo check if only encoders or analogs are also used
            /*if (feedbackDevice.equals(FeedbackDevice.CTRE_MagEncoder_Absolute) ||
                    feedbackDevice.equals(FeedbackDevice.CTRE_MagEncoder_Relative)) {
                talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
            } else {
                canSpark.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
            }*/
            //T:I'm guessing we want quadrature instead of hall effect
            this.encoder = feedbackDevice;
            this.encoder.setInverted(reverseSensor);

            //Only enable the software limits if they were given a value and there's an encoder.
            if (fwdSoftLimit != null) {
                this.canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
                this.canSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, this.feetToEncoder(fwdSoftLimit).intValue());
            } else {
                this.canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
            }
            if (revSoftLimit != null) {
                this.canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
                this.canSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, this.feetToEncoder(revSoftLimit).intValue());
            } else {
                this.canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
            }
        } else {
            this.encoder = new CANEncoder(this.canSpark, EncoderType.kNoSensor, encoderCPR);
        }
        this.canSpark.getPIDController().setFeedbackDevice(this.encoder);

        //Set up gear-based settings.
        this.setGear(currentGear);

        //Set the current limit if it was given
        if (currentLimit != null) {
            //todo either change the parameters to include a stallLimit too,
            // or just do the one parameter setSCLimit method
            // TODO: FPSTalon isn't using the Talon's peak current limiting, so this code is fine as is.
            this.canSpark.setSmartCurrentLimit(currentLimit);
        } else {
            //If we don't have a current limit, disable current limiting.
            this.canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
            this.canSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        }

        //Enable or disable voltage comp
        this.canSpark.enableVoltageCompensation(enableVoltageComp ? CANMotorControllerBase.voltageCompensation : 0);

        // TODO: Don't think this is possible.
        // canSpark.configVoltageMeasurementFilter(voltageCompSamples != null ? voltageCompSamples : 32, 0);

        //Set up notifier for MP execution.
        this.executorNotifier = new Notifier(this.canSpark::processMotionProfile);

        // TODO: I don't think Spark supports differential control, either.
        // canSpark.selectProfileSlot(0, 0);

		/* I doubt these will be used
		if (slaveTalons != null) {
			//Set up slaves.
			for (SlaveTalon slave : slaveTalons) {
				slave.setMaster(port, enableBrakeMode, currentLimit, PDP, voltagePerCurrentLinReg.clone());
				Logger.addLoggable(slave);
			}
		}

		if (slaveVictors != null) {
			//Set up slaves.
			for (SlaveVictor slave : slaveVictors) {
				slave.setMaster(port, enableBrakeMode);
			}
		}*/
    }

    /**
     * Set the motor output voltage to a given percent of available voltage.
     *
     * @param percentVoltage percent of total voltage from [-1, 1]
     */
    public void setPercentVoltage(double percentVoltage) {
        //Warn the user if they're setting Vbus to a number that's outside the range of values.
        if (Math.abs(percentVoltage) > 1.0) {
            Logger.addEvent("WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT " + percentVoltage, this.getClass());
            percentVoltage = Math.signum(percentVoltage);
        }

        this.setpoint = percentVoltage;

        this.canSpark.set(ControlType.kVoltage, percentVoltage);
    }

    /**
     * @return The gear this subsystem is currently in.
     */
    @Override
    public int getGear() {
        return this.currentGearSettings.getGear();
    }

    /**
     * Shift to a specific gear.
     *
     * @param gear Which gear to shift to.
     */
    @Override
    public void setGear(final int gear) {
        //Set the current gear
        final Map<Integer, PerGearSettings> map = this.perGearSettings;
        this.currentGearSettings = this.perGearSettings.<PerGearSettings>get(gear);

        //Set max voltage
        //todo I am fairly sure this is the right method, but I'm not totally sure
        //  unfortunately, instead of having four methods for min and max forward
        //  and reverse outputs, it only has a range method with max forward output
        //  and min reverse output.
        //  Also, they recommend using the SPARK MAX GUI
        this.canSpark.getPIDController().setOutputRange(this.currentGearSettings.getRevPeakOutputVoltage() / 12., this.currentGearSettings.getFwdPeakOutputVoltage() / 12.);
        /*talon.configPeakOutputForward(currentGearSettings.getFwdPeakOutputVoltage() / 12., 0);
        canSpark.configPeakOutputReverse(currentGearSettings.getRevPeakOutputVoltage() / 12., 0);*/

        //Set min voltage
        /*canSpark.configNominalOutputForward(currentGearSettings.getFwdNominalOutputVoltage() / 12., 0);
        canSpark.configNominalOutputReverse(currentGearSettings.getRevNominalOutputVoltage() / 12., 0);*/

        if (this.currentGearSettings.getRampRate() != null) {
            //Set ramp rate, converting from volts/sec to seconds until 12 volts.
            this.canSpark.setClosedLoopRampRate(1 / (this.currentGearSettings.getRampRate() / 12.));
            this.canSpark.setOpenLoopRampRate(1 / (this.currentGearSettings.getRampRate() / 12.));
        } else {
            this.canSpark.setClosedLoopRampRate(0);
            this.canSpark.setOpenLoopRampRate(0);
        }

        //Set motion magic stuff
        if (this.currentGearSettings.getMotionMagicMaxVel() != null) {
            this.canSpark.getPIDController().setSmartMotionMaxVelocity(this.FPSToEncoder(this.currentGearSettings.getMotionMagicMaxVel()).intValue(), 0);
            //We can convert accel the same way we do vel because both are per second.
            this.canSpark.getPIDController().setSmartMotionMaxAccel(this.FPSToEncoder(this.currentGearSettings.getMotionMagicMaxAccel()).intValue(), 0);
        }

        //Set PID stuff
        if (this.currentGearSettings.getMaxSpeed() != null) {
            //Slot 0 velocity gains. We don't set F yet because that changes based on setpoint.
            this.canSpark.getPIDController().setP(this.currentGearSettings.getkP(), 0);
            this.canSpark.getPIDController().setI(this.currentGearSettings.getkI(), 0);
            this.canSpark.getPIDController().setD(this.currentGearSettings.getkD(), 0);

            //We set the MP gains when loading a profile so no need to do it here.
        }
    }

    /**
     * Convert from native units read by an encoder to feet moved. Note this DOES account for post-encoder gearing.
     *
     * @param nativeUnits A distance native units as measured by the encoder.
     * @return That distance in feet, or null if no encoder CPR was given.
     */
    @Nullable
    protected Double encoderToFeet(final double nativeUnits) {
        if (this.encoderCPR == null) {
            return null;
        }
        return nativeUnits / (this.encoderCPR * 4) * this.postEncoderGearing * this.feetPerRotation;
    }

    /**
     * Convert a distance from feet to encoder reading in native units. Note this DOES account for post-encoder
     * gearing.
     *
     * @param feet A distance in feet.
     * @return That distance in native units as measured by the encoder, or null if no encoder CPR was given.
     */
    @Nullable
    protected Double feetToEncoder(final double feet) {
        if (this.encoderCPR == null) {
            return null;
        }

        final double numRotations = feet / this.feetPerRotation;
        final int encoderPPR = this.encoderCPR * 4;
        return numRotations * encoderPPR / this.postEncoderGearing;
    }

    /**
     * Converts the velocity read by the talon's getVelocity() method to the FPS of the output shaft. Note this DOES
     * account for post-encoder gearing.
     *
     * @param encoderReading The velocity read from the encoder with no conversions.
     * @return The velocity of the output shaft, in FPS, when the encoder has that reading, or null if no encoder CPR
     * was given.
     */
    @Nullable
    protected Double encoderToFPS(final double encoderReading) {
        this.RPS = this.nativeToRPS(encoderReading);
        if (this.RPS == null) {
            return null;
        }
        return this.RPS * this.postEncoderGearing * this.feetPerRotation;
    }

    /**
     * Converts from the velocity of the output shaft to what the talon's getVelocity() method would read at that
     * velocity. Note this DOES account for post-encoder gearing.
     *
     * @param FPS The velocity of the output shaft, in FPS.
     * @return What the raw encoder reading would be at that velocity, or null if no encoder CPR was given.
     */
    @Nullable
    protected Double FPSToEncoder(final double FPS) {
        return this.RPSToNative((FPS / this.postEncoderGearing) / this.feetPerRotation);
    }

    /**
     * Convert from canSpark native velocity units to output rotations per second. Note this DOES NOT account for
     * post-encoder gearing.
     *
     * @param nat A velocity in canSpark native units.
     * @return That velocity in RPS, or null if no encoder CPR was given.
     */
    @Contract(pure = true)
    @Nullable
    private Double nativeToRPS(final double nat) {
        if (this.encoderCPR == null) {
            return null;
        }
        return (nat / (this.encoderCPR * 4)) * 10; //4 edges per count, and 10 100ms per second.
    }

    /**
     * Convert from output RPS to the canSpark native velocity units. Note this DOES NOT account for post-encoder
     * gearing.
     *
     * @param RPS The RPS velocity you want to convert.
     * @return That velocity in canSpark native units, or null if no encoder CPR was given.
     */
    @Contract(pure = true)
    @Nullable
    private Double RPSToNative(final double RPS) {
        if (this.encoderCPR == null) {
            return null;
        }
        return (RPS / 10) * (this.encoderCPR * 4); //4 edges per count, and 10 100ms per second.
    }

    /**
     * Set a position setpoint for the Spark.
     *
     * @param feet An absolute position setpoint, in feet.
     */
    public void setPositionSetpoint(final double feet) {
        this.setpoint = feet;

        final Double nativeSetpoint = this.feetToEncoder(feet);
        if (nativeSetpoint == null)
            throw new IllegalStateException("Cannot calculate encoder rotation count because encoder CPR is not set.");

        this.nativeSetpoint = nativeSetpoint;

        if (this.currentGearSettings.getMotionMagicMaxVel() != null) {
            // We don't know the setpoint for motion magic so we can't do fancy F stuff.
            this.canSpark.getPIDController().setFF(0, 0);
            this.canSpark.getPIDController().setReference(this.nativeSetpoint, ControlType.kSmartMotion);
        } else {
            this.canSpark.getPIDController().setFF(1023. / 12. / this.nativeSetpoint * this.currentGearSettings.getFeedForwardComponent().applyAsDouble(feet), 0);
            this.canSpark.getPIDController().setReference(this.nativeSetpoint, ControlType.kPosition);
            this.canSpark.set(ControlType.kPosition, this.nativeSetpoint);
        }
    }

    /**
     * Get the velocity of the canSpark in FPS.
     *
     * @return The canSpark's velocity in FPS, or null if no encoder CPR was given.
     */
    @Nullable
    public Double getVelocity() {
        return this.encoderToFPS(this.canSpark.getEncoder().getVelocity());
    }

    /**
     * Set the velocity for the motor to go at.
     *
     * @param velocity the desired velocity, on [-1, 1].
     */
    @Override
    public void setVelocity(final double velocity) {
        if (this.currentGearSettings.getMaxSpeed() != null) {
            this.setVelocityFPS(velocity * this.currentGearSettings.getMaxSpeed());
        } else {
            this.setPercentVoltage(velocity);
        }
    }

    /**
     * Give a velocity closed loop setpoint in FPS.
     *
     * @param velocity velocity setpoint in FPS.
     */
    protected void setVelocityFPS(final double velocity) {
        this.nativeSetpoint = this.FPSToEncoder(velocity);
        this.canSpark.getPIDController().setFF(1023. / 12. / this.nativeSetpoint * this.currentGearSettings.getFeedForwardComponent().applyAsDouble(velocity), 0);
        this.setpoint = velocity;
        this.canSpark.set(ControlType.kVelocity, this.nativeSetpoint);
    }

    /**
     * Get the current closed-loop velocity error in FPS. WARNING: will give garbage if not in velocity mode.
     *
     * @return The closed-loop error in FPS, or null if no encoder CPR was given.
     */
    @Nullable
    public Double getError() {
        return this.encoderToFPS(this.canSpark.getPIDController().
                getSmartMotionAllowedClosedLoopError(0));
    }

    /**
     * Get the current velocity setpoint of the Spark in FPS. WARNING: will give garbage if not in velocity mode.
     *
     * @return The closed-loop velocity setpoint in FPS, or null if no encoder CPR was given.
     */
    @Nullable
    public Double getSetpoint() {
        return this.setpoint;
    }

    /**
     * Get the voltage the Spark is currently drawing from the PDP.
     *
     * @return Voltage in volts.
     */
    public double getOutputVoltage() {
        // This is actually the output duty cycle.
        //return canSpark.getAppliedOutput();

        /** Implementation in {@link org.usfirst.frc.team449.robot.jacksonWrappers.FPSTalon#getOutputVoltage()}: */
        //return canTalon.getMotorOutputVoltage().

        // This method is only used in the current limiter for the climber and in logging methods,
        // so I think it's okay to leave it unimplemented for now in case this isn't correct.
        /** Either way, this is the implementation of {@link BaseMotorController#getMotorOutputVoltage()} */
        // This assumes that setpoint is the output voltage percent.
        return this.getSetpoint() * this.getBatteryVoltage();
    }

    /**
     * Get the voltage available for the Spark.
     *
     * @return Voltage in volts.
     */
    public double getBatteryVoltage() {
        return this.canSpark.getBusVoltage();
    }

    /**
     * Get the current the Spark is currently drawing from the PDP.
     *
     * @return Current in amps.
     */
    public double getOutputCurrent() {
        return this.canSpark.getOutputCurrent();
    }

    /**
     * Get the current control mode of the Spark. Please don't use this for anything other than logging.
     *
     * @return Control mode as a string.
     */
    public String getControlMode() {
        return String.valueOf(this.canSpark.getControlMode());
    }

    /**
     * Enables the motor, if applicable.
     */
    @Override
    public void enable() {
        //Not a thing anymore
    }

    /**
     * Disables the motor, if applicable.
     */
    @Override
    public void disable() {
        this.canSpark.disable();
    }

    /**
     * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
     *
     * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given gear.
     * @param gear     The number of the gear to use the max speed from to scale the velocity.
     */
    public void setGearScaledVelocity(final double velocity, final int gear) {
        if (this.currentGearSettings.getMaxSpeed() == null) {
            this.setPercentVoltage(velocity);
        } else {
            this.setVelocityFPS(this.perGearSettings.get(gear).getMaxSpeed() * velocity);
        }
    }

    /**
     * Set the velocity scaled to a given gear's max velocity. Used mostly when autoshifting.
     *
     * @param velocity The velocity to go at, from [-1, 1], where 1 is the max speed of the given gear.
     * @param gear     The gear to use the max speed from to scale the velocity.
     */
    public void setGearScaledVelocity(final double velocity, final Shiftable.gear gear) {
        this.setGearScaledVelocity(velocity, gear.getNumVal());
    }

    /**
     * @return the position of the talon in feet, or null if inches per rotation wasn't given.
     */
    public Double getPositionFeet() {
        return this.encoderToFeet(this.encoder.getPosition());
    }

    /**
     * Resets the position of the Talon to 0.
     */
    public void resetPosition() {
        this.canSpark.setSelectedSensorPosition(0, 0, 0);
    }

    /**
     * Get the status of the forwards limit switch.
     *
     * @return {@literal true} if the forwards limit switch is closed; {@literal false} if it's open or doesn't exist.
     */
    public boolean getFwdLimitSwitch() {
        /** {@link CANDigitalInput#get()} also matches the behavior of {@link com.ctre.phoenix.motorcontrol.SensorCollection#isFwdLimitSwitchClosed()} in
         * ignoring whether it is enabled. */
        return this.fwdLimitSwitchNormallyOpen == this.fwdLimitSwitch.get();
    }

    /**
     * Get the status of the reverse limit switch.
     *
     * @return True if the reverse limit switch is closed, false if it's open or doesn't exist.
     */
    public boolean getRevLimitSwitch() {
        return this.revLimitSwitchNormallyOpen == this.revLimitSwitch.get();
    }

    /**
     * A private utility method for updating motionProfileStatus with the current motion profile status. Makes sure that
     * the status is only gotten once per tick, to avoid CAN traffic overload.
     *
     * @see FPSTalon#updateMotionProfileStatus()
     */
    private void updateMotionProfileStatus() {
        if (this.timeMPStatusLastRead < Clock.currentTimeMillis()) {
            this.canSpark.getMotionProfileStatus(this.motionProfileStatus);
            this.timeMPStatusLastRead = Clock.currentTimeMillis();
        }
    }

    /**
     * Whether this talon is ready to start running a profile.
     *
     * @return True if minNumPointsInBottomBuffer points have been loaded or the top buffer is empty, false otherwise.
     */
    public boolean readyForMP() {
        this.updateMotionProfileStatus();
        return this.motionProfileStatus.topBufferCnt == 0 || this.motionProfileStatus.btmBufferCnt >= this.minNumPointsInBottomBuffer;
    }

    /**
     * Whether this talon has finished running a profile.
     *
     * @return True if the active point in the talon is the last point, false otherwise.
     */
    public boolean MPIsFinished() {
        this.updateMotionProfileStatus();
        return this.motionProfileStatus.isLast;
    }

    /**
     * Reset all MP-related stuff, including all points loaded in both the API and bottom-level buffers.
     */
    private void clearMP() {
        this.canSpark.clearMotionProfileHasUnderrun(0);
        this.canSpark.clearMotionProfileTrajectories();
    }

    /**
     * Starts running the loaded motion profile.
     */
    public void startRunningMP() {
        canSpark.set(ControlType.kSmartMotion, SetValueMotionProfile.Enable.value);
    }

    /**
     * Disables the talon and loads the given profile into the talon.
     *
     * @param data The profile to load.
     */
    public void loadProfile(final MotionProfileData data) {
        this.executorNotifier.stop();
        //Reset the Spark
        this.disable();
        this.clearMP();

        // TODO: (this code was copied from FPSTalon) This doesn't make sense. Primitives are located on the stack.
        // TODO: The variable is also never assigned to. What's up with this?
        //Declare this out here to avoid garbage collection
        double feedforward;

        //Set proper PID constants
        if (data.isBackwards()) {
            if (data.isVelocityOnly()) {
                this.canSpark.getPIDController().setP(0, 1);
                this.canSpark.getPIDController().setI(0, 1);
                this.canSpark.getPIDController().setD(0, 1);
            } else {
                this.canSpark.getPIDController().setP(this.currentGearSettings.getMotionProfilePRev(), 1);
                this.canSpark.getPIDController().setI(this.currentGearSettings.getMotionProfileIRev(), 1);
                this.canSpark.getPIDController().setD(this.currentGearSettings.getMotionProfileDRev(), 1);
            }
        } else {
            if (data.isVelocityOnly()) {
                this.canSpark.getPIDController().setP(0, 1);
                this.canSpark.getPIDController().setI(0, 1);
                this.canSpark.getPIDController().setD(0, 1);
            } else {
                this.canSpark.getPIDController().setP(this.currentGearSettings.getMotionProfilePFwd(), 1);
                this.canSpark.getPIDController().setI(this.currentGearSettings.getMotionProfileIFwd(), 1);
                this.canSpark.getPIDController().setD(this.currentGearSettings.getMotionProfileDFwd(), 1);
            }
        }

        this.canSpark.getPIDController().setFF(1023. / 12., 1);

        //Only call position getter once
        final double startPosition = data.resetPosition() ? 0 : this.getPositionFeet();

        //Set point time
        this.canSpark.configMotionProfileTrajectoryPeriod(data.getPointTimeMillis(), 0);

        //Load in profiles
        for (int i = 0; i < data.getData().length; ++i) {
            final TrajectoryPoint point = new TrajectoryPoint();
            //Have to set this so the Spark doesn't throw a null pointer. May be fixed in a future release.
            point.timeDur = data.getPointTimeMillis();

            //Set parameters that are true for all points
            point.profileSlotSelect0 = 1;        // gain selection, we always put MP gains in slot 1.

            // Set all the fields of the profile point
            point.position = this.feetToEncoder(startPosition + (data.getData()[i][0] * (data.isBackwards() ? -1 : 1)));

            feedforward = this.currentGearSettings.getFeedForwardComponent().calcMPVoltage(data.getData()[i][0],
                    data.getData()[i][1], data.getData()[i][2]);
            Logger.addEvent("VelPlusAccel: " + feedforward, this.getClass());
            point.velocity = feedforward;

            //Doing vel+accel shouldn't lead to impossible setpoints, so if it does, we log so we know to change either the profile or kA.
            if (Math.abs(feedforward) > 12) {
                System.out.println("Point " + Arrays.toString(data.getData()[i]) + " has an unattainable velocity+acceleration setpoint!");
                Logger.addEvent("Point " + Arrays.toString(data.getData()[i]) + " has an unattainable velocity+acceleration setpoint!", this.getClass());
            }
            point.zeroPos = i == 0 && data.resetPosition(); // If it's the first point, set the encoder position to 0.
            point.isLastPoint = (i + 1) == data.getData().length; // If it's the last point, isLastPoint = true

            // Send the point to the Spark's buffer
            this.canSpark.pushMotionProfileTrajectory(point);
        }
        this.executorNotifier.startPeriodic(this.updaterProcessPeriodSecs);
    }


    /**
     * Get the headers for the data this subsystem logs every loop.
     *
     * @return An N-length array of String labels for data, where N is the length of the Object[] returned by getData().
     */
    @NotNull
    @Override
    public String[] getHeader() {
        return new String[] {
                "velocity",
                "position",
                "setpoint",
                "error",
                "battery_voltage",
                "voltage",
                "current",
                "control_mode",
                "gear",
                "resistance"
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
        this.voltagePerCurrentLinReg.addPoint(this.getOutputCurrent(), this.PDP.getVoltage() - this.getBatteryVoltage());
        return new Object[] {
                this.getVelocity(),
                this.getPositionFeet(),
                this.getSetpoint(),
                this.getError(),
                this.getBatteryVoltage(),
                this.getOutputVoltage(),
                this.getOutputCurrent(),
                this.getControlMode(),
                this.getGear(),
                -this.voltagePerCurrentLinReg.getSlope()
        };
    }

    /**
     * Get the name of this object.
     *
     * @return A string that will identify this object in the log file.
     */
    @NotNull
    @Override
    public String getLogName() {
        return this.name;
    }

    /**
     * Does nothing, for there is no separate top-level buffer.
     *
     * @see FPSTalon#processMotionProfileBuffer()
     */
    protected void processMotionProfileBuffer() {
        return;
    }

    /**
     * An object representing the canSpark settings that are different for each gear.
     */
    protected static class PerGearSettings extends CANMotorControllerBase.PerGearSettings {

        /**
         * The gear number this is the settings for.
         */
        private final int gear;

        /**
         * The forward and reverse peak output voltages.
         */
        private final double fwdPeakOutputVoltage, revPeakOutputVoltage;

        /**
         * The forwards and reverse nominal output voltages.
         */
        private final double fwdNominalOutputVoltage, revNominalOutputVoltage;

        /**
         * The ramp rate, in volts/sec. null means no ramp rate.
         */
        @Nullable
        private final Double rampRate;

        /**
         * The maximum speed of the motor in this gear, in FPS. Used for throttle scaling.
         */
        @Nullable
        private final Double maxSpeed;

        /**
         * The PID constants for the motor in this gear. Ignored if maxSpeed is null.
         */
        private final double kP, kI, kD;

        /**
         * The forwards PID constants for motion profiles in this gear. Ignored if maxSpeed is null.
         */
        private final double motionProfilePFwd, motionProfileIFwd, motionProfileDFwd;

        /**
         * The reverse PID constants for motion profiles in this gear. Ignored if maxSpeed is null.
         */
        private final double motionProfilePRev, motionProfileIRev, motionProfileDRev;

        /**
         * The component for calculating feedforwards in closed-loop control modes. Ignored if maxSpeed is null.
         */
        @NotNull
        private final FeedForwardComponent feedForwardComponent;

        /**
         * The maximum velocity for motion magic mode, in FPS. Can be null to not use motion magic.
         */
        @Nullable
        private final Double motionMagicMaxVel;

        /**
         * The maximum acceleration for motion magic mode, in FPS per second.
         */
        private final double motionMagicMaxAccel;

        /**
         * Default constructor.
         *
         * @param gearNum                 The gear number this is the settings for. Ignored if gear isn't null.
         * @param gear                    The gear this is the settings for. Can be null.
         * @param fwdPeakOutputVoltage    The peak output voltage for closed-loop modes in the forwards direction, in
         *                                volts. Defaults to 12.
         * @param revPeakOutputVoltage    The peak output voltage for closed-loop modes in the reverse direction, in
         *                                volts. Defaults to -fwdPeakOutputVoltage.
         * @param fwdNominalOutputVoltage The minimum output voltage for closed-loop modes in the forwards direction.
         *                                This does not rescale, it just sets any output below this voltage to this
         *                                voltage. Defaults to 0.
         * @param revNominalOutputVoltage The minimum output voltage for closed-loop modes in the reverse direction.
         *                                This does not rescale, it just sets any output below this voltage to this
         *                                voltage. Defaults to -fwdNominalOutputVoltage.
         * @param rampRate                The ramp rate, in volts/sec. Can be null, and if it is, no ramp rate is used.
         * @param maxSpeed                The maximum speed of the motor in this gear, in FPS. Used for throttle
         *                                scaling. Ignored if kVFwd is null. Calculated from the drive characterization
         *                                terms if null.
         * @param kP                      The proportional PID constant for the motor in this gear. Ignored if kVFwd is
         *                                null. Defaults to 0.
         * @param kI                      The integral PID constant for the motor in this gear. Ignored if kVFwd is
         *                                null. Defaults to 0.
         * @param kD                      The derivative PID constant for the motor in this gear. Ignored if kVFwd is
         *                                null. Defaults to 0.
         * @param motionProfilePFwd       The proportional PID constant for forwards motion profiles in this gear.
         *                                Ignored if kVFwd is null. Defaults to 0.
         * @param motionProfileIFwd       The integral PID constant for forwards motion profiles in this gear. Ignored
         *                                if kVFwd is null. Defaults to 0.
         * @param motionProfileDFwd       The derivative PID constant for forwards motion profiles in this gear. Ignored
         *                                if kVFwd is null. Defaults to 0.
         * @param motionProfilePRev       The proportional PID constant for reverse motion profiles in this gear.
         *                                Ignored if kVFwd is null. Defaults to motionProfilePFwd.
         * @param motionProfileIRev       The integral PID constant for reverse motion profiles in this gear. Ignored if
         *                                kVFwd is null. Defaults to motionProfileIFwd.
         * @param motionProfileDRev       The derivative PID constant for reverse motion profiles in this gear. Ignored
         *                                if kVFwd is null. Defaults to motionProfileDFwd.
         * @param feedForwardComponent    The component for calculating feedforwards in closed-loop control modes.
         *                                Ignored if maxSpeed is null. Defaults to no feedforward.
         * @param motionMagicMaxVel       The maximum velocity for motion magic mode, in FPS. Can be null to not use
         *                                motion magic.
         * @param motionMagicMaxAccel     The maximum acceleration for motion magic mode, in FPS per second.
         */
        @JsonCreator
        public PerGearSettings(final int gearNum,
                               @Nullable final Shiftable.gear gear,
                               @Nullable final Double fwdPeakOutputVoltage,
                               @Nullable final Double revPeakOutputVoltage,
                               @Nullable final Double fwdNominalOutputVoltage,
                               @Nullable final Double revNominalOutputVoltage,
                               @Nullable final Double rampRate,
                               @Nullable final Double maxSpeed,
                               final double kP,
                               final double kI,
                               final double kD,
                               final double motionProfilePFwd,
                               final double motionProfileIFwd,
                               final double motionProfileDFwd,
                               @Nullable final Double motionProfilePRev,
                               @Nullable final Double motionProfileIRev,
                               @Nullable final Double motionProfileDRev,
                               @Nullable final FeedForwardComponent feedForwardComponent,
                               @Nullable final Double motionMagicMaxVel,
                               final double motionMagicMaxAccel) {
            this.gear = gear != null ? gear.getNumVal() : gearNum;
            this.fwdPeakOutputVoltage = fwdPeakOutputVoltage != null ? fwdPeakOutputVoltage : 12;
            this.revPeakOutputVoltage = revPeakOutputVoltage != null ? revPeakOutputVoltage : -this.fwdPeakOutputVoltage;
            this.fwdNominalOutputVoltage = fwdNominalOutputVoltage != null ? fwdNominalOutputVoltage : 0;
            this.revNominalOutputVoltage = revNominalOutputVoltage != null ? revNominalOutputVoltage : -this.fwdNominalOutputVoltage;
            this.rampRate = rampRate;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.motionProfilePFwd = motionProfilePFwd;
            this.motionProfileIFwd = motionProfileIFwd;
            this.motionProfileDFwd = motionProfileDFwd;
            this.motionProfilePRev = motionProfilePRev != null ? motionProfilePRev : this.motionProfilePFwd;
            this.motionProfileIRev = motionProfileIRev != null ? motionProfileIRev : this.motionProfileIFwd;
            this.motionProfileDRev = motionProfileDRev != null ? motionProfileDRev : this.motionProfileDFwd;
            this.feedForwardComponent = feedForwardComponent != null ? feedForwardComponent : FeedForwardComponent.getZeroFeedForward();
            this.maxSpeed = maxSpeed;
            this.motionMagicMaxVel = motionMagicMaxVel;
            this.motionMagicMaxAccel = motionMagicMaxAccel;
        }

        /**
         * Empty constructor that uses all default options.
         */
        public PerGearSettings() {
            this(0, null, null, null, null, null, null, null, 0, 0, 0, 0, 0, 0, null, null, null, null, null, 0);
        }

        /**
         * @return The gear number this is the settings for.
         */
        public int getGear() {
            return this.gear;
        }

        /**
         * @return The peak output voltage for closed-loop modes in the forwards direction, in volts.
         */
        public double getFwdPeakOutputVoltage() {
            return this.fwdPeakOutputVoltage;
        }

        /**
         * @return The peak output voltage for closed-loop modes in the reverse direction, in volts.
         */
        public double getRevPeakOutputVoltage() {
            return this.revPeakOutputVoltage;
        }

        /**
         * @return The minimum output voltage for closed-loop modes in the forwards direction. This does not rescale, it
         * just sets any output below this voltage to this voltage.
         */
        public double getFwdNominalOutputVoltage() {
            return this.fwdNominalOutputVoltage;
        }

        /**
         * @return The minimum output voltage for closed-loop modes in the reverse direction. This does not rescale, it
         * just sets any output below this voltage to this voltage.
         */
        public double getRevNominalOutputVoltage() {
            return this.revNominalOutputVoltage;
        }

        /**
         * @return The ramp rate, in volts/sec.
         */
        @Nullable
        public Double getRampRate() {
            return this.rampRate;
        }

        /**
         * @return The maximum speed of the motor in this gear, in FPS.
         */
        @Nullable
        public Double getMaxSpeed() {
            return this.maxSpeed;
        }

        /**
         * @return The proportional PID constant for the motor in this gear.
         */
        public double getkP() {
            return this.kP;
        }

        /**
         * @return The integral PID constant for the motor in this gear.
         */
        public double getkI() {
            return this.kI;
        }

        /**
         * @return The derivative PID constant for the motor in this gear.
         */
        public double getkD() {
            return this.kD;
        }

        /**
         * @return The proportional PID constant for motion profiles in this gear.
         */
        public double getMotionProfilePFwd() {
            return this.motionProfilePFwd;
        }

        /**
         * @return The integral PID constant for motion profiles in this gear.
         */
        public double getMotionProfileIFwd() {
            return this.motionProfileIFwd;
        }

        /**
         * @return The derivative PID constant for motion profiles in this gear.
         */
        public double getMotionProfileDFwd() {
            return this.motionProfileDFwd;
        }

        /**
         * @return The proportional PID constant for reverse motion profiles in this gear.
         */
        public double getMotionProfilePRev() {
            return this.motionProfilePRev;
        }

        /**
         * @return The integral PID constant for reverse motion profiles in this gear.
         */
        public double getMotionProfileIRev() {
            return this.motionProfileIRev;
        }

        /**
         * @return The derivative PID constant for reverse motion profiles in this gear.
         */
        public double getMotionProfileDRev() {
            return this.motionProfileDRev;
        }

        /**
         * @return The component for calculating feedforwards in closed-loop control modes.
         */
        @NotNull
        public FeedForwardComponent getFeedForwardComponent() {
            return this.feedForwardComponent;
        }

        /**
         * @return The maximum velocity for motion magic mode, in FPS. Can be null to not use motion magic.
         */
        @Nullable
        public Double getMotionMagicMaxVel() {
            return this.motionMagicMaxVel;
        }

        /**
         * @return The maximum acceleration for motion magic mode, in FPS per second.
         */
        public double getMotionMagicMaxAccel() {
            return this.motionMagicMaxAccel;
        }
    }
}

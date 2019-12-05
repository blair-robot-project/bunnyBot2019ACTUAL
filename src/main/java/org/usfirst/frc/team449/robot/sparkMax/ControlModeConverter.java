package org.usfirst.frc.team449.robot.sparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

// Needs serious looking-over.

/**
 * Converts between Talon's {@link ControlMode} and Spark's {@link ControlType}.
 */
public class ControlModeConverter {
    static ControlType toSpark(final ControlMode m) {
        switch (m) {
            case PercentOutput:
                return ControlType.kDutyCycle;
            case Position:
                return ControlType.kPosition;
            case Velocity:
                return ControlType.kVelocity;
            case Current:
                return ControlType.kCurrent;
            case Follower:
                throw new UnsupportedOperationException();
            case MotionProfile:
                return ControlType.kSmartMotion;
            case MotionMagic:
            case MotionMagicArc:
            case MotionProfileArc:
            case Disabled:
            default:
                throw new UnsupportedOperationException();
        }
    }

    static ControlMode toTalon(final ControlType e) {
        switch (e) {
            case kDutyCycle:
                return ControlMode.PercentOutput;
            case kVelocity:
                return ControlMode.Velocity;
            case kVoltage:
                throw new UnsupportedOperationException();
            case kPosition:
                return ControlMode.Position;
            case kSmartMotion:
                throw new UnsupportedOperationException();
            case kCurrent:
                return ControlMode.Current;
            case kSmartVelocity:
            default:
                throw new UnsupportedOperationException();
        }
    }
}
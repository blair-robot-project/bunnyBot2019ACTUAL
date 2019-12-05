package org.usfirst.frc.team449.robot.sparkMax;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANError;

// TODO: Look over this.

/**
 * Converts between Talon's {@link ErrorCode} and Spark's {@link CANError}.
 */
public class ErrorConverter {
    static CANError toSpark(final ErrorCode e) {
        switch (e) {
            case OK:
                return CANError.kOk;
            case GeneralError:
                return CANError.kError;
            case RxTimeout:
            case TxTimeout:
                return CANError.kTimeout;
            case NotImplemented:
                return CANError.kNotImplmented;
            case FirmwareTooOld:
                return CANError.kFirmwareTooOld;
            case UnexpectedArbId:
                return CANError.kParamInvalidID;
            case InvalidParamValue:
                return CANError.kParamInvalid;
            case InvalidHandle:
                return CANError.kInvalid;

            default:
                return CANError.kError;
        }
    }

    static ErrorCode toTalon(final CANError e) {
        switch (e) {
            case kOk:
                return ErrorCode.OK;
            case kError:
                return ErrorCode.GeneralError;
            case kTimeout:
                return ErrorCode.GeneralError;
            case kNotImplmented:
            case kParamNotImplementedDeprecated:
                return ErrorCode.NotImplemented;
            case kFirmwareTooOld:
                return ErrorCode.FirmwareTooOld;
            case kParamInvalidID:
                return ErrorCode.UnexpectedArbId;
            case kParamInvalid:
                return ErrorCode.InvalidParamValue;
            case kInvalid:
                return ErrorCode.InvalidHandle;

            default:
                return ErrorCode.GeneralError;
        }
    }
}

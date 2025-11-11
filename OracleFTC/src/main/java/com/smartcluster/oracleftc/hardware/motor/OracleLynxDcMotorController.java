package com.smartcluster.oracleftc.hardware.motor;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetPositionResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerParamsState;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.smartcluster.oracleftc.hardware.OracleHardware;
import com.smartcluster.oracleftc.hardware.OracleLynxModule;
import com.smartcluster.oracleftc.utils.OracleLastKnown;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class OracleLynxDcMotorController extends LynxDcMotorController  {
    private static final String TAG="LynxDcMotorController";
    public static final int apiMotorFirst = LynxConstants.INITIAL_MOTOR_PORT;
    public static final double apiPowerFirst = -1.0;
    public static final double apiPowerLast  = 1.0;

    private static class MotorProperties
    {
        // We have caches of values that we *could* read from the controller, and need to
        // do so if the cache is invalid
        public OracleLastKnown<Double> lastKnownPower              = new OracleLastKnown<>(false);
        public OracleLastKnown<Integer>                      lastKnownTargetPosition     = new OracleLastKnown<>(false);
        public OracleLastKnown<DcMotor.RunMode>              lastKnownMode               = new OracleLastKnown<>(false);
        public OracleLastKnown<DcMotor.ZeroPowerBehavior>    lastKnownZeroPowerBehavior  = new OracleLastKnown<>(false);
        public OracleLastKnown<Boolean>                      lastKnownEnable             = new OracleLastKnown<>(false);
        public OracleLastKnown<Double>                       lastKnownCurrentAlert       = new OracleLastKnown<>(false); // mA
        public OracleLastKnown<PIDFCoefficients>             lastKnownPIDFCoefficients   = new OracleLastKnown<>(false);
        public OracleLastKnown<PIDCoefficients> lastKnownPIDCoefficients    = new OracleLastKnown<>(false);
        // The remainder of the data is authoritative, here
        public MotorConfigurationType motorType = MotorConfigurationType.getUnspecifiedMotorType();
        public MotorConfigurationType                  internalMotorType = null;
        public Map<DcMotor.RunMode, ExpansionHubMotorControllerParamsState> desiredPIDParams = new ConcurrentHashMap<>();
        public Map<DcMotor.RunMode, ExpansionHubMotorControllerParamsState> originalPIDParams = new ConcurrentHashMap<>();

    }
    public static MotorProperties[] createPropertiesArray(int length)
    {
        MotorProperties[] properties  = new MotorProperties[length];
        for(int i=0;i<length;i++) properties[i]=new MotorProperties();
        return properties;
    }
    private MotorProperties[] motorProperties;

    public OracleLynxDcMotorController(OracleLynxModule module) throws RobotCoreException, InterruptedException {
        super(null, module);
    }

    @Override
    protected void doHook() {
        motorProperties = createPropertiesArray(LynxConstants.NUMBER_OF_MOTORS);
    }
    // Standard functionality

    @Override
    public boolean isMotorEnabled(int motor) {

        if(OracleHardware.optimize==null) return super.isMotorEnabled(motor);
        if(motorProperties[motor].lastKnownEnable.isValid())
        {
            return motorProperties[motor].lastKnownEnable.getValue();
        }else {
            LynxGetMotorChannelEnableCommand command = new LynxGetMotorChannelEnableCommand(getModule(),motor);
            try {
                LynxGetMotorChannelEnableResponse response = command.sendReceive();
                boolean result = response.isEnabled();
                motorProperties[motor].lastKnownEnable.setValue(result);
                return result;
            }
            catch (InterruptedException | RuntimeException | LynxNackException e)
            {
                handleException(e);
            }
        }
        return LynxUsbUtil.makePlaceholderValue(true);
    }

    @Override
    public synchronized void setMotorVelocity(int motor, double ticksPerSecond) {

        if(OracleHardware.optimize==null) {
            super.setMotorVelocity(motor,ticksPerSecond);
            return;
        }
        switch (getMotorMode(motor))
        {
            case RUN_USING_ENCODER:
            case RUN_TO_POSITION:
                break;
            default:
                setMotorMode(motor, DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int iTicksPerSecond = Range.clip((int)Math.round(ticksPerSecond),
                LynxSetMotorTargetVelocityCommand.apiVelocityFirst,
                LynxSetMotorTargetVelocityCommand.apiVelocityLast);

        try {
            LynxSetMotorTargetVelocityCommand command = new LynxSetMotorTargetVelocityCommand(getModule(), motor, iTicksPerSecond);
            if (DEBUG) RobotLog.vv(TAG, "setMotorVelocity: mod=%d motor=%d iPower=%d", getModuleAddress(), motor, iTicksPerSecond);
            getModule().sendCommand(command);
            internalSetMotorEnable(motor, true);
        }
        catch (InterruptedException | RuntimeException |
               LynxUnsupportedCommandException e)
        {
            handleException(e);
        }
    }

    @Override
    public synchronized void setMotorVelocity(int motor, double angularRate, AngleUnit unit) {

        if(OracleHardware.optimize==null)
        {
            super.setMotorVelocity(motor,angularRate,unit);
            return;
        }
        double degreesPerSecond     = UnnormalizedAngleUnit.DEGREES.fromUnit(unit.getUnnormalized(), angularRate);
        double revolutionsPerSecond = degreesPerSecond / 360.0;
        double ticksPerSecond       = motorProperties[motor].motorType.getTicksPerRev() * revolutionsPerSecond;

        setMotorVelocity(motor + apiMotorFirst, ticksPerSecond);
    }



    @SuppressWarnings({"deprecation"})
    @Override
    public synchronized void setPIDFCoefficients(int motor, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {

        if(OracleHardware.optimize==null) {
            super.setPIDFCoefficients(motor,mode,pidfCoefficients);
            return;
        }
        mode = mode.migrate();

        // Remember that we've overridden the values specified by the motor type so that we don't
        // mistakenly undo this effect in updateMotorParams()
        rememberPIDParams(motor, new ExpansionHubMotorControllerParamsState(mode, pidfCoefficients));

        // Actually change the values
        if (!internalSetPIDFCoefficients(motor, mode, pidfCoefficients))
        {
            throw new UnsupportedOperationException(Misc.formatForUser("setting of pidf coefficients not supported: motor=%d mode=%s pidf=%s", motor+apiMotorFirst, mode, pidfCoefficients));
        }
    }


    @Override
    public PIDCoefficients getPIDCoefficients(int motor, DcMotor.RunMode mode) {

        if(OracleHardware.optimize==null) return super.getPIDCoefficients(motor, mode);
        if(motorProperties[motor].lastKnownPIDCoefficients.isValid())
        {
            return motorProperties[motor].lastKnownPIDCoefficients.getValue();
        }else {
            LynxGetMotorPIDControlLoopCoefficientsCommand command = new LynxGetMotorPIDControlLoopCoefficientsCommand(getModule(),motor,mode);
            try {
                LynxGetMotorPIDControlLoopCoefficientsResponse response = command.sendReceive();
                return new PIDCoefficients(
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD())
                );
            }
            catch (LynxNackException e)
            {
                if (e.getNack().getNackReasonCode() == LynxNack.StandardReasonCode.PARAM2)
                {
                    // There's a non-zero F coefficient; ignore it
                    PIDFCoefficients pidfCoefficients = getPIDFCoefficients(motor + apiMotorFirst, mode);
                    return new PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);
                }
                handleException(e);
            }
            catch (InterruptedException|RuntimeException e)
            {
                handleException(e);
            }
        }
        return LynxUsbUtil.makePlaceholderValue(new PIDCoefficients());
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(int motor, DcMotor.RunMode mode) {

        if(OracleHardware.optimize==null) return super.getPIDFCoefficients(motor,mode);

        if(motorProperties[motor].lastKnownPIDFCoefficients.isValid())
        {
            return motorProperties[motor].lastKnownPIDFCoefficients.getValue();
        }else {
            if (getModule().isCommandSupported(LynxGetMotorPIDFControlLoopCoefficientsCommand.class))
            {
                LynxGetMotorPIDFControlLoopCoefficientsCommand command = new LynxGetMotorPIDFControlLoopCoefficientsCommand(this.getModule(), motor, mode);
                try {
                    LynxGetMotorPIDFControlLoopCoefficientsResponse response = command.sendReceive();
                    PIDFCoefficients result =  new PIDFCoefficients(
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD()),
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getF()),
                            response.getInternalMotorControlAlgorithm().toExternal());
                    motorProperties[motor].lastKnownPIDFCoefficients.setValue(result);
                    return result;
                }
                catch (InterruptedException|RuntimeException|LynxNackException e)
                {
                    handleException(e);
                }
                return LynxUsbUtil.makePlaceholderValue(new PIDFCoefficients());
            }
            else
            {
                return new PIDFCoefficients(getPIDCoefficients(motor, mode));
            }
        }
    }

    @Override
    public void setMotorTargetPosition(int motor, int position, int tolerance) {

        if(OracleHardware.optimize==null) {
            super.setMotorTargetPosition(motor,position,tolerance);
            return;
        }
        LynxSetMotorTargetPositionCommand command = new LynxSetMotorTargetPositionCommand(getModule(), motor, position, tolerance);
        try {
            getModule().sendCommand(command);
        }
        catch (LynxUnsupportedCommandException|InterruptedException|RuntimeException e)
        {
            handleException(e);
        }
    }

    @Override
    public double getMotorCurrent(int motor, CurrentUnit unit) {

        if(OracleHardware.optimize==null) return super.getMotorCurrent(motor,unit);
        LynxGetADCCommand command = new LynxGetADCCommand(getModule(), LynxGetADCCommand.Channel.motorCurrent(motor), LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            LynxGetADCResponse response = command.sendReceive();
            return unit.convert(response.getValue(), CurrentUnit.MILLIAMPS);
        }
        catch (InterruptedException|RuntimeException|LynxNackException e)
        {
            handleException(e);
        }
        return LynxUsbUtil.makePlaceholderValue(0.0);
    }

    @Override
    public double getMotorCurrentAlert(int motor, CurrentUnit unit) {

        if(OracleHardware.optimize==null) return super.getMotorCurrentAlert(motor,unit);
        if(motorProperties[motor].lastKnownCurrentAlert.isValid())
        {
            return unit.convert(motorProperties[motor].lastKnownCurrentAlert.getValue(), CurrentUnit.MILLIAMPS);
        }else {
            LynxGetMotorChannelCurrentAlertLevelCommand command = new LynxGetMotorChannelCurrentAlertLevelCommand(getModule(), motor);
            try
            {
                LynxGetMotorChannelCurrentAlertLevelResponse response = command.sendReceive();
                double limit = response.getCurrentLimit();
                motorProperties[motor].lastKnownCurrentAlert.setValue(limit);
                return unit.convert(limit, CurrentUnit.MILLIAMPS);
            }
            catch (InterruptedException|RuntimeException|LynxNackException e)
            {
                handleException(e);
            }
            return LynxUsbUtil.makePlaceholderValue(0.0);
        }
    }

    @Override
    public void setMotorCurrentAlert(int motor, double current, CurrentUnit unit) {
        super.setMotorCurrentAlert(motor,current,unit);
    }

    @Override
    public void setMotorMode(int motor, DcMotor.RunMode mode) {

        if(OracleHardware.optimize==null) {
            super.setMotorMode(motor,mode);
            return;
        }
        if (!motorProperties[motor].lastKnownMode.isValue(mode))
        {
            // Get the current power so we can preserve across the change.
            Double prevPower = motorProperties[motor].lastKnownPower.getNonTimedValue();
            if (prevPower == null)
            {
                prevPower = getMotorPower(motor);
            }

            LynxCommand<? extends LynxMessage> command;
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;
            if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            {
                // Stop the motor, but not in such a way that we disrupt the last known
                // power, since we need to restore same when we come out of this mode.
                setMotorPower(motor, 0);
                command = new LynxResetMotorEncoderCommand(getModule(), motor);
            }
            else
            {
                zeroPowerBehavior = getMotorZeroPowerBehavior(motor);
                command = new LynxSetMotorChannelModeCommand(getModule(), motor, mode, zeroPowerBehavior);
            }
            try {
                if (DEBUG) RobotLog.vv(TAG, "setMotorChannelMode: mod=%d motor=%d mode=%s power=%f zero=%s",
                        getModuleAddress(), motor, mode.toString(), prevPower, zeroPowerBehavior.toString());
                getModule().sendCommand(command);

                // Ok, remember that mode. Note we need to set it before we call internalSetMotorPower()
                motorProperties[motor].lastKnownMode.setValue(mode);

                // re-issue current motor power to ensure it's correct for this mode
                setMotorPower(motor, prevPower);
            }
            catch (InterruptedException | RuntimeException |
                   LynxUnsupportedCommandException e)
            {
                handleException(e);
            }
        }
    }

    @Override
    public DcMotor.RunMode getMotorMode(int motor) {

        if(OracleHardware.optimize==null) return super.getMotorMode(motor);
        if(motorProperties[motor].lastKnownMode.isValid())
        {
            return motorProperties[motor].lastKnownMode.getValue();
        }else {
            LynxGetMotorChannelModeCommand command = new LynxGetMotorChannelModeCommand(getModule(), motor);
            try {
                LynxGetMotorChannelModeResponse response = command.sendReceive();
                DcMotor.RunMode result = response.getMode();
                motorProperties[motor].lastKnownMode.setValue(result);
                return result;
            }
            catch (InterruptedException|RuntimeException|LynxNackException e)
            {
                handleException(e);
            }
        }
        return LynxUsbUtil.makePlaceholderValue(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void setMotorPower(int motor, double power) {

        if(OracleHardware.optimize==null)
        {
            super.setMotorPower(motor,power);
            return;
        }
        power = Range.clip(power, apiPowerFirst, apiPowerLast);
        int iPower = 0;
        if (motorProperties[motor].lastKnownPower.updateValue(power) || OracleHardware.optimize.consistentLoopTimes())
        {
            DcMotor.RunMode mode = getMotorMode(motor);
            LynxCommand<? extends LynxMessage> command = null;
            switch (mode)
            {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER:
                {
                    // Scale 'power' to configured maximum motor speed. This is mostly for legacy
                    // compatibility, as setMotorVelocity exposes this more directly.
                    power = Math.signum(power) * Range.scale(Math.abs(power), 0, apiPowerLast, 0, getDefaultMaxMotorSpeed(motor));
                    iPower = (int)power;
                    command = new LynxSetMotorTargetVelocityCommand(getModule(), motor, iPower);
                    break;
                }
                case RUN_WITHOUT_ENCODER:
                {
                    power = Range.scale(power, apiPowerFirst, apiPowerLast, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast);
                    iPower = (int)power;
                    command = new LynxSetMotorConstantPowerCommand(getModule(), motor, iPower);
                    break;
                }
            }
            try {
                if (command != null)
                {
                    if (DEBUG) RobotLog.vv(TAG, "setMotorPower: mod=%d motor=%d iPower=%d",getModuleAddress(), motor, iPower);
                    getModule().sendCommand(command);
                    setMotorEnable(motor);
                }
            }
            catch (InterruptedException|RuntimeException|LynxUnsupportedCommandException e)
            {
                handleException(e);
            }
        }
    }

    @Override
    public double getMotorPower(int motor) {

        if(OracleHardware.optimize==null) return super.getMotorPower(motor);
        if(motorProperties[motor].lastKnownPower.isValid())
        {
            return motorProperties[motor].lastKnownPower.getValue();
        }else {
            DcMotor.RunMode mode = getMotorMode(motor);
            switch (mode) {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER: {
                    LynxGetMotorTargetVelocityCommand command = new LynxGetMotorTargetVelocityCommand(this.getModule(), motor);
                    try {

                        LynxGetMotorTargetVelocityResponse response = command.sendReceive();
                        int iVelocity = response.getVelocity();
                        double result = Math.signum(iVelocity) * Range.scale(Math.abs(iVelocity), 0, getDefaultMaxMotorSpeed(motor), 0, apiPowerLast);
                        motorProperties[motor].lastKnownPower.setValue(result);
                        return result;


                    } catch (InterruptedException | LynxNackException e) {
                        handleException(e);
                    }
                }
                case RUN_WITHOUT_ENCODER:
                default:
                {
                    LynxGetMotorConstantPowerCommand command = new LynxGetMotorConstantPowerCommand(this.getModule(), motor);
                    try {

                        LynxGetMotorConstantPowerResponse response = command.sendReceive();
                        int iPower = response.getPower();
                        double result = Range.scale(iPower, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast, apiPowerFirst, apiPowerLast);
                        motorProperties[motor].lastKnownPower.setValue(result);
                        return result;

                    } catch (InterruptedException | LynxNackException e) {
                        handleException(e);
                    }
                }
            }
        }
        return LynxUsbUtil.makePlaceholderValue(0.0);
    }


//
//    @Override
//    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
//
//        if(OracleHardware.optimize==null) return super.getMotorZeroPowerBehavior(motor);
//        return getMotorZeroPowerBehaviorAsync(motor).join();
//    }
//
//    @Override
//    public boolean getMotorPowerFloat(int motor) {
//
//        if(OracleHardware.optimize==null) return super.getMotorPowerFloat(motor);
//        return getMotorPowerFloatAsync(motor).join();
//    }
//
//    @Override
//    public void setMotorTargetPosition(int motor, int position) {
//        super.setMotorTargetPosition(motor,position);
//    }
//
//    @Override
//    public int getMotorTargetPosition(int motor) {
//
//        if(OracleHardware.optimize==null) return super.getMotorTargetPosition(motor);
//        return getMotorTargetPositionAsync(motor).join();
//    }
    @Override
    public void setMotorEnable(int motor) {

        if(OracleHardware.optimize==null)
        {
            super.setMotorEnable(motor);
            return;
        }
        internalSetMotorEnable(motor, true);
    }

    @Override
    public void setMotorDisable(int motor) {

        if(OracleHardware.optimize==null)
        {
            super.setMotorDisable(motor);
            return;
        }
        internalSetMotorEnable(motor, false);
    }

    private void internalSetMotorEnable(int motor, boolean enabled)
    {

        if(motorProperties[motor].lastKnownEnable.updateValue(enabled))
        {
            LynxSetMotorChannelEnableCommand command = new LynxSetMotorChannelEnableCommand(getModule(), motor, enabled);
            try {
                command.send();
            }catch (LynxNackException  e)
            {
                LynxNack.ReasonCode reason = e.getNack().getNackReasonCode();
                if (reason == LynxNack.StandardReasonCode.MOTOR_NOT_CONFIG_BEFORE_ENABLED)
                {
                    throw new TargetPositionNotSetException();
                }
                else
                {
                    handleException(e);
                }
                handleException(e);
            }catch (InterruptedException e)
            {
                handleException(e);
            }
        }
    }
    @Override
    public void forgetLastKnown() {

        super.forgetLastKnown();
        if(motorProperties!=null)
            for(MotorProperties motorProperties: motorProperties)
            {
                motorProperties.lastKnownEnable.invalidate();
                motorProperties.lastKnownPower.invalidate();
                motorProperties.lastKnownCurrentAlert.invalidate();
                motorProperties.lastKnownPIDCoefficients.invalidate();
                motorProperties.lastKnownPIDFCoefficients.invalidate();
                motorProperties.lastKnownTargetPosition.invalidate();
                motorProperties.lastKnownZeroPowerBehavior.invalidate();
            }
    }

    @Override
    public synchronized void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        super.setMotorZeroPowerBehavior(motor, zeroPowerBehavior);
    }




    @Override
    public void resetDeviceConfigurationForOpMode() {
        forgetLastKnown();
    }

    @Override
    public synchronized MotorConfigurationType getMotorType(int motor) {

        return motorProperties[motor].motorType;
    }

    @Override
    public synchronized void setMotorType(int motor, MotorConfigurationType motorType) {

        motorProperties[motor].motorType = motorType;
        if (motorProperties[motor].internalMotorType==null)
        {
            // First one is the system setting the type
            motorProperties[motor].internalMotorType = motorType;
        }

        // Remember parameterization, overriding any user-specified values.
        if (motorType.hasExpansionHubVelocityParams())
        {
            rememberPIDParams(motor, motorType.getHubVelocityParams());
        }
        if (motorType.hasExpansionHubPositionParams())
        {
            rememberPIDParams(motor, motorType.getHubPositionParams());
        }
        updateMotorParams(motor);
    }

    @Override
    protected void rememberPIDParams(int motorZ, ExpansionHubMotorControllerParamsState params) {

        motorProperties[motorZ].desiredPIDParams.put(params.mode, params);
    }

    @Override
    protected void updateMotorParams(int motorZ) {

        for (ExpansionHubMotorControllerParamsState params : motorProperties[motorZ].desiredPIDParams.values())
        {
            if (!params.isDefault())
            {
                internalSetPIDFCoefficients(motorZ, params.mode, params.getPidfCoefficients());
            }
        }
    }

    @Override
    protected int getDefaultMaxMotorSpeed(int motorZ) {

        return motorProperties[motorZ].motorType.getAchieveableMaxTicksPerSecondRounded();
    }
    @SuppressWarnings({"deprecation"})
    @Override
    protected boolean internalSetPIDFCoefficients(int motorZ, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {

        boolean supported = true;

        // If this is the very first time that we've set coefficients, then remember what params were in use before we set anything
        if (!motorProperties[motorZ].originalPIDParams.containsKey(mode))
        {
            PIDFCoefficients originalCoefficients = getPIDFCoefficients(motorZ + apiMotorFirst, mode);
            motorProperties[motorZ].originalPIDParams.put(mode, new ExpansionHubMotorControllerParamsState(mode, originalCoefficients));
        }

        int p = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.p);
        int i = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.i);
        int d = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.d);
        int f = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.f);

        if (mode==DcMotor.RunMode.RUN_TO_POSITION && pidfCoefficients.algorithm != MotorControlAlgorithm.LegacyPID)
        {
            // In non-legacy run to position avoid having the user give coefficients that cause double-integration (our
            // fw runs a position loop, that then runs a velocity loop)
            if (pidfCoefficients.i != 0 || pidfCoefficients.d != 0 || pidfCoefficients.f != 0)
            {
                supported = false;
                RobotLog.ww(TAG, "using unreasonable coefficients for RUN_TO_POSITION: setPIDFCoefficients(%d, %s, %s)", motorZ+apiMotorFirst, mode, pidfCoefficients);
            }
        }

        if (supported)
        {
            if (getModule().isCommandSupported(LynxSetMotorPIDFControlLoopCoefficientsCommand.class))
            {
                LynxSetMotorPIDFControlLoopCoefficientsCommand.InternalMotorControlAlgorithm algorithm = LynxSetMotorPIDFControlLoopCoefficientsCommand.InternalMotorControlAlgorithm.fromExternal(pidfCoefficients.algorithm);
                LynxSetMotorPIDFControlLoopCoefficientsCommand command = new LynxSetMotorPIDFControlLoopCoefficientsCommand(getModule(), motorZ, mode, p, i, d, f, algorithm);
                try {
                    command.send();
                }
                catch (InterruptedException | RuntimeException |
                       LynxNackException e)
                {
                    supported = handleException(e);
                }
            }
            else if (f == 0 && pidfCoefficients.algorithm == MotorControlAlgorithm.LegacyPID)
            {
                LynxSetMotorPIDControlLoopCoefficientsCommand command = new LynxSetMotorPIDControlLoopCoefficientsCommand(getModule(), motorZ, mode, p, i, d);
                try {
                    command.send();
                }
                catch (InterruptedException|RuntimeException|LynxNackException e)
                {
                    supported = handleException(e);
                }
            }
            else
            {
                supported = false;
                RobotLog.ww(TAG, "not supported: setPIDFCoefficients(%d, %s, %s)", motorZ+apiMotorFirst, mode, pidfCoefficients);
            }
        }

        return supported;
    }
    @Override
    public void floatHardware() {
        super.floatHardware();
    }
}

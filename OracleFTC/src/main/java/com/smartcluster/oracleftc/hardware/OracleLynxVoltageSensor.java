package com.smartcluster.oracleftc.hardware;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.LastKnown;

@SuppressWarnings({"unused"})
public class OracleLynxVoltageSensor extends LynxVoltageSensor {

    public enum OracleLynxVoltageSensorPolicy {
        CACHED,
        MANUAL,
        NAIVE
    }
    public static double normalizationVoltage = 12.0;

    private static final Object lock = new Object();

    private OracleLynxVoltageSensorPolicy policy = OracleLynxVoltageSensorPolicy.NAIVE;

    public OracleLynxVoltageSensor(Context context, LynxModule module)
        throws RobotCoreException, InterruptedException {
        super(context, module);
        voltageCache.setValue(12.0);
    }

    public void setPolicy(OracleLynxVoltageSensorPolicy policy) {
        if (policy != null) {
            this.policy = policy;
        }
    }

    public OracleLynxVoltageSensorPolicy getPolicy() {
        return policy;
    }

    private double cacheFreshness = 300;
    private LastKnown<Double> voltageCache=new LastKnown<>(cacheFreshness);

    public void setVoltageCacheFreshness(double msFreshness) {
        this.cacheFreshness = msFreshness;
        if (policy == OracleLynxVoltageSensorPolicy.CACHED) {
            synchronized (lock) {
                double cachedVoltage = super.getVoltage();
                voltageCache = new LastKnown<>(cacheFreshness);
                voltageCache.setValue(cachedVoltage);
            }
        }
    }

    public double getVoltageCacheFreshness() {
        return cacheFreshness;
    }

    public double getVoltageManual() {
        synchronized (lock) {
            double result = 12.0;
            switch (policy) {
                case MANUAL:
                case NAIVE:
                    result = super.getVoltage();
                    voltageCache.setValue(result);
                    break;
                case CACHED:
                    if (!voltageCache.isValid()) {
                        result = super.getVoltage();
                        voltageCache.setValue(result);
                    } else {
                        result = voltageCache.getRawValue();
                    }

            }
            return result;
        }
    }

    @Override
    public double getVoltage() {

        synchronized (lock) {
            double result = 12.0;
            switch (policy) {
                case MANUAL:
                    Double cachedVoltage = voltageCache.getNonTimedValue();
                    if (cachedVoltage != null) {
                        result = cachedVoltage;
                    }
                    break;
                case NAIVE:
                    result = super.getVoltage();
                    voltageCache.setValue(result);
                    break;
                case CACHED:
                    if (!voltageCache.isValid()) {
                        result = super.getVoltage();
                        voltageCache.setValue(result);
                    } else {
                        result = voltageCache.getNonTimedValue();
                    }
                    break;
            }
            return result;
        }

    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        cacheFreshness = 1000;
        policy = OracleLynxVoltageSensorPolicy.NAIVE;
        voltageCache.invalidate();
    }
}

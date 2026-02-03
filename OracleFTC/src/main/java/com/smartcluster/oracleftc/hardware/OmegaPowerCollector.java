package com.smartcluster.oracleftc.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;
import java.util.List;

public class OmegaPowerCollector {

    private final OpMode mode;
    private OracleLynxVoltageSensor voltageSensor;
    List<LynxModule> lynxModules;

    public class BulkValues
    {
        // BulkValues
        public final HashMap<Integer, Double> DcMotorValues = new HashMap<Integer, Double>()
        {{
            put(0, 0.0); put(1, 0.0); put(2, 0.0); put(3, 0.0);
            put(4, 0.0); put(5, 0.0); put(6, 0.0); put(7, 0.0);
        }}; //First 4 are supposedly from Control hub, the next 4 from Expansion Hub

        public final HashMap<Integer, Double> ServoValues = new HashMap<Integer, Double>()
        {{
            put(0, 0.0); put(1, 0.0); put(2, 0.0); put(3, 0.0);
            put(4, 0.0); put(5, 0.0); put(6, 0.0); put(7, 0.0);
        }}; //First 4 are supposedly from Control hub, the next 4 from Expansion Hub

        public double getServoValue(int port)
        {
            return ServoValues.get(port);
        }
        public double setServoValue(int port, double val)
        {
            return ServoValues.put(port, val);
        }

        public double getDcMotorValue(int port)
        {
            return DcMotorValues.get(port);
        }
        public double setDcMotorValue(int port, double val)
        {
            return DcMotorValues.put(port, val);
        }
    }

    public BulkValues bulkValues = new BulkValues(); // Where our power will be cached;

    public OmegaPowerCollector(OpMode mode)
    {
        this.mode = mode;
        voltageSensor = mode.hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(100);

        lynxModules = mode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public double calculateNormalizedVoltage(double nominalVoltage)
    {
        return nominalVoltage/voltageSensor.getVoltage();
    }

    public void read()
    {
        for (LynxModule lynxModule : lynxModules)
        {
            lynxModule.clearBulkCache();
            lynxModule.getBulkData();
        }
    }

    public void write()
    {
        // Give me components, ports, values OR I QUIT/DO NOTHING!
    }
}

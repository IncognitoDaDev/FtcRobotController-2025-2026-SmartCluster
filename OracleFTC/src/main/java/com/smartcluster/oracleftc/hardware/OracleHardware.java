package com.smartcluster.oracleftc.hardware;

import android.content.Context;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.smartcluster.oracleftc.utils.ReflectionUtils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@SuppressWarnings({"unused"})
public class OracleHardware implements OpModeManagerNotifier.Notifications {
    private static final String TAG = "OracleHardware";
    private static final OracleHardware instance = new OracleHardware();
    private OpModeManagerImpl opModeManager;
    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        RobotLog.ii(TAG, "attachEventLoop: Attached OracleHardware to event loop");
        instance.opModeManager=eventLoop.getOpModeManager();
        eventLoop.getOpModeManager().registerListener(instance);
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        RobotLog.ii(TAG, "onOpModePreInit: Enabling OracleHardware optimizations");
            HardwareMap hardwareMap = opMode.hardwareMap;

            Map<HardwareDevice, Set<String>> deviceNames = ReflectionUtils.getFieldValue(opMode.hardwareMap, "deviceNames");
//            Map<LynxModule, OracleLynxModule> replacements = new HashMap<>();

//            for(LynxModule lynxModule: hardwareMap.getAll(LynxModule.class))
//            {
//                if(lynxModule instanceof OracleLynxModule) continue; // We have already replaced the LynxModule
//
//                // Get name of LynxModule
//                String deviceName= Objects.requireNonNull(deviceNames.get(lynxModule)).iterator().next();
//
//
//                // Use reflection to initialize the replacement OracleLynxModule
//                OracleLynxModule oracleLynxModule = OracleLynxModule.fromLynxModule(lynxModule);
//                assert oracleLynxModule!=null;
//
//                // In order to swap xxx-LynxModules we need to re-link the LynxUsbDevice with the
//                // OracleLynxModule
//                LynxUsbDevice lynxUsbDevice =oracleLynxModule.getLynxUsbDevice().getDelegationTarget();
//                lynxUsbDevice.removeConfiguredModule(lynxModule);
//
//
//                // Since v8.20, the addConfiguredModule method was removed, so in order to achieve
//                // the same thing, we need to use reflection
//                ConcurrentHashMap<Integer, LynxModule> knownModules = (ReflectionUtils.getFieldValue(lynxUsbDevice, "knownModules"));
//                assert knownModules != null;
//
//                knownModules.put(oracleLynxModule.getModuleAddress(), oracleLynxModule);
//
//
//                // Record the replacement
//                replacements.put(lynxModule, oracleLynxModule);
//                // Swap the modules in the hardware map
//                hardwareMap.remove(deviceName, lynxModule);
//                hardwareMap.put(deviceName, oracleLynxModule);
//
//            }

            boolean oracleVoltageSensorAlreadyAdded = false;
            for(LynxVoltageSensor voltageSensor: hardwareMap.getAll(LynxVoltageSensor.class))
            {
                if(voltageSensor instanceof OracleLynxVoltageSensor) {
                    oracleVoltageSensorAlreadyAdded= true;
                    continue;
                }
                String deviceName =
                        Objects.requireNonNull(Objects.requireNonNull(deviceNames).get(voltageSensor))
                                .iterator().next();
                hardwareMap.voltageSensor.remove(deviceName);
            }
            if(!oracleVoltageSensorAlreadyAdded)
            {
                try {
                    OracleLynxVoltageSensor oracleLynxVoltageSensor =
                            new OracleLynxVoltageSensor(AppUtil.getDefContext(), hardwareMap.getAll(LynxModule.class).iterator().next());
                    hardwareMap.put("OracleLynxVoltageSensor", oracleLynxVoltageSensor);
                    hardwareMap.voltageSensor.put("OracleLynxVoltageSensor",oracleLynxVoltageSensor);
                } catch (RobotCoreException | InterruptedException e) {
                    RobotLog.logStackTrace(e);
                }
            }

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {

    }


}

package org.firstinspires.ftc.teamcode.subsystem;
//iar avem variabile care vor modificate
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotorFeedforward;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

import org.firstinspires.ftc.teamcode.roadrunner.Localizer;

@Config
public class Turret extends Subsystem {

    // Hardware
    private final DcMotorImplEx shooter1, shooter2;
    private final ServoImplEx mainhood, lever;
    private final OracleLynxVoltageSensor voltageSensor;

    // Motion profiles
    public static TrapezoidalMotionProfile hoodMotionProfile = new TrapezoidalMotionProfile(12, 16, 16);
    public static TrapezoidalMotionProfile leverMotionProfile = new TrapezoidalMotionProfile(12, 16, 16);

    // Servo actuators
    public final ServoActuator hoodact, leveract;

    // Flywheel control
    public static MotorFeedforward flywheelFeedforward = new MotorFeedforward(0.135000, 0.000170, 0);
    //
    //optional: Kv (Velocity): 0.000173
    public static PIDController flywheelPID = new PIDController( 0.000006, 0, 0.0000008, 0.5);
    public static LowPassFilter velocityFilter = new LowPassFilter(0.5);
    private double targetVelocity = 0; // RPM
    private static final double RPM_TOLERANCE = 100;

    // Distance-based velocity calculation
// Velocity linear regression constants
    private static final double VELOCITY_SLOPE = 5.687797;
    private static final double VELOCITY_INTERCEPT = 2400.620519;

    // Hood angle linear regression constants
    private static final double HOOD_SLOPE = 0.002533;
    private static final double HOOD_INTERCEPT = 0.024227;

    private boolean inZone;

    public static double HOOD_MIN_POSITION = 0.0;
    public static double HOOD_MAX_POSITION = 0.45;
    public static double LEVER_BLOCK_POSITION = 0.0;  // Position that blocks the flywheel
    public static double LEVER_RELEASE_POSITION = 0.15; // Position that allows shooting

    public Turret(OpMode opMode) {
        super(opMode);

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();

        // Initialize flywheel motors
        shooter1 = hardwareMap.get(DcMotorImplEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2 = hardwareMap.get(DcMotorImplEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        mainhood = hardwareMap.get(ServoImplEx.class, "hood");
        mainhood.setDirection(Servo.Direction.REVERSE);
        lever = hardwareMap.get(ServoImplEx.class, "lever");

        // Hood actuator
        hoodact = new ServoActuator(this, "hood", hoodMotionProfile, mainhood) {
            @Override
            public Command reset() {
                return new InstantCommand(() -> {
                    setTarget(HOOD_MIN_POSITION);
                    mainhood.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target) {
                target = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, target));
                this.target.set(target);
                return true;
            }
        };

        // Lever actuator (blocks/releases flywheel)
        leveract = new ServoActuator(this, "lever", leverMotionProfile, lever) {
            @Override
            public Command reset() {
                return new InstantCommand(() -> {
                    setTarget(LEVER_BLOCK_POSITION);
                    lever.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target) {
                // Lever only needs two positions: block or release
                this.target.set(target);
                return true;
            }
        };


    }

    /**
     * Get current flywheel velocity in RPM
     */
    public double getCurrentVelocity() {
        return velocityFilter.update((shooter2.getVelocity() / 28) * 60);
    }

    // Add this to your Turret.java class
    public void setRawPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    /**
     * Set target flywheel velocity with safety limits
     */
    public void setTargetVelocity(double velocity) {
        targetVelocity = Math.max(0, Math.min(6000, velocity));
    }

    public Command setVelocityByDistance(Localizer localizer, Pose2d corner) {
        return Command.builder()
                .init(() -> inZone = true)
                .update(() -> {
                    localizer.update();
                    Pose2d currentPose = localizer.getPose();

                    // Calculate distance to corner in cm
                    double dx = corner.position.x - currentPose.position.x;
                    double dy = corner.position.y - currentPose.position.y;
                    double distance = Math.sqrt(dx * dx + dy * dy) * 2.54;

                    // Calculate velocity based on distance
                    double velocity = VELOCITY_SLOPE * distance + VELOCITY_INTERCEPT;

                    // Calculate hood position based on distance
                    double hoodPosition = HOOD_SLOPE * distance + HOOD_INTERCEPT;
                    hoodact.setTarget(hoodPosition);

                    // Update flywheel power
                    double currentVelocity = getCurrentVelocity();
                    double power = flywheelPID.update(velocity, currentVelocity)
                            + flywheelFeedforward.update(velocity, 0);
                    power *= (Robot.nominalVoltage / voltageSensor.getVoltage());

                    shooter1.setPower(power);
                    shooter2.setPower(power);

                    // Check if robot is in valid shooting zone
                    double currentX = currentPose.position.x;
                    double currentY = currentPose.position.y;
                    if (currentY >= Math.abs(currentX) + 9 * 1.41
                            || (currentY > -46 + 9 * 1.41 && Math.abs(currentX) < 23 + 9 * 1.41)) {
                        inZone = false;
                    }
                })
                .finished(() -> !inZone)
                .build();
    }

    /**
     * Block the flywheel with lever
     */
    public Command blockShooter() {
        return new InstantCommand(() -> leveract.setTarget(LEVER_BLOCK_POSITION));
    }

    /**
     * Release the lever to allow shooting
     */
    public Command releaseShooter() {
        return new InstantCommand(() -> leveract.setTarget(LEVER_RELEASE_POSITION));
    }

    /**
     * Main update command - runs flywheel control and actuator updates
     */
    public Command update() {
        return new ParallelCommand(
                hoodact.update(),
                leveract.update(),
                Command.builder()
                        .update(() -> {
                            double currentVelocity = getCurrentVelocity();
                            double power = flywheelPID.update(targetVelocity, currentVelocity)
                                    + flywheelFeedforward.update(targetVelocity, 0);
                            power *= (Robot.nominalVoltage / voltageSensor.getVoltage());

                            shooter1.setPower(power);
                            shooter2.setPower(power);
                        })
                        .requires(this)
                        .build()
        );
    }

    /**
     * Wait for flywheel to reach target RPM (with timeout)
     */
    public Command WaitForRPM(double maxMilliseconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(timer::reset)
                .update(() -> {
                    double error = Math.abs(targetVelocity - getCurrentVelocity());
                    telemetry.addData("Velocity Error", error);
                    telemetry.addData("Target Velocity", targetVelocity);
                    telemetry.addData("Current Velocity", getCurrentVelocity());
                })
                .finished(() -> {
                    double error = Math.abs(targetVelocity - getCurrentVelocity());
                    return error < RPM_TOLERANCE || timer.milliseconds() > maxMilliseconds;
                })
                .build();
    }

    /**
     * Reset turret to safe starting position
     */
    public Command reset() {
        return new SequentialCommand(
                hoodact.reset(),
                leveract.reset()
        );
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ExpansionHubOnly;
    }
}
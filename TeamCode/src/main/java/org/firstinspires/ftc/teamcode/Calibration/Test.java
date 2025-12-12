package org.firstinspires.ftc.teamcode.Calibration;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.MotionProfile;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp
public class Test extends LinearOpMode {
    public DcMotor rotation, turret;
    public static int position = 0;
    public static double speed = 0;
    public static int tolerance = 2;
    public Encoder rotate;
    public final double MOTOR_TO_TURRET_RATIO = 260.0 / 48;
    public final double ENCODER_TICKS_PER_ROTATION = 384.5 * MOTOR_TO_TURRET_RATIO;
    public final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION / 360;
    public static PIDController pid = new PIDController(0, 0, 0);
    public static TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(6000, 10000, 12000);
    private final CommandScheduler scheduler = new CommandScheduler();

    public static double Kv = 1.1;
    public static double Ka = 0.2;
    public static double Ks = 0.001;
    FeedforwardCoefficients coefficients = new FeedforwardCoefficients(Kv, Ka, Ks);
    BasicFeedforward shootController = new BasicFeedforward(coefficients);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime resetTime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(this, "Turret");
        Command.run(turret.reset());

        waitForStart();
        scheduler.schedule(
                new ParallelCommand(
                        turret.update()
                )
        );
        while (opModeIsActive() && !isStopRequested()) {
            //turret.set HELP ME 🤗

            double shootPower = shootController.calculate(0,speed, mp.maxAcceleration);
            if(gamepad2.x)position = 100;


            AtomicReference<DualNum<Time>> lastPosition = new AtomicReference<>(turret.rotate.getCurrentPosition().div(ENCODER_TICKS_PER_DEGREE));
            resetTime.reset();

            if (turret.rotate.getCurrentPosition().get(0) <= tolerance || turret.rotate.getCurrentPosition().get(0) >= -tolerance) {
                turret.rot.setPower(0);
            }

            DualNum<Time> currentPosition = turret.rotate.getCurrentPosition();

            final double distance = position - currentPosition.get(0);
            DualNum<Time> mop = mp.getMotionState(Math.abs(distance),
                    resetTime.seconds());
            double power = pid.update(mop.get(0) * Math.signum(distance),
                    turret.rotate.getCurrentPosition().get(0));

            turret.rot.setTargetPosition(position);
            turret.rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.rot.setPower(power);
            turret.setShooter(speed/6000);

            telemetry.addData("Input speed ", speed/6000);
            telemetry.addData("Turret speed", turret.getSpeed());
            telemetry.addData("Input position", position);
            telemetry.addData("Current position", turret.rotate.getCurrentPosition().get(0));
            telemetry.update();

        }
    }
}


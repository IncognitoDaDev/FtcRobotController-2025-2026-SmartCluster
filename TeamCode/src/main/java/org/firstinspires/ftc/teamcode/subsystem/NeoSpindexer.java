package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.function.Supplier;

public class NeoSpindexer extends Subsystem {
    public static PIDController rotaryPID = new PIDController(0.008, 0, 0.000365);//new PIDController(0.00032, 0.000000015, 0.000013, 0);
    public static double Tolerance = 2;
    public static double ThirdTurn = 120;
    public static class CachedSensor
    {
        private ColorType.IdentityObject Front = ColorType.IdentityObject.EMPTY;;
        private ColorType.IdentityObject Left = ColorType.IdentityObject.EMPTY;;
        private ColorType.IdentityObject Right = ColorType.IdentityObject.EMPTY;;

        public void setFront(ColorType.IdentityObject obj)
        {
            if (obj != ColorType.IdentityObject.EMPTY)
                Front = obj;
        }

        public ColorType.IdentityObject getFront() { return Front; }

        public void setRight(ColorType.IdentityObject obj)
        {
            if (obj != ColorType.IdentityObject.EMPTY)
                Right = obj;
        }

        public ColorType.IdentityObject getRight() { return Right; }

        public void setLeft(ColorType.IdentityObject obj)
        {
            if (obj != ColorType.IdentityObject.EMPTY)
                Left = obj;
        }

        public ColorType.IdentityObject getLeft() { return Left; }

        public void reset()
        {
            Front = ColorType.IdentityObject.EMPTY;
            Left = ColorType.IdentityObject.EMPTY;
            Right = ColorType.IdentityObject.EMPTY;
        }
    }



        public final CRServoImplEx servoDexRight, servoDexLeft;
        public final ServoImplEx servoFlapperRight;
        public final RevColorSensorV3 rotaryColorSensorF;

        public final LynxI2cColorRangeSensor rotaryColorSensorR, rotaryColorSensorL;
        public final Encoder rotaryEncoder;

        public final OracleLynxVoltageSensor voltageSensor;

        //private OracleLynxVoltageSensor voltageSensor;


        public double rotaryTargetPos = 0;

        public final ServoActuator flapper;
        public static double flapperDownVal = 0.51, flapperUpVal = 1.0;
        public static ColorType[] Order = {
                ColorType.Nothing,ColorType.Nothing,ColorType.Nothing,ColorType.Nothing
        };//Al patrulea spatiu este pentru a face rotatii mai usoare
        public org.firstinspires.ftc.teamcode.subsystem.Spindex.CachedSensor cachedSensor;
        public NeoSpindexer (OpMode mode)
        {
            super(mode);

            servoDexRight = hardwareMap.get(CRServoImplEx.class, "dexRight");
            servoDexLeft = hardwareMap.get(CRServoImplEx.class, "dexLeft");
            servoFlapperRight=hardwareMap.get(ServoImplEx.class,"flapperRight");
            //servoFlapperLeft=hardwareMap.get(ServoImplEx.class,"flapperLeft");
            rotaryColorSensorF = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");
            rotaryColorSensorR = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorR");
            rotaryColorSensorL = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorL");
            rotaryEncoder = new RawEncoder(hardwareMap.get(DcMotorEx.class,"intakeMotor"));

            //rotaryEncoder.setDirection(Encoder.Direction.REVERSE);

            cachedSensor = new org.firstinspires.ftc.teamcode.subsystem.Spindex.CachedSensor();

            voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
            voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
            voltageSensor.setVoltageCacheFreshness(300);

            flapper = new ServoActuator(this, "flapper", new TrapezoidalMotionProfile(12, 16, 12), servoFlapperRight)
            {
                @Override
                public Command reset()
                {
                    setTarget(flapperDownVal);
                    return new InstantCommand(() ->
                    {
                        servoFlapperRight.setPosition(this.target.get());
                        //servoFlapperLeft.setPosition(this.target.get());
                    });
                }

                @Override
                public boolean setTarget(double target)
                {
                    this.target.set(target);
                    return true;
                }
            };

        }


        public void FlapperDown()
        {
            flapper.setTarget(flapperDownVal);
        }
        public Supplier<Boolean> flapperIsDown(){
            return () -> servoFlapperRight.getPosition()==flapperDownVal;
        }
        public void FlapperUp()
        {
            flapper.setTarget(flapperUpVal);
        }

        /*
         Se spune că oamenii devin naivi în fața generalităților,
         dar se îmblânzesc în fața micilor detalii,
         așa că hai să prețuim și să celebrăm aceste detalii
        */
        public boolean IdentifyColor(RevColorSensorV3 sensor, ColorType[] ColorCheck)
        {
            for(ColorType check : ColorCheck)
            {
                int ok = 1;
                if (check.v3.RED_THRESHOLD[0] > sensor.red() || check.v3.RED_THRESHOLD[1] < sensor.red()) ok = 0;
                if (check.v3.GREEN_THRESHOLD[0] > sensor.green() || check.v3.GREEN_THRESHOLD[1] < sensor.green()) ok = 0;
                if (check.v3.BLUE_THRESHOLD[0] > sensor.blue() || check.v3.BLUE_THRESHOLD[1] < sensor.blue()) ok = 0;

                if (ok == 1) return true;
            }


            return false;
        }

        public boolean IdentifyColor(LynxI2cColorRangeSensor sensor, ColorType[] ColorCheck)
        {
            int r = sensor.red(), g = sensor.green(), b = sensor.blue();
            if (sensor == rotaryColorSensorR)
            {
                r *= 2;
                g *= 2;
                b *= 2;
            }

            for(ColorType check : ColorCheck)
            {
                int ok = 1;
                if (check.v2.RED_THRESHOLD[0] > r || check.v2.RED_THRESHOLD[1] < r) ok = 0;
                if (check.v2.GREEN_THRESHOLD[0] > g || check.v2.GREEN_THRESHOLD[1] < g) ok = 0;
                if (check.v2.BLUE_THRESHOLD[0] > b || check.v2.BLUE_THRESHOLD[1] < b) ok = 0;

                if (ok == 1) return true;
            }

            return false;
        }

        public ColorType.IdentityObject IdentifyColor(RevColorSensorV3 sensor)
        {
            ColorType[] colorCheck = {ColorType.Purple, ColorType.Green, ColorType.Wall, ColorType.Nothing};
            for(ColorType check : colorCheck)
            {
                int ok = 1;
                if (check.v3.RED_THRESHOLD[0] > sensor.red() || check.v3.RED_THRESHOLD[1] < sensor.red()) ok = 0;
                if (check.v3.GREEN_THRESHOLD[0] > sensor.green() || check.v3.GREEN_THRESHOLD[1] < sensor.green()) ok = 0;
                if (check.v3.BLUE_THRESHOLD[0] > sensor.blue() || check.v3.BLUE_THRESHOLD[1] < sensor.blue()) ok = 0;

                if (ok == 1){
                    OrderChange(Order,check);
                    OrderLeft(Order);
                    return check.identity;
                }
            }

            return ColorType.IdentityObject.EMPTY;
        }

        public ColorType.IdentityObject IdentifyColor(LynxI2cColorRangeSensor sensor)
        {
            int r = sensor.red(), g = sensor.green(), b = sensor.blue();
            if (sensor == rotaryColorSensorR)
            {
                r *= 2;
                g *= 2;
                b *= 2;
            }

            ColorType[] colorCheck = {ColorType.Purple, ColorType.Green, ColorType.Wall, ColorType.Nothing};
            for(ColorType check : colorCheck)
            {
                int ok = 1;
                if (check.v2.RED_THRESHOLD[0] > r || check.v2.RED_THRESHOLD[1] < r) ok = 0;
                if (check.v2.GREEN_THRESHOLD[0] > g || check.v2.GREEN_THRESHOLD[1] < g) ok = 0;
                if (check.v2.BLUE_THRESHOLD[0] > b || check.v2.BLUE_THRESHOLD[1] < b) ok = 0;

                if (ok == 1) return check.identity;
            }

            return ColorType.IdentityObject.EMPTY;
        }

        public void sortPurple()
        {
            if (cachedSensor.getLeft() == ColorType.IdentityObject.PURPLE)
            {
                SwitchMode(-1);
            }
            if (cachedSensor.getRight() == ColorType.IdentityObject.PURPLE)
            {
                SwitchMode(1);
            }
            if (cachedSensor.getFront() == ColorType.IdentityObject.PURPLE)
            {
                setTarget(rotaryTargetPos + 180);
            }
        }

        public void sortGreen()
        {
            if (cachedSensor.getLeft() == ColorType.IdentityObject.GREEN)
            {
                SwitchMode(-1);
            }
            if (cachedSensor.getRight() == ColorType.IdentityObject.GREEN)
            {
                SwitchMode(1);
            }
            if (cachedSensor.getFront() == ColorType.IdentityObject.GREEN)
            {
                setTarget(rotaryTargetPos + 180);
            }
        }

        public void sortAny()
        {
            if (cachedSensor.getLeft() == ColorType.IdentityObject.PURPLE || cachedSensor.getLeft() == ColorType.IdentityObject.GREEN)
            {
                SwitchMode(-1);
            }
            if (cachedSensor.getRight() == ColorType.IdentityObject.PURPLE || cachedSensor.getRight() == ColorType.IdentityObject.GREEN)
            {
                SwitchMode(1);
            }
            if (cachedSensor.getFront() == ColorType.IdentityObject.PURPLE || cachedSensor.getFront() == ColorType.IdentityObject.GREEN)
            {
                setTarget(rotaryTargetPos + 180);
            }
        }

        public Supplier<Boolean> sortEmpty()
        {
            if (cachedSensor.getLeft() == ColorType.IdentityObject.EMPTY)
            {
                SwitchMode(-1);
                return () -> true;
            }
            if (cachedSensor.getRight() == ColorType.IdentityObject.EMPTY)
            {
                SwitchMode(1);
                return () -> true;
            }
        if (IdentifyColor(rotaryColorSensorF, new ColorType[]{ColorType.Nothing}))
        {
            setTarget(rotaryTargetPos + 180);
            return ()->true;
//        } // We want to move ball to an empty space from front position

        }
            return () -> false;
        }


        public void FixOrientationForIntake()
        {
            setPosition(rotaryTargetPos - rotaryTargetPos%ThirdTurn);
        }
        public void NextSpace(){
            setPosition(rotaryTargetPos + ThirdTurn);
        }
        public void PrevSpace(){
            setPosition(rotaryTargetPos - ThirdTurn);
        }
        public void SwitchMode(int direction){
            setPosition(rotaryTargetPos - rotaryTargetPos%ThirdTurn*direction);
        }
        public ColorType[] OrderChange(ColorType[] Order,ColorType color){
            if(color!=ColorType.Wall) Order[1] = color;
            return Order;
        }
        public ColorType[] OrderLeft(ColorType[] Order){
        Order[0] = Order[1];
        Order[1] = Order[2];
        Order[2] = Order[3];
        Order[3] = Order[0];
        Order[0] = ColorType.Nothing;

        return Order;
        }
        public boolean OrderFull() {
            return Order[1]!=ColorType.Nothing&&Order[2]!=ColorType.Nothing&&Order[3]!=ColorType.Nothing;
        }
        public boolean IsEmpty(){
        return Order[1]==ColorType.Nothing&&Order[2]==ColorType.Nothing&&Order[3]==ColorType.Nothing;
        }
        public Supplier<Boolean> isEmpty(){
            return ()->Order[1]==ColorType.Nothing&&Order[2]==ColorType.Nothing&&Order[3]==ColorType.Nothing;
        }
        public ColorType[] OrderRight(ColorType[] Order){
            Order[0] = Order[3];
            Order[3] = Order[2];
            Order[2] = Order[1];
            Order[1] = Order[0];
            Order[0] = ColorType.Nothing;

        return Order;
    }
        public void setTarget(double target)
        {
            rotaryTargetPos = target;
        }

        public void setRotaryPower(double value)
        {
            //double voltage = voltageSensor.getVoltage();

            if (value < -1.0) value = -1.0;
            else if (value > 1.0) value = 1.0;

            servoDexRight.setPower(value);
            servoDexLeft.setPower(value);
        }
        public void setPosition(double position){
            if (Math.abs(getPosition())<=position-2.5)
                setRotaryPower(0.7);
            setRotaryPower(0);
        }
        public double getPosition()
        {
            return rotaryEncoder.getCurrentPosition().get(0)/8192*360;
        }

        public final Command update()
        {
            return Command.builder()
                    .update(() ->
                    {
                        if(Math.abs(getPosition()-rotaryTargetPos)<=2.5)
                            setPosition(rotaryTargetPos);

                        telemetry.addData("CurrentPosition", getPosition());
                        telemetry.addData("TargetPosition", rotaryTargetPos);
                        telemetry.addData("ErrorDistance", Math.abs(rotaryTargetPos-getPosition()));
                    })
                    .finished(()->Math.abs(getPosition()-rotaryTargetPos)<=2.5)
                    .end((interrupted)->{
                        setRotaryPower(0);
                    })
                    .build();
        }

//        public final Command move(double position)
//        {
//            return Command.builder()
//                    .init(() ->
//                    {
//                        setTarget(rotaryTargetPos + position);
//                    })
//                    .finished(() -> Math.abs(getPosition() - rotaryTargetPos) <= Tolerance)
//                    .end((interrupted) -> setRotaryPower(0))
//                    .requires(this)
//                    .build();
//        }
//
//        public void moveVoid(double position)
//        {
//            setTarget(rotaryTargetPos + position);
//        }

        public Command reset()
        {
            return new SequentialCommand(
                    new InstantCommand(()-> {setRotaryPower(0);}),
                    new InstantCommand(rotaryEncoder::reset));
        }
 }



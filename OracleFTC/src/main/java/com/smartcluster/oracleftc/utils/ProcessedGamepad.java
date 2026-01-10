package com.smartcluster.oracleftc.utils;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.smartcluster.oracleftc.math.Vector2d;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Wrapper for {@link Gamepad} that allows for better processed input. It works by processing
 * changes between two {@linkplain ProcessedGamepad#process()} and provides extra functionality(eg.
 * detecting when a button is moved, when the trigger is moved past a certain point)
 */
@SuppressWarnings("unused")
public class ProcessedGamepad {
    public final Button a, b, x, y, cross, square, circle, triangle, dpad_up, dpad_down, dpad_left,
        dpad_right, left_bumper, right_bumper, touchpad,
        share, options, ps;
    public final Trigger left_trigger, right_trigger;
    public final Joystick left_stick, right_stick;

    /**
     * Creates a {@linkplain ProcessedGamepad} that wraps around a gamepad
     *
     * @param gamepad wrapped gamepad
     */
    public ProcessedGamepad(Gamepad gamepad) {
        a = new Button(() -> gamepad.a);
        b = new Button(() -> gamepad.b);
        x = new Button(() -> gamepad.x);
        y = new Button(() -> gamepad.y);
        dpad_up = new Button(() -> gamepad.dpad_up);
        dpad_down = new Button(() -> gamepad.dpad_down);
        dpad_left = new Button(() -> gamepad.dpad_left);
        dpad_right = new Button(() -> gamepad.dpad_right);
        left_bumper = new Button(() -> gamepad.left_bumper);
        right_bumper = new Button(() -> gamepad.right_bumper);
        cross = new Button(() -> gamepad.cross);
        square = new Button(() -> gamepad.square);
        circle = new Button(() -> gamepad.circle);
        triangle = new Button(() -> gamepad.triangle);
        left_trigger = new Trigger(() -> (double) gamepad.left_trigger);
        right_trigger = new Trigger(() -> (double) gamepad.right_trigger);
        left_stick =
            new Joystick(() -> new Joystick.JoystickData(gamepad.left_stick_x, gamepad.left_stick_y,
                gamepad.left_stick_button));
        right_stick = new Joystick(
            () -> new Joystick.JoystickData(gamepad.right_stick_x, gamepad.right_stick_y,
                gamepad.right_stick_button));
        touchpad=new Button(()->gamepad.touchpad);
        options=new Button(()->gamepad.options);
        share=new Button(()->gamepad.share);
        ps=new Button(()->gamepad.ps);
    }

    /**
     * Updates the state of the wrapped gamepad, should be called every loop
     */
    public void process() {
        a.process();
        b.process();
        x.process();
        y.process();
        dpad_up.process();
        dpad_down.process();
        dpad_left.process();
        dpad_right.process();
        left_bumper.process();
        right_bumper.process();
        cross.process();
        square.process();
        circle.process();
        triangle.process();
        left_trigger.process();
        right_trigger.process();
        left_stick.process();
        right_stick.process();
        touchpad.process();
        options.process();
        share.process();
        ps.process();
    }

    /**
     * Represents a button on the gamepad, and has predefined functions to detect certain events
     */
    @SuppressWarnings("unused")
    public static class Button {
        private final Supplier<Boolean> internalSupplier;
        public boolean state;
        public boolean lastState;

        public Button(Supplier<Boolean> internalSupplier) {
            this.internalSupplier = internalSupplier;
        }

        /**
         * Called by {@linkplain ProcessedGamepad} to process button state
         */
        protected void process() {
            lastState = state;
            state = internalSupplier.get();
        }

        /**
         * Returns if the button was pressed
         * @return if the button was pressed
         */
        public Supplier<Boolean> pressed() {
            return () -> state && !lastState;
        }

        /**
         * Returns if the button was released
         * @return if the button was released
         */
        public Supplier<Boolean> released() {
            return () -> !state && lastState;
        }

        /**
         * Returns if the button changed
         * @return if the button changed
         */
        public Supplier<Boolean> changed() {
            return () -> state != lastState;
        }
        public Supplier<Boolean> down() {
            return () -> state;
        }
        public Supplier<Boolean> up() {
            return () -> !state;
        }

        public boolean get() {
            return state;
        }
    }

    /**
     * Represents a trigger on the gamepad, and has predefined functions to detect certain events
     */
    public static class Trigger {
        private final Supplier<Double> internalSupplier;
        public double state = 0.0;
        public double lastState = 0.0;

        public Trigger(Supplier<Double> internalSupplier) {
            this.internalSupplier = internalSupplier;
        }

        /**
         * Called by {@linkplain ProcessedGamepad} to process trigger state
         */
        public void process() {
            lastState = state;
            state = internalSupplier.get();
        }

        public Supplier<Boolean> when(Function<Double, Boolean> eval) {
            return () -> eval.apply(internalSupplier.get());
        }

        public double get() {
            return state;
        }
    }

    /**
     * Represents a joystick on the gamepad, and has predefined functions to detect certain events
     */
    public static class Joystick {

        private final Supplier<JoystickData> internalSupplier;
        private JoystickData state = new JoystickData(0, 0, false);
        private JoystickData lastState;

        public Joystick(Supplier<JoystickData> internalSupplier) {
            this.internalSupplier = internalSupplier;
        }
        /**
         * Called by {@linkplain ProcessedGamepad} to process joystick state
         */
        public void process() {
            lastState = state;
            state = internalSupplier.get();
        }

        public JoystickData get() {
            return state;
        }


        /**
         * Returns if the joystick is in the circular deadzone
         *
         * @param epsilon radius of deadzone
         * @return if the joystick is in the deadzone
         */
        public Supplier<Boolean> deadzone(double epsilon) {
            return () -> state.x * state.x + state.y * state.y <= epsilon * epsilon;
        }

        /**
         * Returns if the joystick is in the elliptic deadzone
         *
         * @param epsilonX radius of deadzone on x
         * @param epsilonY radius of deadzone on y
         * @return if the joystick is in the deadzone
         */
        public Supplier<Boolean> deadzone(double epsilonX, double epsilonY) {
            return () -> ((state.x / epsilonX) * (state.x / epsilonX)) +
                ((state.y / epsilonX) * (state.y / epsilonX)) <= 2;
        }

        public Button button() {
            return new Button(() -> state.pressed);
        }

        public static class JoystickData {
            public final double x, y;
            public final boolean pressed;

            public JoystickData(double x, double y, boolean pressed) {
                this.x = x;
                this.y = y;
                this.pressed = pressed;
            }

            public Vector2d vector() {
                return new Vector2d(x, y);
            }
        }

    }
}



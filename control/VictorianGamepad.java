package org.firstinspires.ftc.teamcode.vv7797.opmode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;

public final class VictorianGamepad {

    private final class BooleanControl {
        private final String NAME;
        private boolean pressed, tapped, released;

        private BooleanControl(String name) { NAME = name; }

        private void update(boolean state) {
            tapped = (state && !pressed);
            released = (!state && pressed);
            pressed = state;
        }
    }

    private final class FloatControl {
        private final String NAME;
        private float deadzone, state;

        private FloatControl(String name) { NAME = name; }

        private void update(float state) { this.state = state; }

        private float getState() { return (Math.abs(state) > deadzone ? state : 0); }
    }

    private Gamepad client;
    private ArrayList<BooleanControl> booleanControls;
    private ArrayList<FloatControl> floatControls;

    public boolean a, a_pressed, a_released;
    public boolean b, b_pressed, b_released;
    public boolean x, x_pressed, x_released;
    public boolean y, y_pressed, y_released;

    public boolean dpad_up, dpad_up_pressed, dpad_up_released;
    public boolean dpad_down, dpad_down_pressed, dpad_down_released;
    public boolean dpad_left, dpad_left_pressed, dpad_left_released;
    public boolean dpad_right, dpad_right_pressed, dpad_right_released;
    public boolean left_bumper, left_bumper_pressed, left_bumper_released;
    public boolean right_bumper, right_bumper_pressed, right_bumper_released;
    public boolean left_stick, left_stick_pressed, left_stick_released;
    public boolean right_stick, right_stick_pressed, right_stick_released;


    public float left_stick_x, left_stick_y;
    public float right_stick_x, right_stick_y;
    public float left_trigger, right_trigger;

    public VictorianGamepad(Gamepad g) {
        client = g;

        booleanControls = new ArrayList<BooleanControl>() {{
            add( new BooleanControl("a") );
            add( new BooleanControl("b") );
            add( new BooleanControl("x") );
            add( new BooleanControl("y") );

            add( new BooleanControl("dpad_up") );
            add( new BooleanControl("dpad_down") );
            add( new BooleanControl("dpad_left") );
            add( new BooleanControl("dpad_right") );

            add( new BooleanControl("left_bumper") );
            add( new BooleanControl("right_bumper") );
        }};

        floatControls = new ArrayList<FloatControl>() {{
            add( new FloatControl("left_stick_x") );
            add( new FloatControl("left_stick_y") );

            add( new FloatControl("right_stick_x") );
            add( new FloatControl("right_stick_y") );

            add( new FloatControl("left_trigger") );
            add( new FloatControl("right_trigger") );
        }};
    }

    public void update() {
        for (BooleanControl bool : booleanControls) {
            bool.update(getClientBoolean(bool.NAME));
            mapBoolean(bool);
        }

        for (FloatControl flo : floatControls) {
            flo.update(getClientFloat(flo.NAME));
            mapFloat(flo);
        }
    }

    private void mapBoolean(BooleanControl bool) {
        boolean[] state = new boolean[] { bool.pressed, bool.tapped, bool.released };

        switch (bool.NAME) {
            case "a":
                a = state[0];
                a_pressed = state[1];
                a_released = state[2];
            break;

            case "b":
                b = state[0];
                b_pressed = state[1];
                b_released = state[2];
            break;

            case "x":
                x = state[0];
                x_pressed = state[1];
                x_released = state[2];
            break;

            case "y":
                y = state[0];
                y_pressed = state[1];
                y_released = state[2];
            break;

            case "dpad_up":
                dpad_up = state[0];
                dpad_up_pressed = state[1];
                dpad_up_released = state[2];
            break;

            case "dpad_down":
                dpad_down = state[0];
                dpad_down_pressed = state[1];
                dpad_down_released = state[2];
            break;

            case "dpad_left":
                dpad_left = state[0];
                dpad_left_pressed = state[1];
                dpad_left_released = state[2];
            break;

            case "dpad_right":
                dpad_right = state[0];
                dpad_right_pressed = state[1];
                dpad_right_released = state[2];
            break;

            case "left_bumper":
                left_bumper = state[0];
                left_bumper_pressed = state[1];
                left_bumper_released = state[2];
            break;

            case "right_bumper":
                right_bumper = state[0];
                right_bumper_pressed = state[1];
                right_bumper_released = state[2];
            break;

            case "left_stick":
                left_stick = state[0];
                left_stick_pressed = state[1];
                left_stick_released = state[2];
            break;

            case "right_stick":
                right_stick = state[0];
                right_stick_pressed = state[1];
                right_stick_released = state[2];
            break;

        }
    }

    private void mapFloat(FloatControl flo) {
        float state = flo.getState();

        switch (flo.NAME) {
            case "left_stick_x":
                left_stick_x = state;
            break;

            case "left_stick_y":
                left_stick_y = state;
            break;

            case "right_stick_x":
                right_stick_x = state;
            break;

            case "right_stick_y":
                right_stick_y = state;
            break;

            case "left_trigger":
                left_trigger = state;
            break;

            case "right_trigger":
                right_trigger = state;
            break;
        }
    }

    private boolean getClientBoolean(String name) {
        switch (name) {
            case "a": return client.a;
            case "b": return client.b;
            case "x": return client.x;
            case "y": return client.y;

            case "dpad_up": return client.dpad_up;
            case "dpad_down": return client.dpad_down;
            case "dpad_left": return client.dpad_left;
            case "dpad_right": return client.dpad_right;

            case "left_bumper": return client.left_bumper;
            case "right_bumper": return client.right_bumper;

            case "left_stick": return client.left_stick_button;
            case "right_stick": return client.right_stick_button;

            default: return false;
        }
    }

    private float getClientFloat(String name) {
        switch (name) {
            case "left_stick_x": return client.left_stick_x;
            case "left_stick_y": return client.left_stick_y;

            case "right_stick_x": return client.right_stick_x;
            case "right_stick_y": return client.right_stick_y;

            case "left_trigger": return client.left_trigger;
            case "right_trigger": return client.right_trigger;

            default: return 0.0f;
        }
    }

    private void setFloatDeadzone(String name, float dz) {
        for (FloatControl flo : floatControls)
            if (flo.NAME.equals(name)) {
                flo.deadzone = dz;
                break;
            }
    }
}

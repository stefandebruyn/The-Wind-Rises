package org.firstinspires.ftc.teamcode.vv7797.opmode.control;

public interface PIDFramework {

    void reset();

    double getP(double error);

    double getI();

    double getD(double error);

    double getCorrection(double error);
}

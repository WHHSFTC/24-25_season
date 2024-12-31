package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SlidesPID {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double KpEx = 0.006;
    public static double KiEx = 0.0;
    public static double KdEx = 0.005;
    private double integralEx = 0.0;
    private double prevExError = 0.0;
    private double prevTargetEx = 0.0;
    private double stateEx = 0.0;
    private double runTimeEx = 0.0;


    public static double KpVe = 0.005;
    public static double KiVe = 0.0000002;
    public static double KdVe = 0.03;
    private double integralVe = 0.0;
    private double prevVeError = 0.0;
    private double prevTargetVe = 0.0;
    private double stateVe = 0.0;
    private double runTimeVe = 0.0;

    void updateEx(double state, double runTime) {
        this.stateEx = state;
        this.runTimeEx = runTime;
    }

    void updateVe(double state, double runTime) {
        this.stateVe = state;
        this.runTimeVe = runTime;
    }

    double calculatePowerExtendo(double target) {
        double error = target-stateEx;
        double power = 0.0;
        integralEx += error*runTimeEx;
        power = (error*KpEx + integralEx*KiEx + ((error - prevExError)/runTimeEx)*KdEx);
        prevExError = error;
        prevTargetEx = target;
        return power;
    }

    double calculatePowerVertical(double target) {
        double error = target-stateVe;
        double power = 0.0;
        integralVe += error*runTimeVe;
        power = (error*KpVe + integralVe*KiVe + ((error - prevVeError)/runTimeVe)*KdVe);
        prevVeError = error;
        prevTargetVe = target;
        return power;
    }
}
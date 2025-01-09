package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SlidesPID {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double KpEx = 0.006;
    public static double KiEx = 0.0;
    public static double KdEx = 0.25;
    private double integralEx = 0.0;
    private double prevExError = 0.0;
    private double prevTargetEx = 0.0;
    private double stateEx = 0.0;
    private double runTimeEx = 0.0;


    public static double KpVe = 0.002;
    public static double KiVe = 0;//0.00000001;
    public static double KdVe = 0.030;//0.005;
    public static double g = 0;
    private double integralVe = 0.0;
    private double prevVeError = 0.0;
    private double prevTargetVe = 0.0;
    private double stateVe = 0.0;
    private double runTimeVe = 0.0;


    public static double KpAutoVe = 0.007;
    public static double KdAutoVe = 0.0;
    public static double KiAutoVe = 0.0;
    private double integralAutoVe = 0.0;
    private double prevAutoVeError = 0.0;
    private double prevTargetAutoVe = 0.0;
    private double stateAutoVe = 0.0;
    private double runTimeAutoVe;


    public static double KpAutoEx = 0.013;
    public static double KdAutoEx = 0.0;
    public static double KiAutoEx = 0.0;
    private double integralAutoEx = 0.0;
    private double prevAutoExError = 0.0;
    private double prevTargetAutoEx = 0.0;
    private double stateAutoEx = 0.0;
    private double runTimeAutoEx;


    void updateEx(double state, double runTime) {
        this.stateEx = state;
        this.runTimeEx = runTime;
    }

    void updateVe(double state, double runTime) {
        this.stateVe = state;
        this.runTimeVe = runTime;
    }

    void updateVeAuto(double state, double runTime){
        this.stateAutoVe = state;
        this.runTimeAutoVe = runTime;
    }

    void updateExAuto(double state, double runTime){
        this.stateAutoEx = state;
        this.runTimeAutoEx = runTime;
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
        power = (error*KpVe + integralVe*KiVe + ((error - prevVeError)/runTimeVe)*KdVe) + g;
        prevVeError = error;
        prevTargetVe = target;
        return power;
    }

    double autoCalculatePowerVertical(double target){
        double error = target-stateAutoVe;
        double power = 0.0;
        integralAutoVe += error*runTimeAutoVe;
        power = (error*KpAutoVe + integralAutoVe*KiAutoVe + ((error - prevAutoVeError)/runTimeAutoVe)*KdAutoVe);
        prevAutoVeError = error;
        prevTargetAutoVe = target;
        return power;
    }

    double autoCalculatePowerExtendo(double target){
        double error = target-stateAutoEx;
        double power = 0.0;
        integralAutoEx += error*runTimeAutoEx;
        power = (error*KpAutoEx + integralAutoEx*KiAutoEx + ((error - prevAutoExError)/runTimeAutoEx)*KdAutoEx);
        prevAutoExError = error;
        prevTargetAutoEx = target;
        return power;
    }
}
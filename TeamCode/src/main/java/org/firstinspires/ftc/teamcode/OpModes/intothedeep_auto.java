package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous

public class intothedeep_auto extends intothedeep_opmode{

    @Override
    public void init(){
        super.init();

    }

    @Override
    public void start(){
        super.start();
        blueAlliance = intakeColor.blue() > intakeColor.red();
    }

    @Override
    public void childLoop(){

    }

    public class VertSlides implements Action {
        public VertSlides(double slideVePos){
            slidePositionTargetVe = slideVePos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action vertslides(double slideVePos) {
        return new VertSlides(slideVePos);
    }

    public class ExtendoSlides implements Action {
        public ExtendoSlides(double slideExPos){
            slidePositionTargetEx = slideExPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }


    public class OutputArm implements Action {
        double dlPos;
        double drPos;
        public OutputArm(double deltaLeftPos, double deltaRightPos){
            dlPos = deltaLeftPos;
            drPos = deltaRightPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            deltaLeft.setPosition(dlPos);
            deltaRight.setPosition(drPos);
            return false;
        }
    }

    public Action outputarm(double deltaLeftPos, double deltaRightPos) {
        return new OutputArm(deltaLeftPos, deltaRightPos);
    }


    public class OutputWrist implements Action {
        double oWristPos;
        public OutputWrist(double outputwristpos){
            oWristPos = outputwristpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outputWrist.setPosition(oWristPos);
            return false;
        }
    }

    public Action outputwrist(double outputwristpos) {
        return new OutputWrist(outputwristpos);
    }


    public class OutputClaw implements Action {
        double oClawPos;
        public OutputClaw(double outputclawpos){
            oClawPos = outputclawpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outputClaw.setPosition(oClawPos);
            return false;
        }
    }

    public Action outputclaw(double outputclawpos) {
        return new OutputClaw(outputclawpos);
    }


    public class SpringToggle implements Action {
        double sTogglePos;
        public SpringToggle(double springtogglepos){
            sTogglePos = springtogglepos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            springToggle.setPosition(sTogglePos);
            return false;
        }
    }

    public Action springtoggle(double springtogglepos) {
        return new SpringToggle(springtogglepos);
    }

    public class IntakeClaw implements Action {
        double iClawPos;
        public IntakeClaw(double intakeclawpos){
            iClawPos = intakeclawpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeClaw.setPosition(iClawPos);
            return false;
        }
    }

    public Action intakeclaw(double intakeclawpos) {
        return new IntakeClaw(intakeclawpos);
    }


    public class IntakeWrist implements Action {
        double iWristPos;
        public IntakeWrist(double intakewristpos){
            iWristPos = intakewristpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeWrist.setPosition(iWristPos);
            return false;
        }
    }

    public Action intakewrist(double intakewristpos) {
        return new IntakeWrist(intakewristpos);
    }


    public class IntakeArm implements Action {
        double aPos;
        double bPos;
        public IntakeArm(double alphapos, double betapos){
            aPos = alphapos;
            bPos = betapos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            alpha.setPosition(aPos);
            beta.setPosition(bPos);
            return false;
        }
    }

    public Action intakearm(double alphapos, double betapos) {
        return new IntakeArm(alphapos, betapos);
    }

}

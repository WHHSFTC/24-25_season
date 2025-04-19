package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class extendoAuto extends intothedeep_auto{

    @Override
    public void init(){
        super.init();

        intakeClaw.setPosition(intakeClawOpenPos);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
    }

    @Override
    public void start(){

    }

    @Override
    public void childLoop(){
        super.childLoop();
        slidePositionTargetEx = slideMaxEx;
    }

    @Override
    public void stop(){

    }
}

package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Config
@Autonomous (preselectTeleOp = "intothedeep_tele_blue")
public class EveningSpecial extends intothedeep_auto{
    private final Pose startPos = new Pose(7.21, 103.17, Math.toRadians(270));
    private final Pose scorePos = new Pose(9.21, 10, Math.toRadians(0)); //13.5, 126.3


    private Path scorePreload, park;
    private PathChain grabSample1, grabSample2, grabSample3, sub1, sub2, scoreSample1, scoreSample2, scoreSample3, scoreSub1, scoreSub2;


    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPos), new Point(scorePos)));
        scorePreload.setLinearHeadingInterpolation(startPos.getHeading(), scorePos.getHeading());
    }

    @Override
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.85);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() >= 1.9 && ms.getCurrentPosition() > 1750) {

                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
//                    follower.setMaxPower(0.5);
//                    follower.followPath(grabSample1);
                    setPathState(3);
                }
                break;

        }

    }

    @Override
    public void init(){
        super.init();
        follower.setStartingPose(startPos);
        buildPaths();
        limelight.start();
    }

    @Override
    public void start(){
        super.start();

    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PPAprilTags;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.states.FeederConeGripperStateMachine;

@Autonomous(name = "Park", group = "RoarBotics")
public class ParkOnly extends LinearOpMode {
    PPBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj0 = new Vector2d(30,40); //forward to tile
    static final Vector2d Traj1 = new Vector2d(35, 25.5); //strafe left to pole
    static final Vector2d Traj2 = new Vector2d(30,25.5); //strafe right to middle
    static final Vector2d Traj3 = new Vector2d(30, 38); //BACK
    static final Vector2d Location1 = new Vector2d(57, 40);
    //static final Vector2d Location2 = new Vector2d(30.5, 38);
    static final Vector2d Location3 = new Vector2d(3,40);

    enum State {
        WAIT0,
        CLAWCLOSE,
        STRAFELEFT,
        LIFTUP,
        FORWARD,
        BACK1,
        MOVEARM,
        CLAWOPEN,
        STRAFERIGHT,
        LIFTDOWN,
        TOSTACK,
        IDLE,
        PARK,
        GRAB;
    }

    State currentState = State.IDLE;
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Pose2d startPose = new Pose2d(30, 65, Math.toRadians(-90));

    PPAprilTags ppapriltags;
    String placement = "NONE";

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new PPBase(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Traj0)
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Traj1)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineTo(Traj2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineTo(Traj3)
                .build();

        TrajectorySequence location1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Location1)
                .build();

//        TrajectorySequence location2 = drive.trajectorySequenceBuilder(traj3.end())
//                .lineTo(Location2)
//                .build();

        TrajectorySequence location3 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Location3)
                .build();

        drive.getExpansionHubs().update(getDt());
        drive.robot.getFeeder().update(getDt());

        double t1 = waitTimer.milliseconds();

        ppapriltags = new PPAprilTags();
        ppapriltags.init(hardwareMap);

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int AprilTag = 0;

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        currentState = State.WAIT0;
        waitTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    telemetry.addLine("in the wait0 state");
                    AprilTag = ppapriltags.getDetectedTag();
                    if (AprilTag != 0){
                        switch (AprilTag)
                        {
                            case 2:
                                placement = "ONE";
                                break;
                            case 3:
                                placement = "TWO";
                                break;
                            case 4:
                                placement = "THREE";
                                break;
                        }
                    }
                    else {
                        telemetry.addLine("April Tag not found.");
                    }

                    if (waitTimer.milliseconds() > 3000 && AprilTag != 0){
                        telemetry.addLine("April Tag found: " + placement);
                        ppapriltags.close();
                        currentState = State.FORWARD;
                        waitTimer.reset();
                    }
                    break;
                case CLAWCLOSE:
                    telemetry.addData("placement: ", placement);
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getFeeder().getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.CLOSE);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case STRAFELEFT:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj1);
                        currentState = State.CLAWOPEN;
                        waitTimer.reset();
                    }
                    break;

                case LIFTUP:
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 1000){
                        //you need to add a line of code to set the extn height on the stack tracker
                        drive.robot.getStackTracker().setPoleTargetType(1);
                        drive.robot.getFeeder().extendPoles();

                        currentState = State.FORWARD;
                        waitTimer.reset();
                    }
                    break;

                case FORWARD:
                    if (waitTimer.milliseconds() >= 2000) {
                        drive.followTrajectorySequenceAsync(traj0);
                        currentState = State.PARK;
                        waitTimer.reset();
                    }
                    break;

                case BACK1:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj3);
                        currentState = State.PARK;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARM:
                    if(!drive.isBusy()){
                        //drive.robot.getArmSubsystem().getStateMachine().updateState(ArmStateMachine.State.RIGHT);
                        currentState = State.CLAWOPEN;
                        waitTimer.reset();
                    }
                    break;

                case CLAWOPEN:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getFeeder().getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.OPEN);
                        currentState = State.STRAFERIGHT;
                        waitTimer.reset();
                    }
                    break;

                case STRAFERIGHT:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.LIFTDOWN;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:
                    if(waitTimer.milliseconds() >= 2000){
                        drive.robot.getStackTracker().setPoleTargetType(0);
                        drive.robot.getFeeder().retract();
                        currentState = State.BACK1;
                        waitTimer.reset();
                    }
                    break;

                case TOSTACK:
                    if(waitTimer.milliseconds() >= 2000){
                        drive.followTrajectorySequenceAsync(traj3);
                        currentState = State.GRAB;
                        waitTimer.reset();
                    }
                    break;

                case GRAB:
                    if(!drive.isBusy()){
                        //drive.robot.getClawSubsystem().getStateMachine().updateState(ClawStateMachine.State.CLOSE);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case PARK:
                    telemetry.addData("placement: ", placement);
                    if(waitTimer.milliseconds() >= 2000){
                        if(placement == "ONE"){
                            drive.followTrajectorySequenceAsync(location1);
                        }
                        else if(placement == "TWO"){
                            //drive.followTrajectorySequenceAsync(location2);
                        }
                        else if(placement == "THREE"){
                            drive.followTrajectorySequenceAsync(location3);
                        }
                        currentState = ParkOnly.State.IDLE;
                        waitTimer.reset();
                    }
                    break;

                case IDLE:
                    //PoseStorage.currentPose = drive.getPoseEstimate();
                    //PoseStorage.forBlue = true;
                    break;
            }


            drive.update();
            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            drive.getExpansionHubs().update(getDt());
            drive.robot.getFeeder().update(getDt());

            telemetry.update();
        }

        drive.setMotorPowers(0.0,0.0,0.0,0.0);
    }
    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }
}

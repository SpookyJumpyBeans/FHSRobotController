package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.PPAprilTags;
import org.firstinspires.ftc.teamcode.odometry.trajectorysequence.TrajectorySequence;

//import org.firstinspires.ftc.teamcode.team10515.states.ArmStateMachine;
//import org.firstinspires.ftc.teamcode.team10515.states.ClawStateMachine;
//import org.firstinspires.ftc.teamcode.team10515.states.LiftStateMachine;

@Autonomous(name = "Blue Left", group = "RoarBotics")
public class BlueLeft extends LinearOpMode {
    PPBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj0 = new Vector2d(34,65.5);
    static final Vector2d Traj1 = new Vector2d(34, 12);
    static final Vector2d Traj2 = new Vector2d(20.5,12.5);
    static final Vector2d Traj3 = new Vector2d(54, 13);
    static final Vector2d Location1 = new Vector2d(50, 12);
    static final Vector2d Location2 = new Vector2d(34, 12);
    static final Vector2d Location3 = new Vector2d(12,12);

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        CLAWCLOSE,
        INITSTRAFE,
        LIFTUP,
        FORWARD,
        PRELOAD,
        MOVEARM,
        CLAWOPEN,
        MOVEARMBACK,
        LIFTDOWN,
        TODROP,
        TOSTACK,
        IDLE,
        PARK,
        GRAB
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(31, 65.5, Math.toRadians(-90));


    //these are based on LiftTest
    private static final double HIGH = 24d;
    private static final double MID = 23.5d;
    private static final double LOW = 14d;


    PPAprilTags ppapriltags;
    String placement = "NONE";

    boolean tf = false;

    int counter = 0;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new PPBase(hardwareMap);
        drive.setPoseEstimate(startPose);
        //drive.robot.getLiftSubsystem().getStateMachine().updateState(LiftStateMachine.State.IDLE);
        //drive.robot.getClawSubsystem().getStateMachine().updateState(ClawStateMachine.State.OPEN);
        //drive.robot.getArmSubsystem().getStateMachine().updateState(ArmStateMachine.State.INIT);
        //drive.robot.getCappingArmSubsystem().getStateMachine().updateState(ArmStateMachine.State.REST);

        TrajectorySequence traj0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Traj0)
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Traj1)
                .turn(Math.toRadians(93.5))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineTo(Traj2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineTo(Traj3)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineTo(Traj2)
                .build();

        TrajectorySequence location1 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location1)
                .build();

        TrajectorySequence location2 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location2)
                .build();

        TrajectorySequence location3 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location3)
                .build();

        drive.getExpansionHubs().update(getDt());

        //drive.robot.getLiftSubsystem().update(getDt());
        //drive.robot.getArmSubsystem().update(getDt());
        //drive.robot.getClawSubsystem().update(getDt());

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
                        ppapriltags.close();
                        currentState = State.FORWARD;
                    }
                    break;
                case CLAWCLOSE:
                    telemetry.addData("placement: ", placement);
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getClawSubsystem().getStateMachine().updateState(ClawStateMachine.State.CLOSE);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case INITSTRAFE:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.followTrajectorySequenceAsync(traj0);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case LIFTUP:
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 750){
                        drive.robot.getLiftSubsystem().extend(MID);
//                        if(tf){
//                            currentState = State.TODROP;
//                        }
//                        else {
//
//                            tf = true;
//                        }
                        currentState = State.TODROP;
                        waitTimer.reset();
                    }
                    break;

                case FORWARD:
                    if (waitTimer.milliseconds() >= 2000) {
                        drive.followTrajectorySequenceAsync(traj1);
                        currentState = State.PARK;
                        waitTimer.reset();
                    }
                    break;

                case PRELOAD:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.MOVEARM;
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
                        drive.robot.getClawSubsystem().getStateMachine().updateState(ClawStateMachine.State.OPEN);
                        currentState = State.FORWARD;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARMBACK:
                    if(waitTimer.milliseconds() >= 1000){
                        //drive.robot.getArmSubsystem().getStateMachine().updateState(ArmStateMachine.State.INIT);
                        currentState = State.LIFTDOWN;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:
                    if(waitTimer.milliseconds() >= 1000){
                        currentState = State.PARK;
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

                case TODROP:
                    if(waitTimer.milliseconds() >= 2000){
                        drive.followTrajectorySequenceAsync(traj4);
//                        if(counter < 1){
//                            currentState = State.MOVEARM;
//                            waitTimer.reset();
//                            counter++;
//                        }
                        else {
                            currentState = State.CLAWOPEN;
                            waitTimer.reset();
                        }
                    }
                    break;

                case PARK:
                    telemetry.addData("placement: ", placement);
                    if(waitTimer.milliseconds() >= 2000){
                        if(placement == "ONE"){
                            drive.followTrajectorySequenceAsync(location1);
                        }
                        else if(placement == "TWO"){
                            drive.followTrajectorySequenceAsync(location2);
                        }
                        else if(placement == "THREE"){
                            drive.followTrajectorySequenceAsync(location3);
                        }
                        currentState = BlueLeft.State.IDLE;
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
            //drive.robot.getLiftSubsystem().update(getDt());
            //drive.robot.getArmSubsystem().update(getDt());
            //drive.robot.getClawSubsystem().update(getDt());
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

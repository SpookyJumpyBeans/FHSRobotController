package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.TelemetryWritable;
import org.firstinspires.ftc.teamcode.lib.util.Updatable;
import org.firstinspires.ftc.teamcode.states.IState;

public interface ISubsystem<M extends IState, S extends Enum<S> & Namable> extends TelemetryWritable, Updatable, Namable {
    M getStateMachine();
    S getState();
    void start();
    void stop();
}

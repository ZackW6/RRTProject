package Canvas.Commands;

import java.util.function.BooleanSupplier;

public class Commands {

    public static CommandBase run(Runnable run){
        return new Command(run);
    }

    public static CommandBase timed(Runnable run, double miliseconds){
        return new TimedCommand(run, miliseconds);
    }

    public static CommandBase runOnce(Runnable run){
        return new InstantCommand(run);
    }

    public static CommandBase either(Runnable optionOne, Runnable optionTwo, BooleanSupplier reason){
        return new ConditionalCommand(optionOne, optionTwo, reason);
    }

    public static CommandBase waitMiliseconds(double miliseconds){
        return new WaitCommand(miliseconds);
    }
}

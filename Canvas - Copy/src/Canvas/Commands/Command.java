package Canvas.Commands;

/**
 * Command class is a timer/thread mix, will atempt to match time allotted but if process takes too long will just run with 0 intended delay
 */
public class Command extends CommandBase{
    private Runnable run;

    public Command(Runnable run){
        this.run = run;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        run.run();
    }

    @Override
    public void finallyDo() {
        
    }

}

package Canvas.Commands;

/**
 * Command class is a thread
 */
public class InstantCommand extends CommandBase{

    private Runnable run;
    public InstantCommand(Runnable run){
        this.run = run;
    }

    @Override
    public void initialize() {
        run.run();
        cancel();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void finallyDo() {
        
    }
   
}

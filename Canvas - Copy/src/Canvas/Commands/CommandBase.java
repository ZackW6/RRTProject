package Canvas.Commands;

import java.util.ArrayList;

public abstract class CommandBase {

    private ArrayList<Runnable> initRuns = new ArrayList<>();
    private ArrayList<Runnable> afterRuns = new ArrayList<>();

    protected Thread thread = new Thread(()->{
        initialize();
        for (Runnable run : initRuns){
            run.run();
        }
        while(true){
            execute();
            if (isCanceled()){
                finallyDo();
                for (Runnable run : afterRuns){
                    run.run();
                }
                break;
            }
        }
    });

    protected boolean isCanceled = false;

    public abstract void initialize();

    public abstract void execute();

    public abstract void finallyDo();

    public CommandBase startWith(Runnable run){
        initRuns.add(run);
        return this;
    }

    public CommandBase finallyDo(Runnable run){
        afterRuns.add(run);
        return this;
    }

    public void schedule(){
        isCanceled = false;
        this.thread = new Thread(()->{
            initialize();
            for (Runnable run : initRuns){
                run.run();
            }
            while(true){
                execute();
                if (isCanceled()){
                    finallyDo();
                    for (Runnable run : afterRuns){
                        run.run();
                    }
                    break;
                }
            }
        });
        thread.start();
    }

    public void cancel(){
        isCanceled = true;
    }

    @Deprecated
    public void interrupt(){
        try {
            thread.interrupt();
        } catch (Exception e) {
            System.out.println("Failed");
            e.printStackTrace();
        }
        
        finallyDo();
    }

    public boolean isRunning(){
        return thread.isAlive();
    }

    public boolean isCanceled(){
        return isCanceled;
    }

    /**
     * returns initialize, execute, and finally runnables
     * @return
     */
    public Runnable[] getRunLine(){
        return new Runnable[]{()->initialize(), ()->execute(), ()->finallyDo()};
    }
}

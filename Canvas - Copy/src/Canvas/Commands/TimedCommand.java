package Canvas.Commands;

import Canvas.Util.Profile;

/**
 * Command class is a timer/thread mix, will atempt to match time allotted but if process takes too long will just run with 0 intended delay
 */
public class TimedCommand extends Command{

    private final double waitTime;
    public TimedCommand(Runnable run, double timeMili){
        super(()->{
            Profile profile = new Profile();
            profile.reset();
            profile.start();
            run.run();
            profile.stop();
            double correctWaitTime = timeMili - (profile.getTime());
            if (correctWaitTime>0){
                try {
                    Thread.sleep((long) correctWaitTime);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        });
        waitTime = timeMili;
    }

    public double getTimer() {
        return waitTime;
    }
    
}

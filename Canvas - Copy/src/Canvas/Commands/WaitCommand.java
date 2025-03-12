package Canvas.Commands;

public class WaitCommand extends InstantCommand{

    public WaitCommand(double waitMili){
        super(()->{
            try {
                Thread.sleep((long) waitMili);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        });
    }
}

package Canvas.Commands;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class Trigger {
    BooleanSupplier supplier;
    private ArrayList<CommandBase> commands = new ArrayList<CommandBase>();
    public Trigger (BooleanSupplier supplier){
        this.supplier = supplier;
    }

    private class BoolClass{
        private boolean bool;
        private BoolClass(boolean init){
            bool = init;
        }
        private void set(boolean bool){
            this.bool = bool;
        }
        private boolean getBoolean(){
            return bool;
        }
    }

    public Trigger onTrue(CommandBase runner){
        commands.add(runner);
        BoolClass wasOn = new BoolClass(false);
        Runnable checker = ()->{
            if (supplier.getAsBoolean()){
                if (!wasOn.getBoolean() && !runner.isRunning()){
                    try {
                        runner.schedule();
                    } catch (Exception e) {
                        System.out.println("CaughtOT");
                        e.printStackTrace();
                    }
                    
                    wasOn.set(true);
                }
            }else{
                wasOn.set(false);
            }
        };
        Command command = new TimedCommand(checker, 10);
        commands.add(command);
        command.schedule();
        return this;
    }

    public Trigger onFalse(CommandBase runner){
        commands.add(runner);
        BoolClass wasOn = new BoolClass(true);
        Runnable checker = ()->{
            if (!supplier.getAsBoolean()){
                if (!wasOn.getBoolean() && !runner.isRunning()){
                    runner.schedule();
                    wasOn.set(true);
                }
            }else{
                wasOn.set(false);
            }
        };
        CommandBase command = new TimedCommand(checker, 10);
        commands.add(command);
        command.schedule();
        return this;
    }
    public Trigger whileTrue(CommandBase runner, double time){
        commands.add(runner);
        Runnable checker = ()->{
            if (supplier.getAsBoolean() && !runner.isRunning()){
                try {
                    runner.schedule();
                } catch (Exception e) {
                    System.out.println("CaughtWT");
                    e.printStackTrace();
                }
            }
        };
        CommandBase command = new TimedCommand(checker, time);
        commands.add(command);
        command.schedule();
        return this;
    }
    public Trigger whileFalse(CommandBase runner, double time){
        commands.add(runner);
        Runnable checker = ()->{
            if (!supplier.getAsBoolean() && !runner.isRunning()){
                runner.schedule();
            }
        };
        CommandBase command = new TimedCommand(checker, time);
        commands.add(command);
        command.schedule();
        
        return this;
    }
    public void endAll(){
        for (int i =0; i<commands.size();i++){
            commands.get(i).cancel();
        }
        commands.clear();
    }
}

package Canvas.Commands;

import java.util.function.BooleanSupplier;

public class ConditionalCommand extends CommandBase{

    private final BooleanSupplier reason;
    private final Runnable optionOne;
    private final Runnable optionTwo;

    /**
     * first option is when true on init, second is false
     * @param optionOne
     * @param optionTwo
     * @param reason
     */
    public ConditionalCommand(Runnable optionOne, Runnable optionTwo, BooleanSupplier reason){
        this.optionOne = optionOne;
        this.optionTwo = optionTwo;
        this.reason = reason;
    }

    @Override
    public void initialize() {
        if (reason.getAsBoolean()){
            optionOne.run();
        }else{
            optionTwo.run();
        }
        cancel();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void finallyDo() {
        
    }
    
}

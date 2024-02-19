package frc.robot;

public class RobotState {
    public enum State {
        IDLE,
        INTAKE,
        AMP,
        SPEAKER,
        CLIMB
    }
    private State state, prevState;
    private static RobotState INSTANCE = null;
    private RobotState(){
        this.state = State.IDLE;
        this.prevState = State.IDLE;
    }

    public static RobotState getInstance(){
        if(INSTANCE == null){
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public State getState(){
        return this.state;
    }

    public void setState(State newState){
        this.prevState = this.state;
        this.state = newState;
    }

    public boolean stateChanged(){
        return this.prevState == this.state;
    }
}

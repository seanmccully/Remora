#ifndef STATES_H
#define STATES_H
enum class State {
    ST_SETUP,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET,
    ST_ERROR
};

class StateMachine {
private:
    Mutex stateMutex;
    State currentState;
    State prevState;

public:
    StateMachine() : currentState(State::ST_SETUP), prevState(State::ST_RESET) {}

    void transitionTo(State newState) {
        stateMutex.lock();
        if (currentState != newState) {
            prevState = currentState;
            currentState = newState;
        }
        stateMutex.unlock();
    }

    State getCurrentState() {
        stateMutex.lock();
        State state = currentState;
        stateMutex.unlock();
        return state;
    }

    State getPreviousState() {
        stateMutex.lock();
        State state = prevState;
        stateMutex.unlock();
        return state;
    }
};
#endif 

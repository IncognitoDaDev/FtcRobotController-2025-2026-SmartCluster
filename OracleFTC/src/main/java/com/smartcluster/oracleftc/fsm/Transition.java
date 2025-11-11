package com.smartcluster.oracleftc.fsm;

import com.smartcluster.oracleftc.commands.Command;

/**
 * Represents a FSM transition
 * @param <T> state enum
 */
public interface Transition<T> {
    /**
     * Transition condition
     * @return if the transition should be triggered
     */
    boolean getCondition();

    /**
     * The command which is executed during the transition
     * @return the command associated to the transition
     */
    Command getCommand();

    /**
     * The target state after the transition finishes
     * @return the target state
     */
    T getTargetState();
}

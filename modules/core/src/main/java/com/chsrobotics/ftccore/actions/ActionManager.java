package com.chsrobotics.ftccore.actions;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class ActionManager {

    private final Map<Integer, LinkedList<Action>> actions = new HashMap<>();
    private final List<ContinuousAction> continuousActions = new LinkedList<>();

    public ActionManager()
    {
    }

    public void addAction(Action action)
    {
        if (actions.containsKey(action.getIndex())) {
            LinkedList<Action> actionList = actions.get(action.getIndex());
            for (int i = 0; i < actionList.size(); i++) {
                if (actionList.get(i).getIndex() > action.getIndex()) {
                    actionList.add(i, action);
                }
            }
        } else {
            LinkedList<Action> actionList = new LinkedList<>();
            actionList.add(action);
            actions.put(action.getIndex(), actionList);
        }
    }

    public void addContinuousAction(ContinuousAction continuousAction) {
        continuousActions.add(continuousAction);
    }

    public void executeContinuousActions() {
        for (ContinuousAction a : continuousActions) {
            a.execute();
        }
    }

    // Should be run right before autonomous movement starts
    public void initialize() {
        for (ContinuousAction a : continuousActions) {
            a.initialize();
        }
    }

    // Executes the task at the waypoint with a given index
    public void executeActions(int index)
    {
        LinkedList<Action> actionList = actions.get(index);
        if (actionList == null) {
            return;
        }
        for (int i = 0; i < actionList.size(); i++) {
            Action action = actionList.get(i);
            if (action == null) {
                throw new RuntimeException("Invalid action priority: expected one priority for each value from 0 to " + (actionList.size() - 1) + ", but couldn't find an action with priority " + i);
            }
            action.execute();
//            if (!action.shouldContinueExecutingActionsAtCurrentIndex()) {
//                break;
//            }
        }
    }

    public void reset() {
        actions.clear();
    }

    public List<ContinuousAction> getContinuousActions() {
        return continuousActions;
    }

}

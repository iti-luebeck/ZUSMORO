package smachGenerator;

import java.util.ArrayList;


public interface ISmachableState {
	/**
	 * Returns an ArrayList containing containing 
	 * @return
	 */
	public abstract ArrayList<? extends ISmachableAction> getActions();

	public abstract ArrayList<? extends ISmachableTransition> getTransitions();

	public abstract boolean isInitialState();
	
	public abstract String getText();

}
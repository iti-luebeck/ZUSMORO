package smachGenerator;

import model.State;

public interface ISmachableTransition {

	public abstract State getFollowerState();

	public abstract ISmachableGuard getSmachableGuard();

	public abstract String getLabel();

}
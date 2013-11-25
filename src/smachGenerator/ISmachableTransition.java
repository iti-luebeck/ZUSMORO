package smachGenerator;

public interface ISmachableTransition {

	public abstract ISmachableState getFollowerState();

	public abstract ISmachableGuard getSmachableGuard();

	public abstract String getLabel();

}
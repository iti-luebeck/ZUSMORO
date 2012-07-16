package robots.epuck;

public interface EPuckSensorI {

	public abstract int[] getFloorIr();

	public abstract int[] getIrDistances();

	public abstract int[] getLedState();

	public abstract int[] getMotorState();

}
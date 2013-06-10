package smachGenerator;

import model.bool.Variable.Operator;

public interface ISmachableVariable {

	public abstract int getCompValue();

	public abstract Operator getOperator();

	public abstract String getVariableName();

}
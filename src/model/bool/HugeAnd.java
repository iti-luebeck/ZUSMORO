package model.bool;

import java.util.ArrayList;
import java.util.LinkedList;

import model.bool.Variable.Operator;

import smachGenerator.ISmachableGuard;
import smachGenerator.ISmachableHugeAnd;

public class HugeAnd extends BooleanExpression implements ISmachableHugeAnd, ISmachableGuard {

	private ArrayList<BooleanExpression> operands;

	public HugeAnd() {
		this.operands = new ArrayList<BooleanExpression>();
	}

	public void addOperand(BooleanExpression expr) {
		if (expr == null) {
			throw new IllegalArgumentException("Boolean operants may not be null");
		}
		this.operands.add(expr);
	}

	/* (non-Javadoc)
	 * @see model.bool.ISmachableHugeAnd#getOperands()
	 */
	@Override
	public BooleanExpression[] getOperands() {
		return operands.toArray(new BooleanExpression[operands.size()]);
	}

	@Override
	public boolean eval() {
		boolean result = true;
		for (BooleanExpression expr : this.operands) {
			result = result && expr.eval();
		}
		return result;
	}

	@Override
	public String toString() {
		StringBuilder buffer = new StringBuilder();
		buffer.append("<HugeAnd>\r\n");
		if (operands.size() > 0) {
			buffer.append(operands.get(0));
			for (int i = 1; i < operands.size(); i++) {
				buffer.append("\r\n" + operands.get(i).toString());
			}
		}
		buffer.append("\r\n</HugeAnd>");
		return buffer.toString();
	}

	@Override
	public boolean equals(Object obj) {
		return obj != null && obj instanceof HugeAnd && operands.equals(((HugeAnd)obj).operands);
	}

	public void removeOperand(Variable v) {
		if (v== null) {
			throw new IllegalArgumentException("Boolean operants may not be null");
		}
		this.operands.remove(v);
	}

	@Override
	public LinkedList<String> getSensorNames() {
		LinkedList<String> names= new LinkedList<>();
		for(BooleanExpression be:operands){
			if(be instanceof Variable){
				names.add(((Variable)be).getVariableName());
			}
		}
		return names;
	}

	@Override
	public LinkedList<Operator> getOperators() {
		LinkedList<Operator> operators= new LinkedList<>();
		for(BooleanExpression be:operands){
			if(be instanceof Variable){
				operators.add(((Variable)be).getOperator());
			}
		}
		return operators;
	}

	@Override
	public LinkedList<Integer> getCompValues() {
		LinkedList<Integer> values= new LinkedList<>();
		for(BooleanExpression be:operands){
			if(be instanceof Variable){
				values.add(((Variable)be).getCompValue());
			}
		}
		return values;
	}
}

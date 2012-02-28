package model.bool;

import java.util.ArrayList;

public class HugeAnd extends BooleanExpression {

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
}

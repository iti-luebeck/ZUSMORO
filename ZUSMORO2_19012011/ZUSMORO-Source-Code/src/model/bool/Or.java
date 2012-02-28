package model.bool;

public class Or extends BooleanExpression {

	private BooleanExpression arg1;
	private BooleanExpression arg2;

	public Or(BooleanExpression arg1, BooleanExpression arg2) {
		if (arg1 == null || arg2 == null) {
			throw new IllegalArgumentException("Boolean operants may not be null");
		}
		this.arg1 = arg1;
		this.arg2 = arg2;
	}

	@Override
	public boolean eval() {
		return (arg1.eval() || arg2.eval());
	}

	@Override
	public String toString() {
		return "<Or>\r\n" + arg1 + "\r\n" + arg2 + "\r\n</Or>";
	}

	@Override
	public boolean equals(Object obj) {
		boolean equal = false;
		if (obj != null && obj instanceof Or) {
			Or or = (Or) obj;
			equal = this.arg1.equals(or.arg1) && this.arg2.equals(or.arg2);
			equal |= this.arg1.equals(or.arg2) && this.arg2.equals(or.arg1);
		}
		return equal;
	}
}
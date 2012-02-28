package model.bool;

public class And extends BooleanExpression {

	private BooleanExpression arg1;
	private BooleanExpression arg2;

	public And(BooleanExpression arg1, BooleanExpression arg2) {
		if (arg1 == null || arg2 == null) {
			throw new IllegalArgumentException("Boolean operants may not be null");
		}
		this.arg1 = arg1;
		this.arg2 = arg2;
	}

	@Override
	public boolean eval() {
		return (arg1.eval() && arg2.eval());
	}

	@Override
	public String toString() {
		return "<And>\r\n" + arg1 + "\r\n" + arg2 + "\r\n</And>";
	}

	@Override
	public boolean equals(Object obj) {
		boolean equal = false;
		if (obj != null && obj instanceof And) {
			And and = (And) obj;
			equal = this.arg1.equals(and.arg1) && this.arg2.equals(and.arg2);
			equal |= this.arg1.equals(and.arg2) && this.arg2.equals(and.arg1);
		}
		return equal;
	}
}
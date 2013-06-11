package model.bool;

import model.Automat;

public class Variable extends BooleanExpression {

	private interface Eval {
		public boolean eval(int current, int compare);
	}

	public enum Operator implements Eval {
		BIGGER(">", "&gt;", 5,"BIGGER", new Eval() {
			@Override
			public boolean eval(int current, int compare) {
				return current > compare;
			}
		}),

		BIGGER_EQUAL(">=", "&gt;=", 4,"BIGGER_EQUAL", new Eval() {
			@Override
			public boolean eval(int current, int compare) {
				return current >= compare;
			}
		}), EQUAL("==", "==", 3,"EQUAL", new Eval() {
			@Override
			public boolean eval(int current, int compare) {
				return current == compare;
			}
		}), NOT_EQUAL("!=", "!=", 2,"NOT_EQUAL", new Eval() {
			@Override
			public boolean eval(int current, int compare) {
				return current != compare;
			}
		}), SMALLER_EQUAL("<=", "&lt;=", 1,"SMALLER_EQUAL", new Eval() {
			@Override
			public boolean eval(int current, int compare) {
				return current <= compare;
			}
		}), SMALLER("<", "&lt;", 0,"SMALLER", new Eval() {
			@Override
			public boolean eval(int current, int compare) {
				return current < compare;
			}
		});

		private Eval e;
		private String htmlString;
		private int operatorNumber;
		private String s;
		private String weaker;

		private Operator(String s, String htmlString, int opN, String weaker, Eval e) {
			this.s = s;
			this.htmlString = htmlString;
			this.operatorNumber = opN;
			this.e = e;
			this.weaker=weaker;
		}

		@Override
		public boolean eval(int current, int compare) {
			return e.eval(current, compare);
		}

		public int getOperatorNumber() {
			return operatorNumber;
		}

		public String toHTMLString() {
			return htmlString;
		}
		public Operator getWeaker(){
			return Operator.valueOf(weaker);
		}

		@Override
		public String toString() {
			return s;
		}
	}

	private int compValue;
	private Operator op;
	private String variable;

	public Variable(String variable, Operator op, int compValue) {
		this.variable = variable;
		this.op = op;
		this.compValue = compValue;
	}

	@Override
	public boolean equals(Object obj) {
		boolean equal = false;
		if (obj != null && obj instanceof Variable) {
			Variable var = (Variable) obj;
			equal = this.variable.equals(var.variable);
			equal &= this.compValue == var.compValue;
			equal &= this.op.equals(var.op);
		}
		return equal;
	}

	@Override
	public boolean eval() {
		int currentValue;
		if (Automat.runningAutomat != null) {
			currentValue = Automat.runningAutomat.getVariableValue(variable);
		} else {
			return false;
		}
		return op.eval(currentValue, compValue);
	}

	public int getCompValue() {
		return this.compValue;
	}

	
	public Operator getOperator() {
		return this.op;
	}
	
	public String getVariableName() {
		return this.variable;
	}

	@Override
	public String toString() {
		StringBuilder buffer = new StringBuilder();
		buffer.append("<Variable>\r\n");
		buffer.append(variable + "\r\n" + op.name() + "\r\n" + compValue);
		buffer.append("\r\n</Variable>");
		return buffer.toString();
	}
	
}

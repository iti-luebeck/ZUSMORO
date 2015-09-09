/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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

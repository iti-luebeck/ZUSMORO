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

import java.util.ArrayList;
import java.util.LinkedList;

import model.bool.Variable.Operator;

import smachGenerator.ISmachableGuard;

public class HugeAnd extends BooleanExpression implements ISmachableGuard {

	private ArrayList<BooleanExpression> operands;

	public HugeAnd() {
		this.operands = new ArrayList<BooleanExpression>();
	}

	public void addOperand(BooleanExpression expr) {
		if (expr == null) {
			throw new IllegalArgumentException(
					"Boolean operants may not be null");
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
		return obj != null && obj instanceof HugeAnd
				&& operands.equals(((HugeAnd) obj).operands);
	}

	public void removeOperand(Variable v) {
		if (v == null) {
			throw new IllegalArgumentException(
					"Boolean operants may not be null");
		}
		this.operands.remove(v);
	}

	@Override
	public LinkedList<String> getSensorNames() {
		LinkedList<String> names = new LinkedList<>();
		for (BooleanExpression be : operands) {
			if (be instanceof Variable) {
				names.add(((Variable) be).getVariableName());
			}
		}
		return names;
	}

	@Override
	public LinkedList<String> getOperators() {
		LinkedList<String> operators = new LinkedList<>();
		for (BooleanExpression be : operands) {
			if (be instanceof Variable) {
				operators.add(((Variable) be).getOperator() + "");
			}
		}
		return operators;
	}

	@Override
	public LinkedList<Integer> getCompValues() {
		LinkedList<Integer> values = new LinkedList<>();
		for (BooleanExpression be : operands) {
			if (be instanceof Variable) {
				values.add(((Variable) be).getCompValue());
			}
		}
		return values;
	}
}

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
package model;

public class ActionController extends Action {

	private int offset;
	private int min;
	private int max;
	private int kpr;
	private int compvalue;
	private String var;

	public ActionController(String key) {
		super(key, 1);
	}

	public int getOffset() {
		return offset;
	}

	public int getMin() {
		return min;
	}

	public int getMax() {
		return max;
	}

	public int getKpr() {
		return kpr;
	}

	public int getCompvalue() {
		return compvalue;
	}

	public String getVar() {
		return var;
	}

	public void setOffset(int offset) {
		this.offset = offset;
	}

	public void setMin(int min) {
		this.min = min;
	}

	public void setMax(int max) {
		this.max = max;
	}

	public void setKpr(int kpr) {
		this.kpr = kpr;
	}

	public void setCompvalue(int compvalue) {
		this.compvalue = compvalue;
	}

	public void setVar(String var) {
		this.var = var;
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof ActionController) {
			ActionController ac = (ActionController) o;
			return getKey() == ac.getKey() && offset == ac.offset && min == ac.min && max == ac.max && kpr == ac.kpr
					&& var.equals(ac.var) && compvalue == ac.compvalue;
		} else {
			return false;
		}
	}

	@Override
	public String toString() {
		return getKey() + " " + offset + " " + min + " " + max + " " + kpr + " " + var + " " + compvalue;
	}

}

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

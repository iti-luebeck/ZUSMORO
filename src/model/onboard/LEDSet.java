package model.onboard;

import java.util.ArrayList;
import java.util.BitSet;

import smachGenerator.ISmachableAction;

import model.Action;

public class LEDSet {

	private BitSet leds;

	public LEDSet() {
		leds = new BitSet(11);
		leds.clear();
	}

	public int getValue() {
		int res = 0;
		int bitValue = 1;
		for (int i = 0; i < leds.size(); i++) {
			if (leds.get(i)) {
				res += bitValue;
			}
			bitValue *= 2;
		}
		return res;
	}

	public void set(ArrayList<Action> actions){
		for (ISmachableAction a : actions) {
			if (a.getKey().startsWith("LED")||a.getKey().equals("BEEP")) {
				leds.set(ledIndexOf(a.getKey()), a.getValue() == 1);
			}
		}

	}

	private int ledIndexOf(String key) {
		int res = -1;
		if (key.equalsIgnoreCase("ledfront")) {
			res = 8;
		} else if (key.equalsIgnoreCase("ledbody")) {
			res = 9;
		} else if (key.equalsIgnoreCase("beep")) {
			res = 10;
		} else {
			res = Character.getNumericValue(key.charAt(3));
		}

		return res;
	}

	public static int[] getLEDArray(int parseInt) {
		int[] res=new int[8];
		int temp=parseInt;
		for(int i=0;i<8;i++){
			res[i]=temp%2;
			temp>>=1;
		}
		return res;
	}


}

package org.vector8177.util;

public class VectorUtils {
  public static boolean isInBetween(double num, double lower, double upper, boolean inclusive) {
    return inclusive ? num >= lower && num <= upper : num > lower && num < upper;
  }
}

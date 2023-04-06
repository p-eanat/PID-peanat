#include "unity.h"
#include "Arduino.h"
#include "PID_peanat.h"


void setUp(void) {}
void tearDown(void) {}


void test_setup() {
  PID pid(1.0, 2.0, 3.0);
  TEST_ASSERT_EQUAL(1.0, pid.getkp());
  TEST_ASSERT_EQUAL(2.0, pid.getki());
  TEST_ASSERT_EQUAL(3.0, pid.getkd());

  pid.updateCoeffs(1.0, 0.0, 0.0);
  TEST_ASSERT_EQUAL(1.0, pid.getkp());
  TEST_ASSERT_EQUAL(0.0, pid.getki());
  TEST_ASSERT_EQUAL(0.0, pid.getkd());
}

void test_init_out() {
  PID pid(1.0, 2.0, 3.0);
  pid.setInitOutput(10.0);
  double init_val = pid.compute(1.0, 2.0);
  TEST_ASSERT_EQUAL(10.0, init_val);
}

void test_rev() {
  PID pid(1.0, 2.0, 3.0);
  pid.setReverse(true);
  pid.setReverse(true);
  TEST_ASSERT_EQUAL(-1.0, pid.getkp());
  TEST_ASSERT_EQUAL(-2.0, pid.getki());
  TEST_ASSERT_EQUAL(-3.0, pid.getkd());
}

void test_p() {
  PID pid(1.0, 0.0, 0.0);
  pid.setOutputBounds(0.0, 10.0);
  pid.setDeadband(5.0);
  
  // Initial output
  double val = pid.compute(2.0, 1.0);
  TEST_ASSERT_EQUAL(0.0, val);

  // Output should not change because it's within deadband
  delay(100);
  val = pid.compute(2.0, 1.0);
  TEST_ASSERT_EQUAL(0.0, val);

  // Output should not change because has not met sample time requirement
  pid.setSampleTime(200);
  delay(100);
  val = pid.compute(15.0, 2.0);
  TEST_ASSERT_EQUAL(0.0, val);

  // This time, should be max
  delay(100);
  val = pid.compute(15.0, 2.0);
  TEST_ASSERT_EQUAL(10.0, val);
}

void test_i() {
  PID pid(0.0, 10.0, 0.0);
  pid.setOutputBounds(0.0, 10.0);
  pid.setInitOutput(0.001);
  pid.compute(2.0, 1.0);

  delay(100);
  float val = pid.compute(2.0, 1.0);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, val);
  delay(900);
  float val = pid.compute(2.0, 1.0);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, val);

  // Output should not have changed
  delay(100);
  float val = pid.compute(2.0, 1.0);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, val);

  delay(100);
  float val = pid.compute(2.0, 1.0);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, val);


}


void setup() {
  delay(2000);

  UNITY_BEGIN();

  RUN_TEST(test_setup);
  RUN_TEST(test_init_out);
  RUN_TEST(test_rev);
  RUN_TEST(test_p);
  RUN_TEST(test_i);

  UNITY_END();
}

void loop() {}

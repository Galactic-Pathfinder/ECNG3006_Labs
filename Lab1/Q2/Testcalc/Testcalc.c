#include <stddef.h>
#include "unity.h"
#include "calc.h"
#include <limits.h>
#include <stdint.h>
//pio run -e CalcUnityTest -t upload

void test_sub()
{
	int output = sub(20, 10);
	TEST_ASSERT_EQUAL(10, output);
}

void test_mul()
{
	int output = mul(10, 2);
	TEST_ASSERT_EQUAL(20, output);
}

void test_mul_overflow()
{
	int output = mul(10000000, 999999999);
	TEST_ASSERT_EQUAL(-1, output);
}

void test_mul_underflow()
{
	int output = mul(-10000000, 99999999);
	TEST_ASSERT_EQUAL(-1, output);
}

void test_div()
{
	int output = divide(10, 0);
	TEST_ASSERT_EQUAL(0, output);
}

void test_mod()
{
	TEST_ASSERT_EQUAL(0, mod(10, 2));
	TEST_ASSERT_EQUAL(1, mod(10, 3));
}

void test_power_two()
{
	TEST_ASSERT_EQUAL(9, power_two(3));
}

void test_power_two_overflow()
{
	TEST_ASSERT_EQUAL(-1, power_two(1000000));
}

void test_power()
{
	TEST_ASSERT_EQUAL(8, power(2, 3));
}

void test_is_greater()
{
	TEST_ASSERT_TRUE(is_greater(20, 10));
	TEST_ASSERT_FALSE(is_greater(10, 20));
}



void test_is_smaller()
{
	TEST_ASSERT_TRUE(is_smaller(9, 10));
	TEST_ASSERT_FALSE(is_smaller(10, 9));
}

void test_fill_single_digit_positive_number()
{
	int expected_arr[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	size_t expected_size = sizeof(expected_arr) / sizeof(expected_arr[0]);

	int arr[9];
	size_t arr_size = sizeof(arr) / sizeof(arr[0]);

	fill_single_digit_positive_number(arr, arr_size);

	TEST_ASSERT_EQUAL_INT_ARRAY(expected_arr, arr, expected_size);

	TEST_ASSERT_EQUAL_MEMORY(expected_arr, arr, expected_size);
}

void test_true_as_string()
{
	TEST_ASSERT_EQUAL_STRING("true", true_as_string());
	TEST_ASSERT_EQUAL_STRING_LEN("truz", true_as_string(), 3);
	TEST_ASSERT_NOT_NULL(true_as_string());
	TEST_ASSERT_EQUAL_MEMORY("true", true_as_string(), 4);
}

void test_false_as_string()
{
	TEST_ASSERT_EQUAL_STRING_MESSAGE("false", false_as_string(),
					 "The output should be 'false'");
	TEST_ASSERT_EQUAL_STRING_LEN_MESSAGE("falze", false_as_string(), 3,
					     "First three characters should be 'tru'");
}


void setUp()
{

}

void tearDown()
{

}


void test_add(void)
{
	

    TEST_ASSERT_EQUAL_INT_MESSAGE(30, add(10, 20), 
        "Failed standard positive addition (10 + 20)");

    /* Both operands fit in a signed 8-bit int (max 127) but the sum does not.
       It fits in 16 bits (max 32767), so this only passes if int is >= 16 bits. */
    TEST_ASSERT_EQUAL_INT_MESSAGE(200, add(100, 100), 
        "Failed 8-bit overflow range addition (100 + 100)"); /* wraps to 56 on 8-bit int */
        
    TEST_ASSERT_EQUAL_INT_MESSAGE(254, add(127, 127), 
        "Failed boundary addition at edge of 8-bit signed int limit (127 + 127)"); /* wraps to -56 on 8-bit int */
        
    TEST_ASSERT_TRUE_MESSAGE(INT_MAX >= 32767, 
        "Environment restriction: int is narrower than 16 bits on this platform");
}

void test_is_equal(void)
{
    TEST_ASSERT_TRUE_MESSAGE(is_equal(10,10), 
        "Failed validation: identical positive integers should be evaluated as equal");
        
    TEST_ASSERT_FALSE_MESSAGE(is_equal(10, 5), 
        "Failed validation: mismatching positive integers should not be evaluated as equal");
}

void test_power(void)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(1024, power(2, 10), 
        "Failed base-2 exponentiation handling (2^10)");
        
    TEST_ASSERT_EQUAL_INT_MESSAGE(1000, power(10, 3), 
        "Failed base-10 exponentiation handling (10^3)");
}

void test_magnitude(void)
{
    /* if int were unsigned, -1 < 0 would be false */
    TEST_ASSERT_TRUE_MESSAGE((int)-1 < 0, 
        "Compiler representation error: int type is unexpected behavior or not signed");

    TEST_ASSERT_EQUAL_UINT_MESSAGE(5u, magnitude(-5), 
        "Failed absolute value conversion for a standard negative integer (-5)");
        
    TEST_ASSERT_EQUAL_UINT_MESSAGE(5u, magnitude(5), 
        "Failed identity conversion for a standard positive integer (5)");
        
    TEST_ASSERT_EQUAL_UINT_MESSAGE(0u, magnitude(0), 
        "Failed boundary handling for zero magnitude value (0)");
        
    TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)INT_MAX + 1u, magnitude(INT_MIN), 
        "Failed critical boundary handling: magnitude of absolute maximum negative limits (INT_MIN)");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_add);
    RUN_TEST(test_is_equal);
    RUN_TEST(test_power);
    RUN_TEST(test_magnitude);
    return UNITY_END();
}

int run_tests(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_add);
	RUN_TEST(test_sub);
	RUN_TEST(test_mul);
	RUN_TEST(test_mul_overflow);
	RUN_TEST(test_mul_underflow);
	RUN_TEST(test_div);
	RUN_TEST(test_mod);
	RUN_TEST(test_power_two);
	RUN_TEST(test_power_two_overflow);
	RUN_TEST(test_power);
	RUN_TEST(test_is_greater);
	RUN_TEST(test_is_equal);
	RUN_TEST(test_is_smaller);
	RUN_TEST(test_fill_single_digit_positive_number);
	RUN_TEST(test_true_as_string);
	RUN_TEST(test_false_as_string);
 	RUN_TEST(test_magnitude);

  	return (int)UNITY_END();
  
	
}

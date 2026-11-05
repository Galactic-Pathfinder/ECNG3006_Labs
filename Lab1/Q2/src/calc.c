#include <limits.h>

#include "calc.h"

int add(int x, int y)
{
	return x + y;
}

int sub(int x, int y)
{
	return x - y;
}

int mul(int x, int y)
{
	long long int result = (long long int)x * y;
	return result >= INT_MAX || result <= INT_MIN ? -1 : (int)result;
}

/* Renamed from div() to avoid clashing with the C library div() */
int divide(int x, int y)
{
	return (y == 0) ? 0 : x / y;
}

int mod(int x, int y)
{
	return x % y;
}

int power_two(int x)
{
	return power(x, 2);
}

int power(int x, int n)
{
	int out = x;
	for (int i = 1; i < n; i++) {
		out = mul(out, x);
	}
	return out;
}

bool is_equal(int x, int y)
{
	return x == y;
}

bool is_greater(int x, int y)
{
	return x > y;
}

bool is_smaller(int x, int y)
{
	return !is_equal(x, y) && !is_greater(x, y);
}

void fill_single_digit_positive_number(int arr[], size_t size)
{
	if ((int)size < 9) {
		return;
	}

	for (int i = 1; i <= 9; i++) {
		arr[i - 1] = i;
	}
}

const char *true_as_string(void)
{
	return "true";
}

const char *false_as_string(void)
{
	return "false";
}

/*
 * Returns the unsigned magnitude of x.
 * The conversion is done in unsigned arithmetic so it is also
 * correct (and well defined) for INT_MIN.
 */
unsigned int magnitude(int x)
{
	return (x < 0) ? (0u - (unsigned int)x) : (unsigned int)x;
}

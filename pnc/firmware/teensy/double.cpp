#include "integer_only.h"
#include "double.h"

Double Double::fromInteger(uint64_t magnitude, bool negative) {
  return normalized(magnitude, 0, negative);
}

Double operator+(Double a, Double b) {
  if (a.isZero()) return b;
  if (b.isZero()) return a;
  if (a.exponent() < b.exponent() ||
      (a.exponent() == b.exponent() && a.coefficient() < b.coefficient())) {
    const Double t = a; a = b; b = t;
  }
  const unsigned gap = unsigned(a.exponent() - b.exponent());
  const uint64_t left = a.coefficient() * 100;
  uint64_t right;
  bool sticky = false;
  if (gap <= 2) right = b.coefficient() * Double::power10(2 - gap);
  else if (gap <= 18) {
    const uint64_t divisor = Double::power10(gap - 2);
    right = b.coefficient() / divisor;
    sticky = b.coefficient() % divisor != 0;
  } else { right = 0; sticky = true; }
  const bool subtract = a.negative() != b.negative();
  // For subtraction, retain a lower bound and sticky remainder just as for
  // addition. This matters when a power-of-ten boundary loses a digit.
  const uint64_t result = subtract ? left - right - uint64_t(sticky) : left + right;
  return Double::normalized(result, a.exponent() - 2, a.negative(), sticky);
}

Double operator*(Double a, Double b) {
  if (a.isZero() || b.isZero()) return Double();
  // Base 10^8 limbs: every partial product and carry fits uint64_t on ARM32.
  // No compiler-specific 128-bit type or runtime wide-integer dependency.
  constexpr uint64_t BASE = 100000000;
  const uint64_t ac = a.coefficient(), bc = b.coefficient();
  const uint64_t a0 = ac % BASE, a1 = ac / BASE;
  const uint64_t b0 = bc % BASE, b1 = bc / BASE;
  const uint64_t p0 = a0 * b0;
  const uint64_t p1 = a0 * b1 + a1 * b0 + p0 / BASE;
  const uint64_t p2 = a1 * b1 + p1 / BASE;
  // Product has 31 or 32 digits. Keep 17 digits and a sticky remainder;
  // normalized() performs the one and only nearest-even rounding.
  const uint64_t tail = (p1 % BASE) * BASE + p0 % BASE;
  const bool short_product = p2 < Double::MINIMUM;
  const uint64_t divisor = short_product ? 100000000000000ULL : 1000000000000000ULL;
  const uint64_t leading = p2 * (short_product ? 100 : 10) + tail / divisor;
  return Double::normalized(leading,
      a.exponent() + b.exponent() + (short_product ? 14 : 15),
      a.negative() != b.negative(), tail % divisor != 0);
}

Double operator/(Double a, Double b) {
  if (b.isZero()) __builtin_trap();
  if (a.isZero()) return Double();
  const uint64_t divisor = b.coefficient();
  uint64_t remainder = a.coefficient();
  uint64_t quotient = remainder / divisor;
  remainder %= divisor;
  int exponent = a.exponent() - b.exponent();
  // Generate 17 significant digits. Remainder * 10 stays below 10^17.
  while (quotient < Double::LIMIT) {
    remainder *= 10;
    quotient = quotient * 10 + remainder / divisor;
    remainder %= divisor;
    --exponent;
  }
  return Double::normalized(quotient, exponent,
      a.negative() != b.negative(), remainder != 0);
}

bool operator<(Double a, Double b) {
  if (a == b) return false;
  if (a.negative() != b.negative()) return a.negative();
  if (a.isZero()) return !b.negative();
  if (b.isZero()) return a.negative();
  const bool magnitude_less = a.exponent() < b.exponent() ||
      (a.exponent() == b.exponent() && a.coefficient() < b.coefficient());
  return a.negative() ? !magnitude_less : magnitude_less;
}

Double Double::sqrt() const {
  if (negative()) __builtin_trap();
  if (isZero()) return *this;
  // Decimal longhand square root of coefficient * 10^shift (31/32 digits).
  // A 16-digit integer root and its exact remainder determine rounding.
  const int shift = exponent() % 2 == 0 ? 16 : 15;
  const uint64_t c = coefficient();
  uint64_t root = 0, remainder = 0;
  for (int pair = 15; pair >= 0; --pair) {
    unsigned digits = 0;
    for (int position = pair * 2 + 1; position >= pair * 2; --position) {
      const int index = position - shift;
      const unsigned digit = index >= 0 && index < 16
          ? unsigned((c / power10(unsigned(index))) % 10) : 0;
      digits = digits * 10 + digit;
    }
    remainder = remainder * 100 + digits;
    unsigned digit = 9;
    while ((20 * root + digit) * digit > remainder) --digit;
    remainder -= (20 * root + digit) * digit;
    root = root * 10 + digit;
  }
  if (remainder > root) ++root;
  return normalized(root, (exponent() - shift) / 2, false);
}

uint64_t Double::truncatedMagnitude() const {
  uint64_t c = coefficient();
  int e = exponent();
  if (e <= -16) return 0;
  if (e < 0) return c / power10(unsigned(-e));
  while (e--) {
    if (c > UINT64_MAX / 10) __builtin_trap();
    c *= 10;
  }
  return c;
}

int64_t Double::roundedInteger() const {
  uint64_t n = truncatedMagnitude();
  const int e = exponent();
  if (e < 0 && e >= -16) {
    const uint64_t divisor = power10(unsigned(-e));
    if (coefficient() % divisor >= divisor / 2) ++n;
  }
  const uint64_t limit = uint64_t(INT64_MAX) + uint64_t(negative());
  if (n > limit) __builtin_trap();
  return negative() ? -int64_t(n - (n != 0)) - int64_t(n != 0) : int64_t(n);
}

fixed_decimal_t Double::scientific() const {
  fixed_decimal_t out{};
  out.source_bits = encoding();
  out.whole = coefficient();
  out.negative = negative();
  out.exponent10 = int16_t(exponent());
  // Shorten exact powers of ten and zeros without changing the value.
  while (out.whole && out.whole % 10 == 0) { out.whole /= 10; ++out.exponent10; }
  return out;
}

fixed_decimal_t Double::fixed(int places) const {
  fixed_decimal_t out{};
  out.source_bits = encoding();
  if (places < 0) places = 0;
  if (places > FIXED_DECIMAL_MAX_PLACES) places = FIXED_DECIMAL_MAX_PLACES;
  out.decimal_places = uint8_t(places);
  uint64_t c = coefficient();
  int e = exponent();
  constexpr uint64_t MAX_WHOLE = 9000000000000000000ULL;
  if (*this > Double(MAX_WHOLE) || *this < -Double(MAX_WHOLE)) {
    out.status = fixed_decimal_status_t::OUT_OF_RANGE;
    return out;
  }
  if (e >= 0) {
    out.whole = truncatedMagnitude();
  } else {
    const unsigned shift = unsigned(-e);
    uint64_t remainder = c;
    if (shift < 16) {
      const uint64_t divisor = power10(shift);
      out.whole = c / divisor;
      remainder = c % divisor;
    }
    if (shift <= unsigned(places)) {
      out.fractional = remainder * power10(unsigned(places) - shift);
    } else {
      const unsigned discard = shift - unsigned(places);
      if (discard <= 16) {
        const uint64_t divisor = power10(discard);
        out.fractional = remainder / divisor;
        if (remainder % divisor >= divisor / 2) ++out.fractional;
      }
    }
    const uint64_t scale = power10(unsigned(places));
    if (out.fractional == scale) { out.fractional = 0; ++out.whole; }
  }
  if (out.whole > MAX_WHOLE) out.status = fixed_decimal_status_t::OUT_OF_RANGE;
  out.negative = negative() && (out.whole || out.fractional);
  return out;
}

fixed_decimal_t toFixedDecimal(Double value, int places) { return value.fixed(places); }
fixed_decimal_t toScientificDecimal(Double value) { return value.scientific(); }

const char* fixedDecimalStatusName(fixed_decimal_status_t status) {
  switch (status) {
    case fixed_decimal_status_t::VALID: return "VALID";
    case fixed_decimal_status_t::NAN_VALUE: return "NAN";
    case fixed_decimal_status_t::POSITIVE_INFINITY: return "POSITIVE_INFINITY";
    case fixed_decimal_status_t::NEGATIVE_INFINITY: return "NEGATIVE_INFINITY";
    case fixed_decimal_status_t::OUT_OF_RANGE: return "OUT_OF_RANGE";
    default: return "UNKNOWN";
  }
}

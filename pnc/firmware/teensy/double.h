#pragma once

#include <stdint.h>
#include <type_traits>

#if defined(__arm__) && defined(__GNUC__)
#pragma GCC push_options
#pragma GCC target("general-regs-only")
#endif

struct fixed_decimal_t;

// Finite decimal arithmetic for firmware. Two integer words, eight bytes;
// 16 significant decimal digits, rounded to nearest with ties to even.
// No allocation, native FP conversions, NaN, infinity, or mutable global state.
// A nonzero value is coefficient * 10^exponent, with a normalized 16-digit
// coefficient and exponent in [-398, 369]. Zero has the all-zero representation.
// This is a decimal replacement for the operations we use, not binary64 emulation.
class alignas(8) Double {
 public:
  constexpr Double() : low_(0), high_(0) {}

  template<class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  constexpr Double(T value) : low_(0), high_(0) {
    const bool neg = std::is_signed<T>::value && value < 0;
    const uint64_t magnitude = neg ? uint64_t(0) - uint64_t(value) : uint64_t(value);
    if (__builtin_is_constant_evaluated() || __builtin_constant_p(value))
      *this = normalized(magnitude, 0, neg);
    else
      *this = fromInteger(magnitude, neg);
  }

  // Literal construction is constexpr and does not pass through a native real.
  static constexpr Double decimal(const char* text) {
    Double out;
    if (!tryParse(text, out)) __builtin_trap();
    return out;
  }

  // External-input boundary: reject malformed/out-of-range tokens without
  // changing out. Canonical scientific output round-trips exactly.
  static __attribute__((noinline)) constexpr bool tryParse(const char* text, Double& out) {
    if (!text || !*text) return false;
    bool neg = false;
    if (*text == '-' || *text == '+') { neg = *text == '-'; ++text; }
    uint64_t coefficient = 0;
    int significant = 0, fractional = 0, dropped = 0;
    unsigned guard = 0;
    bool sticky = false, point = false, digits = false, nonzero = false;
    while ((*text >= '0' && *text <= '9') || *text == '.') {
      if (*text == '.') {
        if (point) return false;
        point = true;
        ++text;
        continue;
      }
      digits = true;
      const unsigned digit = unsigned(*text++ - '0');
      if (point && ++fractional > 1000) return false;
      nonzero = nonzero || digit != 0;
      if (!nonzero) continue;
      if (significant < 16) { coefficient = coefficient * 10 + digit; ++significant; }
      else {
        if (dropped == 0) guard = digit;
        else sticky = sticky || digit != 0;
        if (++dropped > 1000) return false;
      }
    }
    if (!digits) return false;
    int exponent = 0;
    if (*text == 'e' || *text == 'E') {
      ++text;
      bool exp_negative = false;
      if (*text == '-' || *text == '+') { exp_negative = *text == '-'; ++text; }
      if (*text < '0' || *text > '9') return false;
      while (*text >= '0' && *text <= '9') {
        exponent = exponent * 10 + (*text++ - '0');
        if (exponent > 1000) return false;
      }
      if (exp_negative) exponent = -exponent;
    }
    if (*text) return false;
    if (!nonzero) { out = Double(); return true; }
    exponent += dropped - fractional;
    if (guard > 5 || (guard == 5 && (sticky || (coefficient & 1)))) ++coefficient;
    if (coefficient == LIMIT) { coefficient /= 10; ++exponent; }
    while (coefficient < MINIMUM) { coefficient *= 10; --exponent; }
    if (exponent < MIN_EXPONENT || exponent > MAX_EXPONENT) return false;
    out = pack(coefficient, exponent, neg);
    return true;
  }

  constexpr bool negative() const { return (high_ >> 31) != 0; }
  constexpr bool isZero() const { return (low_ | high_) == 0; }
  constexpr uint64_t coefficient() const {
    const uint64_t bits = encoding();
    return (high_ & 0x60000000U) == 0x60000000U
        ? (bits & ((uint64_t(1) << 51) - 1)) | (uint64_t(1) << 53)
        : bits & ((uint64_t(1) << 53) - 1);
  }
  constexpr int exponent() const {
    return isZero() ? 0 : int(((high_ & 0x60000000U) == 0x60000000U
        ? encoding() >> 51 : encoding() >> 53) & 1023U) - 398;
  }
  constexpr uint64_t encoding() const { return (uint64_t(high_) << 32) | low_; }

  template<class T, typename std::enable_if<std::is_integral<T>::value &&
      !std::is_same<T, bool>::value, int>::type = 0>
  explicit operator T() const {
    const uint64_t n = truncatedMagnitude();
    const uint64_t max_positive = std::is_signed<T>::value
        ? (uint64_t(1) << (sizeof(T) * 8 - 1)) - 1
        : uint64_t(T(~T(0)));
    if ((!std::is_signed<T>::value && negative()) ||
        n > max_positive + uint64_t(negative())) __builtin_trap();
    // Avoid a signed negation of INT64_MIN.
    return negative() ? T(-T(n - (n != 0)) - T(n != 0)) : T(n);
  }
  explicit constexpr operator bool() const { return !isZero(); }

  constexpr Double operator-() const {
    return isZero() ? *this : fromEncoding(encoding() ^ (uint64_t(1) << 63));
  }
  Double& operator+=(Double b) { return *this = *this + b; }
  Double& operator-=(Double b) { return *this = *this - b; }
  Double& operator*=(Double b) { return *this = *this * b; }
  Double& operator/=(Double b) { return *this = *this / b; }

  friend Double operator+(Double a, Double b);
  friend Double operator-(Double a, Double b) { return a + (-b); }
  friend Double operator*(Double a, Double b);
  friend Double operator/(Double a, Double b);
  friend constexpr bool operator==(Double a, Double b) { return a.encoding() == b.encoding(); }
  friend constexpr bool operator!=(Double a, Double b) { return !(a == b); }
  friend bool operator<(Double a, Double b);
  friend bool operator>(Double a, Double b) { return b < a; }
  friend bool operator<=(Double a, Double b) { return !(b < a); }
  friend bool operator>=(Double a, Double b) { return !(a < b); }

  Double sqrt() const;
  int64_t roundedInteger() const; // half away from zero, matching llround
  fixed_decimal_t fixed(int decimal_places) const;
  fixed_decimal_t scientific() const;

 private:
  uint32_t low_;
  uint32_t high_;
  static constexpr uint64_t MINIMUM = 1000000000000000ULL;
  static constexpr uint64_t LIMIT = 10000000000000000ULL;
  static constexpr int MIN_EXPONENT = -398;
  static constexpr int MAX_EXPONENT = 369;

  static constexpr uint64_t power10(unsigned n) {
    uint64_t v = 1;
    while (n--) v *= 10;
    return v;
  }
  static constexpr Double fromEncoding(uint64_t bits) {
    Double out;
    out.low_ = uint32_t(bits);
    out.high_ = uint32_t(bits >> 32);
    return out;
  }
  static constexpr Double pack(uint64_t c, int e, bool neg) {
    if (!c) return Double();
    const uint64_t biased = uint64_t(e + 398);
    const uint64_t bits = c < (uint64_t(1) << 53)
        ? (biased << 53) | c
        : (uint64_t(3) << 61) | (biased << 51) | (c & ((uint64_t(1) << 51) - 1));
    return fromEncoding(bits | (uint64_t(neg) << 63));
  }
  static constexpr Double normalized(uint64_t c, int e, bool neg, bool sticky = false) {
    if (!c) return Double();
    if (c >= LIMIT) {
      uint64_t divisor = 1;
      while (c / divisor >= LIMIT) { divisor *= 10; ++e; }
      const uint64_t remainder = c % divisor;
      c /= divisor;
      if (remainder > divisor / 2 ||
          (remainder == divisor / 2 && (sticky || (c & 1)))) ++c;
      if (c == LIMIT) { c /= 10; ++e; }
    }
    while (c < MINIMUM) { c *= 10; --e; }
    if (e < MIN_EXPONENT || e > MAX_EXPONENT) __builtin_trap();
    return pack(c, e, neg);
  }
  static Double fromInteger(uint64_t magnitude, bool negative);
  uint64_t truncatedMagnitude() const;
};

// Raw numeric literal: the compiler passes characters, never an FP value.
// A constexpr local forces parsing during compilation even in runtime code.
template<char... Chars>
constexpr Double operator""_D() {
  constexpr char text[] = {Chars..., '\0'};
  constexpr Double value = Double::decimal(text);
  return value;
}

static_assert(sizeof(Double) == 8, "Double must preserve history buffer sizes");
static_assert(std::is_trivially_copyable<Double>::value, "Snapshots require trivial copies");

// Compatibility helpers for existing statistical expressions. Every Double is
// finite by construction; invalid external input is rejected by tryParse.
inline Double sqrt(Double value) { return value.sqrt(); }
inline int64_t llround(Double value) { return value.roundedInteger(); }
inline constexpr bool isfinite(Double) { return true; }

static constexpr uint8_t FIXED_DECIMAL_MAX_PLACES = 12U;
enum class fixed_decimal_status_t : uint8_t {
  VALID = 0, NAN_VALUE = 1, POSITIVE_INFINITY = 2, NEGATIVE_INFINITY = 3, OUT_OF_RANGE = 4,
};
struct fixed_decimal_t {
  uint64_t whole;
  uint64_t fractional;
  uint64_t source_bits; // Double integer encoding, not IEEE binary64 evidence.
  uint8_t decimal_places;
  uint8_t negative;
  fixed_decimal_status_t status;
  int16_t exponent10 = 0;
  bool valid() const { return status == fixed_decimal_status_t::VALID; }
};

// Preserve the existing publication ABI and call sites.
fixed_decimal_t toFixedDecimal(Double value, int decimal_places);
fixed_decimal_t toScientificDecimal(Double value);
const char* fixedDecimalStatusName(fixed_decimal_status_t status);

#if defined(__arm__) && defined(__GNUC__)
#pragma GCC pop_options
#endif

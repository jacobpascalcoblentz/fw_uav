#include <gtest/gtest.h>
#include "fw_uav/core/result.h"

using namespace fw_uav;

TEST(ResultTest, SuccessConstruction) {
    Result<int> r(42);
    EXPECT_TRUE(r.is_ok());
    EXPECT_FALSE(r.is_error());
    EXPECT_EQ(r.value(), 42);
}

TEST(ResultTest, ErrorConstruction) {
    Result<int> r(ErrorCode::InvalidParameter);
    EXPECT_FALSE(r.is_ok());
    EXPECT_TRUE(r.is_error());
    EXPECT_EQ(r.error(), ErrorCode::InvalidParameter);
}

TEST(ResultTest, BoolConversion) {
    Result<int> ok(42);
    Result<int> err(ErrorCode::InvalidParameter);
    EXPECT_TRUE(static_cast<bool>(ok));
    EXPECT_FALSE(static_cast<bool>(err));
}

TEST(ResultTest, ValueOr) {
    Result<int> ok(42);
    Result<int> err(ErrorCode::InvalidParameter);
    EXPECT_EQ(ok.value_or(0), 42);
    EXPECT_EQ(err.value_or(0), 0);
}

TEST(ResultTest, ErrorStr) {
    Result<int> r(ErrorCode::GPSNoFix);
    EXPECT_STREQ(r.error_str(), "No GPS fix");
}

TEST(ResultTest, Map) {
    Result<int> ok(10);
    auto mapped = ok.map([](int x) { return x * 2; });
    EXPECT_TRUE(mapped.is_ok());
    EXPECT_EQ(mapped.value(), 20);

    Result<int> err(ErrorCode::InvalidParameter);
    auto mapped_err = err.map([](int x) { return x * 2; });
    EXPECT_TRUE(mapped_err.is_error());
    EXPECT_EQ(mapped_err.error(), ErrorCode::InvalidParameter);
}

TEST(ResultTest, AndThen) {
    auto doubleIfPositive = [](int x) -> Result<int> {
        if (x > 0) return x * 2;
        return ErrorCode::InvalidParameter;
    };

    Result<int> ok(10);
    auto chained = ok.and_then(doubleIfPositive);
    EXPECT_TRUE(chained.is_ok());
    EXPECT_EQ(chained.value(), 20);

    Result<int> negative(-5);
    auto chained_neg = negative.and_then(doubleIfPositive);
    EXPECT_TRUE(chained_neg.is_error());
}

TEST(ResultTest, VoidResult) {
    Result<void> ok = Result<void>::ok();
    EXPECT_TRUE(ok.is_ok());

    Result<void> err(ErrorCode::NotImplemented);
    EXPECT_TRUE(err.is_error());
    EXPECT_EQ(err.error(), ErrorCode::NotImplemented);
}

TEST(ResultTest, OkHelper) {
    auto r = Ok(42);
    EXPECT_TRUE(r.is_ok());
    EXPECT_EQ(r.value(), 42);

    auto v = Ok();
    EXPECT_TRUE(v.is_ok());
}

TEST(ResultTest, ErrHelper) {
    auto r = Err<int>(ErrorCode::Timeout);
    EXPECT_TRUE(r.is_error());
    EXPECT_EQ(r.error(), ErrorCode::Timeout);

    auto v = Err(ErrorCode::Timeout);
    EXPECT_TRUE(v.is_error());
}

TEST(ResultTest, MoveSemantics) {
    struct MoveOnly {
        int value;
        MoveOnly(int v) : value(v) {}
        MoveOnly(const MoveOnly&) = delete;
        MoveOnly(MoveOnly&& o) : value(o.value) { o.value = 0; }
        MoveOnly& operator=(const MoveOnly&) = delete;
        MoveOnly& operator=(MoveOnly&&) = default;
    };

    Result<MoveOnly> r(MoveOnly(42));
    EXPECT_TRUE(r.is_ok());
    EXPECT_EQ(r.value().value, 42);
}

TEST(ErrorCodeTest, AllErrorsHaveStrings) {
    // Sample error codes to verify string coverage
    EXPECT_STREQ(error_code_str(ErrorCode::Ok), "Ok");
    EXPECT_STREQ(error_code_str(ErrorCode::IMUReadFailed), "IMU read failed");
    EXPECT_STREQ(error_code_str(ErrorCode::ControlNotInitialized), "Controller not initialized");
    EXPECT_STREQ(error_code_str(ErrorCode::RCLost), "RC signal lost");
    EXPECT_STREQ(error_code_str(ErrorCode::OutOfMemory), "Out of memory");
}

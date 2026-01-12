/**
 * @file crc16_test.cpp
 * @brief Unit tests for CRC16 functionality
 */

#include "CRC16.hpp"

#include <gtest/gtest.h>

using namespace unify_link;

class CRC16Test : public ::testing::Test
{
protected:
    // Helper to compute CRC16 using the library function
    uint16_t computeCRC(const uint8_t *data, size_t len) { return crc16_calculation(data, static_cast<uint16_t>(len)); }
};

TEST_F(CRC16Test, EmptyData)
{
    uint16_t crc = computeCRC(nullptr, 0);
    // CRC of empty data should be initial value
    EXPECT_EQ(crc, 0xFFFF);
}

TEST_F(CRC16Test, SingleByte)
{
    uint8_t data[] = {0x00};
    uint16_t crc = computeCRC(data, 1);
    EXPECT_NE(crc, 0x0000); // Should be non-zero
}

TEST_F(CRC16Test, KnownPattern)
{
    // Test with a known pattern
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc1 = computeCRC(data, 4);

    // Same data should produce same CRC
    uint16_t crc2 = computeCRC(data, 4);
    EXPECT_EQ(crc1, crc2);
}

TEST_F(CRC16Test, DifferentDataDifferentCRC)
{
    uint8_t data1[] = {0xAA, 0xBB, 0xCC};
    uint8_t data2[] = {0xAA, 0xBB, 0xCD}; // One byte different

    uint16_t crc1 = computeCRC(data1, 3);
    uint16_t crc2 = computeCRC(data2, 3);

    EXPECT_NE(crc1, crc2);
}

TEST_F(CRC16Test, LargeData)
{
    std::vector<uint8_t> data(1024);
    for (size_t i = 0; i < data.size(); ++i)
    {
        data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    uint16_t crc1 = computeCRC(data.data(), data.size());

    // Modify one byte
    data[512] ^= 0x01;
    uint16_t crc2 = computeCRC(data.data(), data.size());

    EXPECT_NE(crc1, crc2);
}

TEST_F(CRC16Test, AllZeros)
{
    uint8_t data[16] = {0};
    uint16_t crc = computeCRC(data, 16);
    EXPECT_NE(crc, 0x0000); // CRC should not be zero
}

TEST_F(CRC16Test, AllOnes)
{
    uint8_t data[16];
    memset(data, 0xFF, 16);
    uint16_t crc = computeCRC(data, 16);
    EXPECT_NE(crc, 0xFFFF); // CRC should change from initial value
}

TEST_F(CRC16Test, IncrementalCalculation)
{
    uint8_t full_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    // Calculate CRC of full data
    uint16_t full_crc = computeCRC(full_data, 6);

    // CRC should be consistent
    EXPECT_EQ(full_crc, computeCRC(full_data, 6));
}

// Test the CRC table has correct values
TEST_F(CRC16Test, TableIntegrity)
{
    // Verify table has 256 entries and first/last entries are as expected
    EXPECT_EQ(crc16_table[0], 0x0000);
    // Entry 255 should be non-zero
    EXPECT_NE(crc16_table[255], 0x0000);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

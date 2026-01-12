/**
 * @file update_link_test.cpp
 * @brief Unit tests for Update_Link component
 */

#include "unify_link.hpp"
#include "update_Link.hpp"

#include <gtest/gtest.h>

using namespace unify_link;

class UpdateLinkTest : public ::testing::Test
{
protected:
    static constexpr uint32_t BUFFER_SIZE = 4096;
    Unify_link_base link_base;
    Update_Link_t *update_link;

    void SetUp() override { update_link = new Update_Link_t(link_base); }

    void TearDown() override { delete update_link; }

    void roundTrip()
    {
        uint8_t frame[2048];
        uint32_t len = 0;
        link_base.send_buff_pop(frame, &len);
        link_base.rev_data_push(frame, len);
        link_base.parse_data_task();
    }
};

TEST_F(UpdateLinkTest, ComponentId)
{
    EXPECT_EQ(Update_Link_t::component_id, COMPONENT_ID_UPDATE);
}

TEST_F(UpdateLinkTest, DataIds)
{
    EXPECT_EQ(Update_Link_t::FIRMWARE_INFO_ID, 1);
    EXPECT_EQ(Update_Link_t::FIRMWARE_CRC_ID, 2);
}

TEST_F(UpdateLinkTest, FirmwareCrcRoundTrip)
{
    Update_Link_t::firmware_crc_t sent = {.crc16 = 0xBEEF};

    update_link->send_firmware_crc(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(update_link->firmware_crc.crc16, sent.crc16);
}

TEST_F(UpdateLinkTest, FirmwareInfoRoundTrip)
{
    Update_Link_t::firmware_info_t sent{};
    for (size_t i = 0; i < sizeof(sent.firmware_data); ++i)
    {
        sent.firmware_data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    update_link->send_firmware_info(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(std::memcmp(update_link->firmware_info.firmware_data, sent.firmware_data, sizeof(sent.firmware_data)), 0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

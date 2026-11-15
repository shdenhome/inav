#include "gtest/gtest.h"
#include "unittest_macros.h"

#include <iostream>
#include <string>
#include <cstdint>

extern "C" {
#include "io/osd.h"
#include "io/osd_utils.h"
};


TEST(OSDTest, TestCentiNumber)
{
   //bool osdFormatCentiNumber(char *buff, int32_t centivalue, uint32_t scale, int maxDecimals, int maxScaledDecimals, int length);
   char buf[11] = "0123456789";

   osdFormatCentiNumber(buf, 12345, 1, 2, 3, 7, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, " 123.45"));

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, 12345, 1, 2, 2, 6, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, "123.45"));

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, 12345, 1, 2, 2, 5, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, "123.4"));

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, 12345, 1, 2, 2, 4, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, " 123")); // this should be causing #8769

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, 12345, 1, 2, 2, 3, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, "123"));
   std::cout << "'" << buf << "'" << std::endl;

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, -12345, 1, 2, 2, 8, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, " -123.45"));



   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, -12345, 1, 2, 2, 7, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, "-123.45"));

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, -12345, 1, 2, 2, 6, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, "-123.4"));

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, -12345, 1, 2, 2, 5, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, " -123"));

   memset(buf, 0, 11);
   osdFormatCentiNumber(buf, -12345, 1, 2, 2, 4, false);
   std::cout << "'" << buf << "'" << std::endl;
   EXPECT_FALSE(strcmp(buf, "-123"));

   EXPECT_EQ(1, 1);

}

TEST(OSDTest, MigratesLayoutPositionsWhenMztcSlotIsAdded)
{
   constexpr uint16_t oldItemCount = OSD_ITEM_COUNT - 1;
   uint16_t oldPositions[OSD_LAYOUT_COUNT][oldItemCount];
   osdLayoutsConfig_t migrated = {};

   for (uint8_t layout = 0; layout < OSD_LAYOUT_COUNT; layout++) {
      for (uint16_t item = 0; item < oldItemCount; item++) {
         oldPositions[layout][item] = layout * oldItemCount + item + 1;
      }
   }

   ASSERT_TRUE(osdMigrateLayoutPositionsForInsertedItem(&migrated.item_pos[0][0],
      &oldPositions[0][0], OSD_LAYOUT_COUNT, oldItemCount, OSD_MZTC_STATUS_RESERVED));

   for (uint8_t layout = 0; layout < OSD_LAYOUT_COUNT; layout++) {
      for (uint16_t item = 0; item < OSD_MZTC_STATUS_RESERVED; item++) {
         EXPECT_EQ(oldPositions[layout][item], migrated.item_pos[layout][item]);
      }

      EXPECT_EQ(0, migrated.item_pos[layout][OSD_MZTC_STATUS_RESERVED]);
      for (uint16_t oldItem = OSD_MZTC_STATUS_RESERVED; oldItem < oldItemCount; oldItem++) {
         EXPECT_EQ(oldPositions[layout][oldItem], migrated.item_pos[layout][oldItem + 1]);
      }
   }

   EXPECT_FALSE(osdMigrateLayoutPositionsForInsertedItem(&migrated.item_pos[0][0],
      &oldPositions[0][0], OSD_LAYOUT_COUNT, oldItemCount, oldItemCount + 1));
}

#include "gtest/gtest.h"
#include "IOExpanderUtils.h"

TEST (IOExpanderUtilsTest, getBit){
    bool expected[2][8] = {{0, 1, 0, 1, 0, 1, 0, 1}, {1, 0, 1, 0, 1, 0, 1, 0}};  // 0101010110101010
    for(int i=0; i<8; i++){
        ASSERT_EQ(expected[1][7-i], IOExpanderUtils::getBit(0x55AA, 0, i));
    }
    for(int i=0; i<8; i++){
        ASSERT_EQ(expected[0][7-i], IOExpanderUtils::getBit(0x55AA, 1, i));
    }
}